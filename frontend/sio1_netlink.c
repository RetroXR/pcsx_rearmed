/*
 * (C) 2026 PCSX-ReARMed contributors
 *
 * This work is licensed under the terms of the GNU GPL version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include <stdlib.h>
#include <string.h>

#include "sio1_netlink.h"
#include "../libpcsxcore/psxcommon.h"
#include "../libpcsxcore/r3000a.h"
#include "../libpcsxcore/sio1.h"

/* Peers whose protocol id differs are never joined, which is what keeps this
 * cable out of a socket meant for something else. */
#define NETLINK_PROTOCOL "psx-sio-1"

/* A link cable has two ends. The bus is protocol-agnostic and would happily
 * join a third console; refusing it here is cheaper than working out what a
 * three-way null modem would even mean. */
#define NETLINK_MAX_PEERS 2

/* Bus message, packed by hand. Both ends are the same build today, but
 * protocol_id exists so that something else could speak this later, and by then
 * a shared struct layout would be an assumption nobody remembers making. */
enum {
	NL_BYTE = 1,   /* one byte off the wire, stamped when it finishes arriving */
	NL_LINES       /* the sender's DTR and RTS, stamped at its commit horizon */
};
#define NL_MSG_SIZE 8

/* Local line bits, as CTRL orders them. The cable crosses both pairs, so a
 * peer's DTR is read as DSR here and its RTS as CTS. */
#define NL_DTR 1
#define NL_RTS 2

/* Events waiting for this console's clock to reach them.
 *
 * Everything the bus hands over is held here rather than applied on arrival.
 * The bus delivers a message as soon as it exists, which is up to a commit
 * horizon before the moment it is stamped for, and WHEN it arrives depends on
 * how two emulation threads happened to interleave. Releasing by tick instead
 * makes the moment a peer's byte lands a function of emulated time alone, which
 * is what rollback and lockstep multiplayer need it to be.
 *
 * Deep enough that it cannot fill: a byte occupies the wire for far longer than
 * the horizon, so only a handful can ever be in flight at once. */
#define NL_PENDING_MAX 64

struct nl_event {
	uint64_t tick;
	/* Arrival order, which is what settles a tie.
	 *
	 * Two bytes can carry the SAME tick: the rendezvous interval has a floor,
	 * so at a fast enough baud rate more than one byte's time fits inside it,
	 * and everything a console originates in that window is stamped no earlier
	 * than the horizon it has already promised. Ordering by tick alone then
	 * leaves the tie to whatever the queue happens to look like, and a serial
	 * port that hands its bytes over in the wrong order is not a serial port. */
	uint64_t seq;
	u8 type;
	u8 value;
};

static const struct retro_link_interface *nl_link;
static retro_link_port_t *nl_handle;

static int nl_attached;

static int nl_self_id;
static unsigned nl_peers;

/* A monotonic 64-bit clock accumulated from psxRegs.cycle.
 *
 * psxRegs.cycle is 32 bits and wraps roughly every two minutes of emulated
 * time, which the bus would read as the console jumping four billion cycles
 * backwards. It also restarts at a reset and moves arbitrarily at a state load.
 * Only forward steps are accumulated, so the figure published to the bus can
 * never go back however the counter behind it moves. */
static u32 nl_last_raw;
static int nl_have_raw;
static uint64_t nl_now;

/* The furthest tick published as this console's commit horizon. Nothing may be
 * stamped before it, because peers have already been allowed to run there. */
static uint64_t nl_safe;

/* The last tick anything was stamped for.
 *
 * A byte originated while the rendezvous interval is coarse waits for the
 * horizon; the next byte, sent once the interval has gone fine, would carry an
 * EARLIER tick and overtake it. Bytes leave a serial port in the order they were
 * written and arrive in that order, so a stamp never goes backwards. */
static uint64_t nl_last_stamp;

static u8 nl_lines;
static int nl_lines_published;
static u8 nl_peer_lines;
static int nl_peer_lines_seen;

static struct nl_event nl_pending[NL_PENDING_MAX];
static unsigned nl_pending_count;
static uint64_t nl_next_seq;

static uint64_t nl_clock(void)
{
	u32 raw = psxRegs.cycle;
	s32 delta;

	if (!nl_have_raw) {
		nl_have_raw = 1;
		nl_last_raw = raw;
		return nl_now;
	}

	delta = (s32)(raw - nl_last_raw);
	nl_last_raw = raw;
	if (delta > 0)
		nl_now += (u32)delta;
	return nl_now;
}

static void nl_send(uint64_t tick, u8 type, u8 value)
{
	u8 msg[NL_MSG_SIZE];

	if (!nl_attached)
		return;
	if (tick < nl_safe)
		tick = nl_safe;
	if (tick < nl_last_stamp)
		tick = nl_last_stamp;
	nl_last_stamp = tick;

	memset(msg, 0, sizeof(msg));
	msg[0] = type;
	msg[1] = (u8)nl_self_id;
	msg[4] = value;
	nl_link->send(nl_handle, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

static void nl_publish_lines(void)
{
	nl_lines_published = 1;
	nl_send(nl_clock(), NL_LINES, nl_lines);
}

static void nl_forget_peer(void)
{
	nl_pending_count = 0;
	nl_peer_lines = 0;
	nl_peer_lines_seen = 0;
	nl_lines_published = 0;
	sio1SetPeerLines(0, 0);
}

static void nl_refresh_peers(void)
{
	unsigned was = nl_peers;
	unsigned count = 0;
	int id;

	if (!nl_attached) {
		nl_peers = 0;
		nl_self_id = 0;
		if (was >= 2)
			nl_forget_peer();
		return;
	}

	id = nl_link->peers(nl_handle, &count);
	if (id < 0 || count > NETLINK_MAX_PEERS) {
		if (count > NETLINK_MAX_PEERS)
			SysPrintf("sio1: %u consoles on the link; a PlayStation cable joins %d\n",
				count, NETLINK_MAX_PEERS);
		nl_self_id = 0;
		nl_peers = 0;
	}
	else {
		nl_self_id = id;
		nl_peers = count;
	}

	if (nl_peers == was)
		return;

	if (nl_peers < 2) {
		nl_forget_peer();
		return;
	}

	/* A cable has just been seated. Whatever either console heard before is
	 * from a bus that no longer exists, so both ends start listening again. */
	nl_peer_lines_seen = 0;
	nl_peer_lines = 0;
	sio1SetPeerLines(0, 0);
}

static void nl_queue(uint64_t tick, u8 type, u8 value)
{
	if (nl_pending_count >= NL_PENDING_MAX) {
		SysPrintf("sio1: link backlog full, dropping\n");
		return;
	}
	nl_pending[nl_pending_count].tick = tick;
	nl_pending[nl_pending_count].seq = nl_next_seq++;
	nl_pending[nl_pending_count].type = type;
	nl_pending[nl_pending_count].value = value;
	nl_pending_count++;
}

static void nl_apply(const struct nl_event *ev)
{
	switch (ev->type) {
	case NL_BYTE:
		sio1Receive(ev->value);
		break;
	case NL_LINES: {
		int first = !nl_peer_lines_seen;

		nl_peer_lines = ev->value;
		nl_peer_lines_seen = 1;
		/* Crossed: the peer's DTR arrives here as DSR, its RTS as CTS. */
		sio1SetPeerLines(!!(nl_peer_lines & NL_DTR), !!(nl_peer_lines & NL_RTS));

		/* Answer the first one, and only the first. Hearing from the peer is
		 * proof that it has a timeline for a message to be placed on, which is
		 * exactly what an earlier announcement of ours may have lacked.
		 * Answering every one instead makes the two consoles greet each other
		 * for the rest of the session. */
		if (first)
			nl_publish_lines();
		break;
	}
	}
}

/* Hand over everything whose moment has come, oldest first.
 *
 * Scanned for the earliest rather than taken off the front, because the two
 * kinds of message are not stamped alike: a byte is stamped a whole byte-time
 * out and a line change only a commit horizon out, so a line change published
 * after a byte can be due before it. */
static void nl_release(void)
{
	uint64_t now = nl_clock();

	for (;;) {
		unsigned i, best = NL_PENDING_MAX;

		for (i = 0; i < nl_pending_count; i++) {
			if (nl_pending[i].tick > now)
				continue;
			if (best == NL_PENDING_MAX ||
			    nl_pending[i].tick < nl_pending[best].tick ||
			    (nl_pending[i].tick == nl_pending[best].tick &&
			     nl_pending[i].seq < nl_pending[best].seq))
				best = i;
		}
		if (best == NL_PENDING_MAX)
			return;

		{
			struct nl_event ev = nl_pending[best];
			nl_pending[best] = nl_pending[--nl_pending_count];
			nl_apply(&ev);
		}
	}
}

static void nl_drain(void)
{
	u8 msg[NL_MSG_SIZE];
	uint64_t tick;
	unsigned from;
	size_t len = sizeof(msg);

	while (nl_link->recv(nl_handle, &tick, &from, msg, &len)) {
		if (len == NL_MSG_SIZE && (msg[0] == NL_BYTE || msg[0] == NL_LINES))
			nl_queue(tick, msg[0], msg[4]);
		len = sizeof(msg);
	}
}

/* ── the driver libpcsxcore sees ──────────────────────────────────────────── */

static void nl_drv_lines(int dtr, int rts)
{
	u8 was = nl_lines;

	nl_lines = (dtr ? NL_DTR : 0) | (rts ? NL_RTS : 0);
	if (nl_lines != was || !nl_lines_published)
		nl_publish_lines();
}

static void nl_drv_tx(unsigned char data, u32 cycles)
{
	/* Stamped when the byte finishes arriving, not when it left. The receiving
	 * console clocks it in over the same number of bit-times the sender clocks
	 * it out, so that is the moment its own RX FIFO fills. */
	nl_send(nl_clock() + cycles, NL_BYTE, data);
}

static u32 nl_drv_poll(u32 grain, u32 horizon)
{
	uint64_t now, grant;

	if (!nl_attached)
		return grain;

	now = nl_clock();
	nl_safe = now + horizon;

	/* Published before reading, because a peer parked on this console's horizon
	 * cannot move until it has been told the horizon moved, and it may be
	 * holding the very message this console is about to want.
	 *
	 * And before anything is SENT, because this call is what anchors the
	 * timeline the bus places a message on. A cable being seated re-anchors
	 * every console on it, and a message put on the wire between that moment
	 * and this console's next rendezvous carries an offset from an origin that
	 * has been thrown away. */
	grant = nl_link->advance(nl_handle, now, nl_safe, now + grain);

	nl_refresh_peers();

	/* Keep saying where the lines are until the other console answers.
	 *
	 * One announcement is not enough, and the reason is worth stating: the bus
	 * cannot place a message on a peer that has not published its own position
	 * yet, so it drops it. Both consoles re-anchor the instant a cable is
	 * seated, so whichever of them notices first announces into exactly that
	 * gap and is heard by nobody. It then has no reason to try again -- the
	 * membership it is watching did not change -- and that console's DTR is
	 * never seen, so the game on the other end sits reading DSR low with a
	 * cable plugged in and a peer count of two. */
	if (nl_peers >= 2 && !nl_peer_lines_seen)
		nl_publish_lines();

	nl_drain();
	nl_release();

	if (grant != RETRO_LINK_UNBOUNDED && grant > now && grant - now < (uint64_t)grain)
		return (u32)(grant - now);
	return grain;
}

static void nl_drv_sync(void)
{
	/* No bus call: a register read must not become a rendezvous. This only lets
	 * a byte already taken off the wire land at the cycle it was due rather than
	 * at the next poll, which matters because a game polls STAT far more often
	 * than the port meets the other console. */
	nl_release();
}

static void nl_drv_reanchor(void)
{
	nl_have_raw = 0;
	nl_pending_count = 0;
	nl_lines_published = 0;
	nl_peer_lines = 0;
}

static int nl_drv_connected(void)
{
	return nl_peers >= 2;
}

static const struct sio1_driver nl_driver = {
	nl_drv_lines,
	nl_drv_tx,
	nl_drv_poll,
	nl_drv_sync,
	nl_drv_reanchor,
	nl_drv_connected,
};

/* ── attach and detach ────────────────────────────────────────────────────── */

void sio1NetlinkAttach(const struct retro_link_interface *link, unsigned port)
{
	if (nl_attached || !link)
		return;

	nl_link = link;

	nl_handle = link->attach(port, NETLINK_PROTOCOL, PSXCLK);
	if (!nl_handle) {
		SysPrintf("sio1: the frontend refused a link on port %u\n", port);
		nl_link = NULL;
		return;
	}

	nl_attached = 1;
	nl_have_raw = 0;
	nl_now = 0;
	nl_safe = 0;
	nl_pending_count = 0;
	nl_next_seq = 0;
	nl_last_stamp = 0;
	nl_lines = 0;
	nl_lines_published = 0;
	nl_peer_lines = 0;
	nl_peers = 0;
	nl_self_id = 0;

	sio1SetDriver(&nl_driver);
	SysPrintf("sio1: link cable attached to port %u\n", port);
}

void sio1NetlinkDetach(void)
{
	sio1SetDriver(NULL);

	if (nl_attached) {
		nl_link->detach(nl_handle);
		nl_attached = 0;
		nl_handle = NULL;
	}
	nl_link = NULL;
	nl_pending_count = 0;
	nl_peers = 0;
}
