/***************************************************************************
 *   Copyright (C) 2026 PCSX-ReARMed contributors                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02111-1307 USA.           *
 ***************************************************************************/

#include "sio1.h"
#include "psxhw.h"
#include "psxevents.h"

/* SIO1_STAT, 1F801054h */
#define SIO1_TX_RDY      0x0001 /* the holding register will take a byte */
#define SIO1_RX_RDY      0x0002 /* the receive FIFO has something in it */
#define SIO1_TX_EMPTY    0x0004 /* nothing left to send, shift register idle */
#define SIO1_PARITY_ERR  0x0008
#define SIO1_RX_OVERRUN  0x0010
#define SIO1_STOP_ERR    0x0020
#define SIO1_RX_INVERTED 0x0040
#define SIO1_DSR         0x0080
#define SIO1_CTS         0x0100
#define SIO1_IRQ         0x0200
#define SIO1_ERRORS      (SIO1_PARITY_ERR | SIO1_RX_OVERRUN | SIO1_STOP_ERR)

/* SIO1_CTRL, 1F80105Ah */
#define SIO1_TX_EN       0x0001
#define SIO1_DTR         0x0002
#define SIO1_RX_EN       0x0004
#define SIO1_TX_BREAK    0x0008
#define SIO1_ACK         0x0010
#define SIO1_RTS         0x0020
#define SIO1_RESET       0x0040
#define SIO1_RX_IRQ_MODE 0x0300
#define SIO1_TX_IRQ_EN   0x0400
#define SIO1_RX_IRQ_EN   0x0800
#define SIO1_DSR_IRQ_EN  0x1000

/* I_STAT bit 8. NOT bit 7, which is SIO0's -- the controller and memory card
 * bus has its own interrupt and sio.c already raises it. */
#define SIO1_INTERRUPT   0x100

#define SIO1_FIFO_SIZE   8

/* How often the port meets whatever is on the other end, in CPU cycles.
 *
 * The ceiling is one byte's time on the wire, because a byte announced now
 * lands one byte-time from now and the peer has to still be short of that
 * moment when it hears about it. Half of that leaves room for a peer that is
 * itself a grain behind. The floor is cost: every grain is a rendezvous between
 * two emulation threads, so a game running at 115200 baud must not spend more
 * of its time synchronising than emulating.
 *
 * The idle figure is separate and much coarser. Nothing is being carried, so
 * the only thing the rendezvous can discover is that a cable has just been
 * plugged in, and two milliseconds of emulated time is a fast enough answer to
 * a question a hand asked. */
#define SIO1_GRAIN_MIN   256
#define SIO1_GRAIN_MAX   (PSXCLK / 1000)
#define SIO1_GRAIN_IDLE  (PSXCLK / 500)

static const struct sio1_driver *sio1_drv;

static struct {
	u16 mode;
	u16 ctrl;
	u16 baud;

	u8  rx[SIO1_FIFO_SIZE];
	u8  rx_head;
	u8  rx_count;

	u8  tx_hold;
	u8  tx_hold_full;
	u8  tx_active;
	u32 tx_end;

	u8  errors;   /* STAT bits 3..5, cleared by the acknowledge bit */
	u8  irq;
	u8  dsr;
	u8  cts;
} sio1;

static void sio1_schedule(u32 cycles);

static void sio1_irq(void) {
	if (!sio1.irq) {
		sio1.irq = 1;
		psxHu32ref(0x1070) |= SWAPu32(SIO1_INTERRUPT);
	}
}

/* Whether the port is emulated at all. Without a driver there is no wire, and
 * the port answers exactly as it did before any of this existed. */
static int sio1_enabled(void) {
	return sio1_drv != NULL;
}

/* Whether anything is on the other end. Only the rendezvous interval is decided
 * by this: an enabled port with nothing cabled to it is a real serial port with
 * a dead cable, which is a state the guest is entitled to observe. */
static int sio1_connected(void) {
	return sio1_drv && sio1_drv->connected && sio1_drv->connected();
}

/* One bit's time on the wire, in CPU cycles.
 *
 * The baud generator reloads with BAUD times the factor MODE selects and runs
 * off the same 33.8688 MHz clock psxRegs.cycle counts, so the product IS the
 * cycle count. A game asking for 9600 baud writes 55 with the 64x factor:
 * 55 * 64 = 3520 cycles, and 33868800 / 3520 is 9622 baud. */
static u32 sio1_cycles_per_bit(void) {
	static const u8 factor[4] = { 1, 1, 16, 64 };
	u32 cycles = (u32)sio1.baud * factor[sio1.mode & 3];

	/* A game that has not written BAUD yet, or wrote zero, would otherwise ask
	 * for an instantaneous byte and a zero-cycle rendezvous. */
	if (cycles < 8)
		cycles = 8;
	return cycles;
}

static u32 sio1_cycles_per_byte(void) {
	u32 bits = 1 + 5 + ((sio1.mode >> 2) & 3);   /* start bit, then 5 to 8 data */

	if (sio1.mode & 0x10)
		bits++;                                   /* parity */
	bits += ((sio1.mode >> 6) & 3) == 3 ? 2 : 1;  /* stop bits; 1.5 counts as 1 */

	return sio1_cycles_per_bit() * bits;
}

static u32 sio1_grain(void) {
	u32 grain;

	if (!sio1_connected())
		return SIO1_GRAIN_IDLE;

	grain = sio1_cycles_per_byte() / 2;
	if (grain < SIO1_GRAIN_MIN)
		grain = SIO1_GRAIN_MIN;
	if (grain > SIO1_GRAIN_MAX)
		grain = SIO1_GRAIN_MAX;
	return grain;
}

static u32 sio1_rx_threshold(void) {
	return 1u << ((sio1.ctrl & SIO1_RX_IRQ_MODE) >> 8);
}

/* Hand the holding register to the wire, if there is one to hand over and the
 * guest has enabled the transmitter. */
static void sio1_tx_kick(void) {
	u32 cycles;

	if (sio1.tx_active || !sio1.tx_hold_full || !(sio1.ctrl & SIO1_TX_EN))
		return;
	if (!sio1_enabled()) {
		/* No wire and no event to time a transfer against, so the byte is
		 * simply dropped rather than starting a transfer nothing will finish. */
		sio1.tx_hold_full = 0;
		return;
	}

	cycles = sio1_cycles_per_byte();
	sio1.tx_hold_full = 0;
	sio1.tx_active = 1;
	sio1.tx_end = psxRegs.cycle + cycles;

	if (sio1_drv->tx)
		sio1_drv->tx(sio1.tx_hold, cycles);

	sio1_schedule(cycles);
}

static void sio1_tx_done(void) {
	sio1.tx_active = 0;

	/* TX Ready Flag 2 has just come up. Raising the interrupt on that edge and
	 * not on flag 1 is deliberate: flag 1 says the holding register is free,
	 * which is true almost all the time, so a game that enabled the transmit
	 * interrupt would be handed one continuously. */
	if (sio1.ctrl & SIO1_TX_IRQ_EN)
		sio1_irq();

	sio1_tx_kick();
}

static void sio1_schedule(u32 cycles) {
	if (sio1.tx_active) {
		u32 remaining = sio1.tx_end - psxRegs.cycle;
		if ((s32)remaining < (s32)cycles)
			cycles = remaining;
	}
	if ((s32)cycles < 8)
		cycles = 8;
	set_event(PSXINT_SIO1, cycles);
}

void sio1SetDriver(const struct sio1_driver *drv) {
	sio1_drv = drv;
	if (drv)
		sio1_schedule(SIO1_GRAIN_IDLE);
	else
		psxRegs.interrupt &= ~(1 << PSXINT_SIO1);
}

void sio1Receive(unsigned char data) {
	if (sio1.rx_count >= SIO1_FIFO_SIZE) {
		sio1.errors |= SIO1_RX_OVERRUN;
		return;
	}

	sio1.rx[(sio1.rx_head + sio1.rx_count) & (SIO1_FIFO_SIZE - 1)] = data;
	sio1.rx_count++;

	if ((sio1.ctrl & SIO1_RX_IRQ_EN) && sio1.rx_count >= sio1_rx_threshold())
		sio1_irq();
}

void sio1SetPeerLines(int dsr, int cts) {
	int rising = dsr && !sio1.dsr;

	sio1.dsr = dsr ? 1 : 0;
	sio1.cts = cts ? 1 : 0;

	/* DSR coming up is how a game finds out a console has appeared on the other
	 * end of the cable, which is the one line change worth an interrupt. */
	if (rising && (sio1.ctrl & SIO1_DSR_IRQ_EN))
		sio1_irq();
}

/* Put the port back the way it powers up. Split from sio1Reset because the
 * guest can ask for this through CTRL bit 6 without the console's clock having
 * moved, and re-anchoring the driver's timeline there would throw away a
 * position that is still correct. */
static void sio1_port_reset(void) {
	memset(&sio1, 0, sizeof(sio1));
	psxRegs.interrupt &= ~(1 << PSXINT_SIO1);

	if (sio1_drv) {
		if (sio1_drv->lines)
			sio1_drv->lines(0, 0);
		sio1_schedule(SIO1_GRAIN_IDLE);
	}
}

void sio1Reset(void) {
	/* A hard reset or a state load moves psxRegs.cycle by an arbitrary amount,
	 * including backwards, and a driver accumulating elapsed cycles must not
	 * read that as time having passed. */
	if (sio1_drv && sio1_drv->reanchor)
		sio1_drv->reanchor();
	sio1_port_reset();
}

void sio1Update(void) {
	u32 grain = sio1_grain();
	u32 next = grain;

	if (sio1.tx_active && (s32)(psxRegs.cycle - sio1.tx_end) >= 0)
		sio1_tx_done();

	if (sio1_drv && sio1_drv->poll) {
		/* The horizon is the grain. The promise it stands for is that nothing
		 * this console originates will land before it, and a byte handed to the
		 * wire lands a whole byte-time later, so a grain of half that is
		 * comfortably inside what was promised. */
		next = sio1_drv->poll(grain, grain);
		if (next == 0 || next > grain)
			next = grain;
	}

	sio1_schedule(next);
}

void sio1Write8(unsigned char value) {
	sio1.tx_hold = value;
	sio1.tx_hold_full = 1;
	sio1_tx_kick();
}

void sio1WriteStat16(unsigned short value) {
	/* Read-only on hardware, as SIO0's is. */
}

void sio1WriteMode16(unsigned short value) {
	sio1.mode = value;
}

void sio1WriteCtrl16(unsigned short value) {
	u16 was = sio1.ctrl;

	if (value & SIO1_RESET) {
		/* Nothing but the reset is honoured on a write that carries it: the
		 * hardware clears the registers rather than applying the rest of the
		 * value to them. */
		sio1_port_reset();
		return;
	}

	if (value & SIO1_ACK) {
		sio1.errors = 0;
		sio1.irq = 0;
	}

	/* Neither the acknowledge nor the reset bit is a register bit; both are
	 * momentary and read back as zero. */
	sio1.ctrl = value & ~(SIO1_ACK | SIO1_RESET);

	if ((sio1.ctrl ^ was) & (SIO1_DTR | SIO1_RTS)) {
		if (sio1_drv && sio1_drv->lines)
			sio1_drv->lines(!!(sio1.ctrl & SIO1_DTR), !!(sio1.ctrl & SIO1_RTS));
	}

	/* A game commonly fills the holding register before enabling the
	 * transmitter, so this write is where the byte actually leaves. */
	sio1_tx_kick();
}

void sio1WriteBaud16(unsigned short value) {
	sio1.baud = value;
}

unsigned char sio1Read8(void) {
	unsigned char ret;

	if (sio1_drv && sio1_drv->sync)
		sio1_drv->sync();

	if (!sio1.rx_count)
		return 0xff;

	ret = sio1.rx[sio1.rx_head];
	sio1.rx_head = (sio1.rx_head + 1) & (SIO1_FIFO_SIZE - 1);
	sio1.rx_count--;
	return ret;
}

u32 sio1ReadStat16(void) {
	u32 stat;

	if (!sio1_enabled()) {
		/* What this register has always answered, and the reason it was a
		 * constant rather than a status: with no SIO1 emulation behind it a
		 * computed value was whatever happened to be in the I/O page, and
		 * Armored Core and F1 misdetected a link cable from it.
		 *
		 * It is not what an idle port reads -- it has neither TX ready flag
		 * set, so a game waiting on one before writing a byte never gets that
		 * far -- but it is what every session without a link driver has always
		 * seen, and installing one must be the only thing that changes it. */
		return 0xa0;
	}

	if (sio1_drv->sync)
		sio1_drv->sync();

	stat = 0;
	if (!sio1.tx_hold_full)
		stat |= SIO1_TX_RDY;
	if (!sio1.tx_hold_full && !sio1.tx_active)
		stat |= SIO1_TX_EMPTY;
	if (sio1.rx_count)
		stat |= SIO1_RX_RDY;
	if (sio1.dsr)
		stat |= SIO1_DSR;
	if (sio1.cts)
		stat |= SIO1_CTS;
	if (sio1.irq)
		stat |= SIO1_IRQ;
	stat |= sio1.errors;

	/* Bits 11-31 are the baud generator's countdown. Left at zero: reading it
	 * costs a divide on a register games poll in tight loops, and nothing is
	 * known to time anything against it. */
	return stat;
}

unsigned short sio1ReadMode16(void) {
	return sio1.mode;
}

unsigned short sio1ReadCtrl16(void) {
	return sio1.ctrl;
}

unsigned short sio1ReadBaud16(void) {
	return sio1.baud;
}
