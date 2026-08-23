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

#ifndef _SIO1_H_
#define _SIO1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "psxcommon.h"

/* The serial port at 1F801050h, and the link cable that plugs into it.
 *
 * This is SIO1, the 8-pin socket on the back of the console, and it has nothing
 * to do with sio.c: that is SIO0, the controller and memory card bus, which
 * shares only a register layout and a name. A PlayStation link cable is a null
 * modem between two of these sockets -- TX to RX, DTR to DSR, RTS to CTS, both
 * ways -- so the two consoles are equals and neither owns the clock.
 *
 * Nothing here knows how the other console is reached. The port is emulated as
 * a UART and a driver carries the bytes; with no driver installed the port
 * behaves exactly as it did before it was emulated at all, which is what keeps
 * every unlinked game running as it used to.
 */

/* The wire behind the socket, installed by whichever frontend can offer one. */
struct sio1_driver {
	/* This console's DTR and RTS, as the guest last left them. The peer reads
	 * them as DSR and CTS, since the cable crosses both pairs. */
	void (*lines)(int dtr, int rts);

	/* One byte leaving the port. `cycles` is how long it occupies the wire, so
	 * a driver that has to say WHEN the byte lands at the other end can work it
	 * out without knowing anything about MODE or BAUD. */
	void (*tx)(unsigned char data, u32 cycles);

	/* Rendezvous with whatever is on the other end and hand over anything that
	 * has arrived, by calling sio1Receive(). `grain` is how far this console
	 * intends to run before asking again and `horizon` how far ahead of itself
	 * it promises not to originate anything. Returns the number of cycles until
	 * it wants to be called again, which may be shorter than `grain`.
	 *
	 * May block: this is the point at which two emulated machines meet. */
	u32 (*poll)(u32 grain, u32 horizon);

	/* Deliver anything already taken off the wire whose moment has come.
	 * Must not block or touch the wire, so that a register read can call it and
	 * see a byte land at the cycle it was due rather than at the next poll. */
	void (*sync)(void);

	/* Forget where this console's clock was. A reset or a state load moves
	 * psxRegs.cycle by an arbitrary amount, and a driver keeping a running
	 * total must not read that as elapsed time. */
	void (*reanchor)(void);

	/* Forget what the peer's lines were last known to be, and ask again.
	 *
	 * The port keeps DSR and CTS as levels, and a reset clears them along with
	 * everything else. The driver holds the same levels, so after a reset the
	 * two disagree: the guest reads DSR low while the driver still believes it
	 * has been told, and a level nobody is going to repeat is a level that
	 * never comes back. */
	void (*forget_peer)(void);

	/* Whether anything is actually cabled to this console. A driver may be
	 * installed for a whole session with nothing on the other end. */
	int (*connected)(void);
};

void sio1SetDriver(const struct sio1_driver *drv);

/* Called by the driver. */
void sio1Receive(unsigned char data);
void sio1SetPeerLines(int dsr, int cts);

void sio1Reset(void);
void sio1Update(void);

void sio1Write8(unsigned char value);
void sio1WriteStat16(unsigned short value);
void sio1WriteMode16(unsigned short value);
void sio1WriteCtrl16(unsigned short value);
void sio1WriteBaud16(unsigned short value);

unsigned char  sio1Read8(void);
u32            sio1ReadStat16(void);
unsigned short sio1ReadMode16(void);
unsigned short sio1ReadCtrl16(void);
unsigned short sio1ReadBaud16(void);

#ifdef __cplusplus
}
#endif
#endif
