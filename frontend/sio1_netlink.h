/*
 * (C) 2026 PCSX-ReARMed contributors
 *
 * This work is licensed under the terms of the GNU GPL version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef __SIO1_NETLINK_H__
#define __SIO1_NETLINK_H__

#include "libretro.h"

/* A PlayStation link cable carried by the frontend's link bus.
 *
 * Two consoles emulated in one process cannot reach each other on their own: a
 * frontend that runs several cores at once generally loads each from its own
 * copy of the shared library, so each instance has its own copy of every global
 * and no coordinator inside the core can be shared between them. The frontend
 * is the only thing they have in common, so it hosts the bus and the core joins
 * it through RETRO_ENVIRONMENT_GET_LINK_INTERFACE.
 *
 * What crosses is bytes and the two handshake lines, nothing else. A link cable
 * is a null modem -- TX to RX, DTR to DSR, RTS to CTS, both ways -- so neither
 * console owns the clock and there is no master to elect.
 *
 * Attaching is safe whether or not anything is ever cabled to this console: an
 * unjoined port bounds nobody and carries nothing.
 */
void sio1NetlinkAttach(const struct retro_link_interface *link, unsigned port);
void sio1NetlinkDetach(void);

#endif /* __SIO1_NETLINK_H__ */
