/*
 * A Sony PocketStation in a memory card slot (RetroXR).
 *
 * The device is Pokket's emulation core (deps/pokket, MIT), run inside the console: its own
 * kernel answers every card byte, the plain card commands (52h/53h/57h) and the PocketStation
 * ones (50h, 58h..5Fh), so a game that asks "is a PocketStation there?" (58h) gets the device's
 * own answer and offers its minigame download. Nothing here implements a command: 5Bh/5Ch run
 * functions out of the downloaded app, which only the device's CPU can answer.
 *
 * The device's flash IS the card. It is loaded from Mcd{N}Data when the slot is switched to
 * a PocketStation, and whatever the device writes goes back into Mcd{N}Data (and SaveMcd), so
 * the frontend's SAVE_RAM sees the card the device holds. A card the frontend swaps in under it
 * (Mcd{N}Data changed while the device's flash did not) is loaded into the device again.
 *
 * Timing is by the console's frames and bytes, never the host clock: the device runs one PS1
 * frame's worth of its own time each frame, and inside each byte for as long as its kernel
 * takes to acknowledge it. Not yet in savestates (logged).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "psxcommon.h"
#include "sio.h"
#include "system.h"
#include "pocketstation.h"
#include "psemu/psemu.h"

/* The reference clock runs at 1,056,000 Hz: one 60 Hz frame is 17,600 of its cycles. */
#define PKST_FRAME_REF_CYCLES 17600u
/* How long the kernel may take to enable communication after a dock, in frames. */
#define PKST_DOCK_FRAMES 60
/* After /SEL is released the kernel leaves its wait and readies the next command. */
#define PKST_RELEASE_REF_CYCLES 512u

static psemu_t *dev[2];
static int wanted[2];
static int enabled[2];
static int selected[2];
static unsigned char shadow[2][MCD_SIZE];
static int warned_state;
/* PKST_TRACE in the environment logs the first bytes of the exchange. */
static int traced;

static char *card_data(int slot) { return slot == 0 ? Mcd1Data : Mcd2Data; }
static char *card_path(int slot) { return slot == 0 ? Config.Mcd1 : Config.Mcd2; }

static int read_bios(unsigned char *bios)
{
	/* Pokket's own core reads <system>/pocketstation.bin; a frontend that gives every core a
	   system folder of its own (RetroXR) keeps it beside the pokket core's. */
	const char *names[] = { "pocketstation.bin", "../pokket/pocketstation.bin" };
	char path[MAXPATHLEN + 32];
	size_t i;
	for (i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
		FILE *f;
		size_t n;
		snprintf(path, sizeof(path), "%s/%s", Config.BiosDir, names[i]);
		f = fopen(path, "rb");
		if (!f)
			continue;
		n = fread(bios, 1, PSEMU_BIOS_SIZE, f);
		fclose(f);
		if (n == PSEMU_BIOS_SIZE) {
			SysWarn("[pocketstation] BIOS %s\n", path);
			return 1;
		}
		SysWarn("[pocketstation] %s is %u bytes, not %u\n", path, (unsigned)n, (unsigned)PSEMU_BIOS_SIZE);
	}
	SysWarn("[pocketstation] no pocketstation.bin in %s or ../pokket: the slot stays a plain card\n",
		Config.BiosDir);
	return 0;
}

static void drop(int slot)
{
	if (dev[slot])
		psemu_destroy(dev[slot]);
	dev[slot] = NULL;
	enabled[slot] = 0;
	selected[slot] = 0;
}

/* Load the card into the device, power it on and dock it; communication follows a few frames
   later, once its kernel has seen the dock (PocketStationFrame). */
static int attach(int slot)
{
	unsigned char bios[PSEMU_BIOS_SIZE];
	drop(slot);
	if (Config.HLE) {
		SysWarn("[pocketstation] slot %d: the HLE BIOS reads cards directly, not over the bus; "
			"a PocketStation needs a real PS1 BIOS, so the slot stays a plain card\n", slot + 1);
		return 0;
	}
	if (!read_bios(bios))
		return 0;
	dev[slot] = psemu_create();
	if (!dev[slot])
		return 0;
	if (psemu_load_bios(dev[slot], bios, PSEMU_BIOS_SIZE) != PSEMU_OK
			|| psemu_load_content(dev[slot], (const uint8_t *)card_data(slot), MCD_SIZE) != PSEMU_OK) {
		SysWarn("[pocketstation] slot %d: the device refused the BIOS or the card\n", slot + 1);
		drop(slot);
		return 0;
	}
	psemu_reset(dev[slot]);
	memcpy(shadow[slot], card_data(slot), MCD_SIZE);
	psemu_com_set_docked(dev[slot], 1);
	SysWarn("[pocketstation] slot %d: a PocketStation, docked (BIOS revision %s)\n", slot + 1,
		psemu_settings_offsets_known(dev[slot]) ? "J110 as Pokket traced it" : "other");
	if (!warned_state) {
		warned_state = 1;
		SysWarn("[pocketstation] the device is not in savestates yet\n");
	}
	return 1;
}

/* The device wrote its flash: put it on the card. The frontend swapped the card: put it in the
   device. Both at once keeps the frontend's (it is the newer act) and says so. */
static void sync_flash(int slot)
{
	const unsigned char *flash = psemu_flash_data(dev[slot]);
	char *card = card_data(slot);
	int device_moved = memcmp(flash, shadow[slot], MCD_SIZE) != 0;
	int card_moved = memcmp(card, shadow[slot], MCD_SIZE) != 0;
	if (card_moved) {
		if (device_moved)
			SysWarn("[pocketstation] slot %d: the card and the device both changed; the card wins\n", slot + 1);
		SysWarn("[pocketstation] slot %d: the card changed under the device; reloading it\n", slot + 1);
		attach(slot);
		return;
	}
	if (device_moved) {
		int frame, written = 0;
		for (frame = 0; frame < MCD_SIZE / 128; frame++) {
			int off = frame * 128;
			if (memcmp(flash + off, shadow[slot] + off, 128) != 0) {
				memcpy(card + off, flash + off, 128);
				SaveMcd(card_path(slot), card, off, 128);
				written++;
			}
		}
		memcpy(shadow[slot], flash, MCD_SIZE);
		SysWarn("[pocketstation] slot %d: the device wrote %d frame(s) of its flash\n", slot + 1, written);
	}
}

void PocketStationSetWanted(int slot, int on)
{
	if (slot < 0 || slot > 1 || wanted[slot] == !!on)
		return;
	wanted[slot] = !!on;
	SysWarn("[pocketstation] slot %d: %s\n", slot + 1, on ? "a PocketStation" : "a plain card");
	if (!on)
		drop(slot);
}

int PocketStationActive(int slot)
{
	return slot >= 0 && slot < 2 && dev[slot] != NULL && enabled[slot];
}

int PocketStationWanted(int slot)
{
	return slot >= 0 && slot < 2 && wanted[slot];
}

void PocketStationFrame(void)
{
	int slot;
	for (slot = 0; slot < 2; slot++) {
		if (!wanted[slot])
			continue;
		if (!dev[slot]) {
			if (!attach(slot)) {
				wanted[slot] = 0;
				continue;
			}
		}
		psemu_run(dev[slot], PKST_FRAME_REF_CYCLES);
		if (!enabled[slot] && psemu_com_is_enabled(dev[slot])) {
			enabled[slot] = 1;
			SysWarn("[pocketstation] slot %d: communication enabled\n", slot + 1);
		}
		sync_flash(slot);
	}
}

unsigned char PocketStationByte(int slot, unsigned char in, int *acked)
{
	uint8_t out = 0xff;
	if (!PocketStationActive(slot)) {
		*acked = 0;
		return 0xff;
	}
	selected[slot] = 1;
	*acked = psemu_com_transfer(dev[slot], in, &out, PSEMU_COM_DEFAULT_TIMEOUT_CYCLES) != 0;
	if (getenv("PKST_TRACE") && traced < 600) {
		traced++;
		SysWarn("[pocketstation] byte %d: in %02x out %02x %s\n", traced, in, out, *acked ? "ack" : "no ack");
	}
	return out;
}

void PocketStationRelease(int slot)
{
	if (slot < 0 || slot > 1 || !dev[slot] || !selected[slot])
		return;
	selected[slot] = 0;
	psemu_com_set_selected(dev[slot], 0);
	psemu_run(dev[slot], PKST_RELEASE_REF_CYCLES);
	sync_flash(slot);
}

void PocketStationShutdown(void)
{
	drop(0);
	drop(1);
	wanted[0] = wanted[1] = 0;
}
