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
 * takes to acknowledge it.
 *
 * Savestates carry each docked device whole (Pokket's own state: CPU, RAM, flash, timers,
 * the COM link mid-byte) in a chunk of their own (PocketStationFreeze), so a load is exact
 * and a PlayStation with one docked can roll back. The state size is fixed for the run
 * whether or not a slot holds one, because a frontend measures it once.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#include "psxcommon.h"
#include "sio.h"
#include "system.h"
#include "pocketstation.h"
#include "psemu/psemu.h"
#include "misc.h"

/* The reference clock runs at 1,056,000 Hz: one 60 Hz frame is 17,600 of its cycles. */
#define PKST_FRAME_REF_CYCLES 17600u
/* How long the kernel may take to enable communication after a dock, in frames. */
#define PKST_DOCK_FRAMES 60
/* How long the device runs on its own before it is docked, in PS1 frames of its time (16.7 s):
   power-on, HELLO and the date screen's timeout to the clock. Docked straight from a power-on
   its BIOS draws nothing, and a real one goes in already running. Measured once, below. */
#define PKST_BOOT_FRAMES 1000
/* After /SEL is released the kernel leaves its wait and readies the next command. */
#define PKST_RELEASE_REF_CYCLES 512u

static psemu_t *dev[2];
static int wanted[2];
static int enabled[2];
static int selected[2];
static unsigned char shadow[2][MCD_SIZE];
/* The LCD as last handed to the frontend, and whether it has been since a dock. */
static unsigned char lcd_sent[2][PSEMU_LCD_STRIDE * PSEMU_LCD_HEIGHT];
static int lcd_valid[2];

/* frontend/libretro.c: the frontend's device display interface, or false. */
extern bool plat_card_device_screen(int slot, const uint32_t *pixels, unsigned width, unsigned height);
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
	double boot_ms = 0;
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
	lcd_valid[slot] = 0;
	{
		int i;
		clock_t t0 = clock();
		for (i = 0; i < PKST_BOOT_FRAMES; i++)
			psemu_run(dev[slot], PKST_FRAME_REF_CYCLES);
		boot_ms = (double)(clock() - t0) * 1000.0 / CLOCKS_PER_SEC;
	}
	psemu_com_set_docked(dev[slot], 1);
	SysWarn("[pocketstation] slot %d: a PocketStation, booted %d frames on its own, docked in %.0f ms (BIOS revision %s)\n", slot + 1, PKST_BOOT_FRAMES, boot_ms,
		psemu_settings_offsets_known(dev[slot]) ? "J110 as Pokket traced it" : "other");
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

/* The LCD to the frontend when it changed: 1 bpp, bit 0 the leftmost pixel, set is black,
   as the device draws it (its own 180 degree turn in the dock included). */
static void send_screen(int slot)
{
	const uint8_t *fb = psemu_get_framebuffer(dev[slot]);
	uint32_t px[PSEMU_LCD_WIDTH * PSEMU_LCD_HEIGHT];
	int row, col;
	if (lcd_valid[slot] && memcmp(fb, lcd_sent[slot], sizeof(lcd_sent[slot])) == 0)
		return;
	for (row = 0; row < PSEMU_LCD_HEIGHT; row++)
		for (col = 0; col < PSEMU_LCD_WIDTH; col++) {
			int on = (fb[row * PSEMU_LCD_STRIDE + col / 8] >> (col % 8)) & 1;
			px[row * PSEMU_LCD_WIDTH + col] = on ? 0x000000u : 0xffffffu;
		}
	memcpy(lcd_sent[slot], fb, sizeof(lcd_sent[slot]));
	if (!lcd_valid[slot] && !plat_card_device_screen(slot, px, PSEMU_LCD_WIDTH, PSEMU_LCD_HEIGHT))
		SysWarn("[pocketstation] slot %d: no frontend takes the device's screen\n", slot + 1);
	else if (lcd_valid[slot])
		plat_card_device_screen(slot, px, PSEMU_LCD_WIDTH, PSEMU_LCD_HEIGHT);
	lcd_valid[slot] = 1;
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
		send_screen(slot);
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

/* --- Savestates ------------------------------------------------------------ */

#define PKST_FREEZE_MAGIC 0x54534b50u /* "PKST" */
#define PKST_FREEZE_VERSION 1u

static size_t device_state_size(void)
{
	static size_t size;
	if (!size) {
		psemu_t *probe = psemu_create();
		if (probe) {
			size = psemu_state_size(probe);
			psemu_destroy(probe);
		}
	}
	return size;
}

size_t PocketStationFreezeSize(void)
{
	/* magic, version, then per slot four flag bytes, a length and the state. */
	return 8 + 2 * (4 + 4 + device_state_size());
}

int PocketStationFreeze(void *f, int Mode)
{
	u32 magic = PKST_FREEZE_MAGIC, version = PKST_FREEZE_VERSION;
	int slot;
	if (Mode) {
		SaveFuncs.write(f, &magic, sizeof(magic));
		SaveFuncs.write(f, &version, sizeof(version));
		for (slot = 0; slot < 2; slot++) {
			unsigned char flags[4] = { (unsigned char)wanted[slot], (unsigned char)enabled[slot],
				(unsigned char)selected[slot], 0 };
			u32 size = dev[slot] ? (u32)psemu_state_size(dev[slot]) : 0;
			void *blob = size ? malloc(size) : NULL;
			if (size && (!blob || psemu_save_state(dev[slot], blob, size) != PSEMU_OK)) {
				SysWarn("[pocketstation] slot %d: the device did not save; the state holds none\n", slot + 1);
				size = 0;
			}
			SaveFuncs.write(f, flags, sizeof(flags));
			SaveFuncs.write(f, &size, sizeof(size));
			if (size)
				SaveFuncs.write(f, blob, size);
			free(blob);
		}
		return 1;
	}

	SaveFuncs.read(f, &magic, sizeof(magic));
	SaveFuncs.read(f, &version, sizeof(version));
	if (magic != PKST_FREEZE_MAGIC || version != PKST_FREEZE_VERSION)
		return 0;
	for (slot = 0; slot < 2; slot++) {
		unsigned char flags[4];
		u32 size = 0;
		void *blob;
		SaveFuncs.read(f, flags, sizeof(flags));
		SaveFuncs.read(f, &size, sizeof(size));
		if (!size) {
			drop(slot);
			wanted[slot] = flags[0];
			continue;
		}
		if (size > 4 * 1024 * 1024 || !(blob = malloc(size))) {
			SysWarn("[pocketstation] slot %d: a device state of %u bytes, skipped\n", slot + 1, (unsigned)size);
			SaveFuncs.seek(f, size, SEEK_CUR);
			continue;
		}
		SaveFuncs.read(f, blob, size);
		/* A device to load into: the BIOS is ROM and not in the state. */
		if ((dev[slot] || attach(slot)) && psemu_load_state(dev[slot], blob, size) == PSEMU_OK) {
			wanted[slot] = flags[0];
			enabled[slot] = flags[1];
			selected[slot] = flags[2];
			/* The card is the device's flash as it was saved. */
			memcpy(shadow[slot], psemu_flash_data(dev[slot]), MCD_SIZE);
			memcpy(card_data(slot), shadow[slot], MCD_SIZE);
			lcd_valid[slot] = 0;
		}
		else
			SysWarn("[pocketstation] slot %d: the device state did not load\n", slot + 1);
		free(blob);
	}
	return 1;
}

void PocketStationShutdown(void)
{
	drop(0);
	drop(1);
	wanted[0] = wanted[1] = 0;
}
