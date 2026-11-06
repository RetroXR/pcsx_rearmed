/* A Sony PocketStation in a memory card slot (RetroXR). See pocketstation.c. */
#ifndef __POCKETSTATION_H__
#define __POCKETSTATION_H__

/* Slots are 0 and 1. A wanted slot becomes a docked device at the next frame (with a BIOS and a
   real PS1 BIOS), and is active once its kernel has enabled communication. */
void PocketStationSetWanted(int slot, int on);
int PocketStationWanted(int slot);
int PocketStationActive(int slot);
/* Once per emulated frame, after the console's. */
void PocketStationFrame(void);
/* One byte of a card command, from 81h on: the device's reply, and whether it acknowledged. */
unsigned char PocketStationByte(int slot, unsigned char in, int *acked);
/* The console dropped DTR or reset the port: the command is over. */
void PocketStationRelease(int slot);
void PocketStationShutdown(void);

#endif
