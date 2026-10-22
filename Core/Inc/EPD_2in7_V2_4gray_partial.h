#ifndef __EPD_2IN7_V2_4GRAY_PARTIAL_H_
#define __EPD_2IN7_V2_4GRAY_PARTIAL_H_

#include "EPD_2in7_V2.h"

/*
 * Partial refresh in 4-gray mode (experimental, see the .c file).
 *
 * Image : full frame in the SAME format as for EPD_2IN7_V2_4GrayDisplay()
 *         (2 bits/pixel, 4 px per byte, MSB first, 44 bytes per row,
 *         11616 bytes; 11=white 10=light 01=dark 00=black).
 *         It must hold the TARGET grays of every pixel being updated.
 *
 * Precondition: the panel was already shown with EPD_2IN7_V2_Init_4GRAY() +
 * EPD_2IN7_V2_4GrayDisplay(), and every pixel that is NOT updated matches
 * what is on the panel. Do NOT call EPD_2IN7_V2_Init() / Display_Partial()
 * in between without a fresh full 4-gray refresh (they reset the controller
 * and load the OTP waveform).
 */

/* Mask: 1 bit/pixel, MSB first, 22 bytes per row. 1 = update this pixel,
 * 0 = zero drive (the gray background stays untouched). */
#define EPD_4GRAY_MASK_BYTES   ((EPD_2IN7_V2_WIDTH / 8) * EPD_2IN7_V2_HEIGHT)
#define EPD_4GRAY_MASK_SET(m, x, y) \
    ((m)[(UDOUBLE)(y) * (EPD_2IN7_V2_WIDTH / 8) + ((x) >> 3)] |= (UBYTE)(0x80 >> ((x) & 7)))

/* Pixel-exact update: only masked pixels are redrawn. Two waveform passes,
 * 53 frames (~1.1 s) with the default timings plus fixed costs.
 * Return: 0 = ok, -1 = BUSY timeout (10 s). */
int EPD_2IN7_V2_4Gray_PartialMask(const UBYTE *Image, const UBYTE *Mask);

/* Full 4-gray refresh of the whole panel (flashes) with an adjustable shake group
 * (EPD_FULL_SHAKE_RP / EPD_FULL_SHAKE_TP in the .c file). No hardware reset: needs a
 * prior EPD_2IN7_V2_Init_4GRAY(). Image = stock Waveshare 2 bpp buffer, 11616 bytes.
 * Return: 0 = ok, -1 = BUSY timeout. */
int EPD_2IN7_V2_4Gray_Full(const UBYTE *Image);

/* With EPD_4GRAY_KEEP_POWER (default 1) the panel's analog part stays powered after an
 * update; call this when the display goes idle (and before sleeping / cutting power).
 * Return: 0 = ok, -1 = BUSY timeout. */
int EPD_2IN7_V2_4Gray_PowerOff(void);

#ifdef EPD_4GRAY_PROFILE
/* Phase durations of the last update, ms. Full refresh fills setup/spi/wait1 only. */
typedef struct {
    uint16_t setup1;   /* registers + LUT of pass 1 */
    uint16_t spi1;     /* RAM planes of pass 1 (incl. pixel conversion) */
    uint16_t wait1;    /* 0x20 -> BUSY low: pass 1 waveform + power sequencing */
    uint16_t spi2;     /* LUT + RAM planes of pass 2 */
    uint16_t wait2;    /* pass 2 waveform + power sequencing */
} EPD_4Gray_Profile;
extern EPD_4Gray_Profile epd_4gray_prof;
#endif

/* Convenience: rectangle, coordinates INCLUSIVE (builds a mask internally). */
int EPD_2IN7_V2_4Gray_Partial(const UBYTE *Image,
                              UWORD Xstart, UWORD Ystart,
                              UWORD Xend,   UWORD Yend);

#endif
