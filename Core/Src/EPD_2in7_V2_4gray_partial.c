/*****************************************************************************
* | File     : EPD_2in7_V2_4gray_partial.c
* | Function : EXPERIMENTAL partial refresh for the 4-gray mode, 2.7inch V2
*
* Why a special routine is needed
*   The stock LUT_DATA_4Gray is an ABSOLUTE waveform: every pixel is shaken
*   (VSH 20 / VSL 20 frames, x4) up to the VSL extreme and then moved towards
*   VSH by 0 / 3 / 8 / 25 frames (LUT rows 0..3). All 4 LUT rows are used for
*   the 4 gray levels, so there is no "do not touch this pixel" code, and the
*   0x20 trigger always updates the whole panel.
*
* What this file does (two partial-style passes, custom LUT in register 0x32)
*   pass 1 ERASE: pixels of the dirty mask get row 3 = long VSL pulse
*                 (-> VSL extreme); everything else gets row 0 = "keep"
*                 (zero drive, row 0 is empty in both LUTs below).
*   pass 2 DRAW : row 1/2/3 = VSH for 3/8/25 frames (exactly the final pulses
*                 of the stock waveform); pixels of the target gray that maps
*                 to row 0, and everything outside the rectangle, are "keep".
*   The RAM bits written for a gray pixel are exactly the bits that
*   EPD_2IN7_V2_4GrayDisplay() writes, so which gray lands on which LUT row is
*   the same as in the stock waveform.
*
* NOT TESTED ON HARDWARE. Tunables are marked [TUNE].
******************************************************************************/
#include "EPD_2in7_V2_4gray_partial.h"
#include "Debug.h"
#include <string.h>

/* Declared (non-static) in EPD_2in7_V2.c */
extern UBYTE LUT_DATA_4Gray[159];

/* [TUNE] RAM fill for LUT row 0 = "keep" (both planes).
 * 0x00 : row 0 <-> pixel code (0x24=0, 0x26=0)  (SSD1680 convention, expected)
 * 0xFF : use if the result looks inverted / the rectangle comes out wrong     */
#define EPD_KEEP_FILL   0x00

/* [TUNE] frames. Row 3 (black-most) / row 2 / row 1 as in the stock LUT. */
#define T_ROW1   3
#define T_ROW2   8
#define T_ROW3   25
#define T_ERASE  40   /* VSL pulse that must saturate from any gray; >= ~1.5*T_ROW3 */
#define T_ERASE_PRE 0 /* optional VSH pre-pulse before the erase, frames */

#define BYTES_PER_ROW   (EPD_2IN7_V2_WIDTH / 8)    /* 22 plane bytes / 44 image bytes */
#define IMG_STRIDE      (EPD_2IN7_V2_WIDTH / 4)

/* --------------------------- low level --------------------------------- */
static void g_cmd(UBYTE reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void g_data(UBYTE d)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(d);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void g_burst_begin(void)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
}

static void g_burst_end(void)
{
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void g_busy(void)
{
    do {
        if (DEV_Digital_Read(EPD_BUSY_PIN) == 0) break;
    } while (1);
    DEV_Delay_ms(20);
}

/* same as EPD_2IN7_V2_TurnOnDisplay_4GRAY: LUT from register, mode 1 */
static void g_turn_on(void)
{
    g_cmd(0x22);
    g_data(0xC7);
    g_cmd(0x20);
    g_busy();
}

/* ------------------------------ LUT ------------------------------------ */
/* Layout: [0..59] VS 5 rows x 12 groups (byte = phases A,B,C,D, 2 bit each,
 * 01=VSH1 10=VSL), [60..143] 12 groups x {TPA,TPB,SRAB,TPC,TPD,SRCD,RP},
 * [144..152] FR + XON (copied from the stock LUT). */
static void lut_base(UBYTE *l)
{
    memset(l, 0, 153);
    memcpy(&l[144], &LUT_DATA_4Gray[144], 9);
}

static void lut_group(UBYTE *l, int g, UBYTE tpa, UBYTE tpb, UBYTE tpc, UBYTE tpd, UBYTE rp)
{
    UBYTE *p = &l[60 + g * 7];
    p[0] = tpa; p[1] = tpb; p[2] = 0; p[3] = tpc; p[4] = tpd; p[5] = 0; p[6] = rp;
}

static void lut_build_erase(UBYTE *l)
{
    lut_base(l);
    l[3 * 12 + 0] = 0x60;                 /* row 3: A=VSH (pre), B=VSL */
    lut_group(l, 0, T_ERASE_PRE, T_ERASE, 0, 0, 0);
}

static void lut_build_draw(UBYTE *l)
{
    lut_base(l);
    l[1 * 12 + 0] = 0x40;                 /* row 1: A=VSH             */
    l[2 * 12 + 0] = 0x50;                 /* row 2: A,B=VSH           */
    l[3 * 12 + 0] = 0x54;                 /* row 3: A,B,C=VSH         */
    lut_group(l, 0, T_ROW1, T_ROW2 - T_ROW1, T_ROW3 - T_ROW2, 0, 0);
}

/* ---------------------------- RAM planes -------------------------------- */
/* Mask: 1 bit per pixel, MSB first, 22 bytes per row (5808 bytes). 1 = update. */

/* bitsel: 0 -> plane 0x24 (inverted low pixel bit), 1 -> plane 0x26 (inverted
 * high pixel bit). Identical to the conversion in EPD_2IN7_V2_4GrayDisplay.
 * erase_pass: masked pixels get the "not keep" code, others the "keep" code. */
static void send_plane(UBYTE ram_cmd, const UBYTE *img, UBYTE bitsel,
                       const UBYTE *mask, UBYTE erase_pass)
{
    g_cmd(ram_cmd);
    g_burst_begin();
    for (UWORD y = 0; y < EPD_2IN7_V2_HEIGHT; y++) {
        for (UWORD xb = 0; xb < BYTES_PER_ROW; xb++) {
            UBYTE m = mask[(UDOUBLE)y * BYTES_PER_ROW + xb];
            UBYTE out;
            if (m == 0) {
                out = EPD_KEEP_FILL;                      /* nothing to do here */
            } else if (erase_pass) {
                out = (UBYTE)(EPD_KEEP_FILL ^ m);         /* masked: ~keep      */
            } else {
                const UBYTE *p = img + (UDOUBLE)y * IMG_STRIDE + xb * 2;
                UBYTE g = 0;
                for (UBYTE k = 0; k < 8; k++) {
                    UBYTE px  = (UBYTE)((p[k >> 2] >> (6 - 2 * (k & 3))) & 3);
                    UBYTE bit = (UBYTE)(((px >> bitsel) & 1) ^ 1);
                    g = (UBYTE)((g << 1) | bit);
                }
                out = (UBYTE)((g & m) | (EPD_KEEP_FILL & (UBYTE)~m));
            }
            DEV_SPI_WriteByte(out);
        }
    }
    g_burst_end();
}

static void send_setup_and_lut(const UBYTE *lut153)
{
    g_cmd(0x3C); g_data(0x80);                    /* border: no drive (as in Display_Partial) */

    g_cmd(0x2C); g_data(LUT_DATA_4Gray[158]);     /* same order as Init_4GRAY */
    g_cmd(0x3F); g_data(LUT_DATA_4Gray[153]);
    g_cmd(0x03); g_data(LUT_DATA_4Gray[154]);
    g_cmd(0x04); g_data(LUT_DATA_4Gray[155]);
                 g_data(LUT_DATA_4Gray[156]);
                 g_data(LUT_DATA_4Gray[157]);

    g_cmd(0x32);
    for (UWORD i = 0; i < 153; i++) g_data(lut153[i]);

    g_cmd(0x11); g_data(0x03);                    /* entry mode */
    g_cmd(0x44); g_data(0x00); g_data(0x15);      /* X: 0..21 bytes */
    g_cmd(0x45); g_data(0x00); g_data(0x00); g_data(0x07); g_data(0x01); /* Y: 0..263 */
    g_cmd(0x4E); g_data(0x00);
    g_cmd(0x4F); g_data(0x00); g_data(0x00);
}

/* ------------------------------ API ------------------------------------ */
void EPD_2IN7_V2_4Gray_PartialMask(const UBYTE *Image, const UBYTE *Mask)
{
    UBYTE lut[153];

    /* pass 1: masked pixels -> VSL extreme */
    lut_build_erase(lut);
    send_setup_and_lut(lut);
    send_plane(0x24, Image, 0, Mask, 1);
    send_plane(0x26, Image, 1, Mask, 1);
    g_turn_on();

    /* pass 2: draw the grays of Image into the masked pixels */
    lut_build_draw(lut);
    send_setup_and_lut(lut);
    send_plane(0x24, Image, 0, Mask, 0);
    send_plane(0x26, Image, 1, Mask, 0);
    g_turn_on();
}

void EPD_2IN7_V2_4Gray_Partial(const UBYTE *Image,
                               UWORD Xstart, UWORD Ystart,
                               UWORD Xend,   UWORD Yend)
{
    static UBYTE mask[EPD_4GRAY_MASK_BYTES];

    if (Xend >= EPD_2IN7_V2_WIDTH)  Xend = EPD_2IN7_V2_WIDTH - 1;
    if (Yend >= EPD_2IN7_V2_HEIGHT) Yend = EPD_2IN7_V2_HEIGHT - 1;
    if (Xstart > Xend || Ystart > Yend) return;

    memset(mask, 0, sizeof mask);
    for (UWORD y = Ystart; y <= Yend; y++)
        for (UWORD x = Xstart; x <= Xend; x++)
            EPD_4GRAY_MASK_SET(mask, x, y);

    EPD_2IN7_V2_4Gray_PartialMask(Image, mask);
}
