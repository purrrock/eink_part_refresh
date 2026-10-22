/*****************************************************************************
* | File     : EPD_2in7_V2_4gray_partial.c
* | Function : partial and fast full refresh for the 4-gray mode, 2.7inch V2
*
* Why a special routine is needed
*   The stock LUT_DATA_4Gray is an ABSOLUTE waveform: every pixel is shaken
*   (VSH 20 / VSL 20 frames, x4) up to the VSL extreme and then moved towards
*   VSH by 0 / 3 / 8 / 25 frames (LUT rows 0..3). All 4 LUT rows are used for
*   the 4 gray levels, so there is no "do not touch this pixel" code, and the
*   0x20 trigger always updates the whole panel.
*
* Partial update = two passes with a custom LUT in register 0x32.
*   LUT row 0 is empty ("keep"): unmasked pixels are written as RAM pair (0,0).
*
*   pass 1 (T_ERASE / T_ROW3 frames, concurrently):
*     masked pixel, target black                -> row 3: VSH  T_ROW3  frames
*     masked pixel, any other target            -> rows 1/2: VSL T_ERASE frames
*   pass 2 (T_ROW2 frames):
*     masked pixel, target light / dark gray    -> rows 1/2: VSH T_ROW1 / T_ROW2
*   Black and white are saturated extremes, so they do not depend on the old
*   gray and need no erase first. Only the two intermediate grays have to start
*   from white, which pass 1 provides. Total = max(T_ERASE,T_ROW3)+T_ROW2 frames
*   (53 with the defaults) instead of T_ERASE+T_ROW3 (85) for erase-then-draw.
*   The RAM bits of a gray pixel are exactly the bits that
*   EPD_2IN7_V2_4GrayDisplay() writes, so which gray lands on which LUT row is
*   the same as in the stock waveform.
*
* Time = frames * 20 ms (frame rate set by the stock FR bytes) + fixed costs.
*   The fixed part was measured on hardware as ~370 ms per 0x22=0xC7 sequence
*   (that command powers the analog part up and down around every pass). With
*   EPD_4GRAY_KEEP_POWER the analog part stays on between passes and updates.
*
* Hardware history: with T_ROW3 = 25 (stock) black was under-driven (ghosts
* that vanish after a full refresh), hence 45. Tunables are marked [TUNE].
******************************************************************************/
#include "EPD_2in7_V2_4gray_partial.h"
#include "Debug.h"
#include <string.h>
#include <stdint.h>

/* Declared (non-static) in EPD_2in7_V2.c */
extern UBYTE LUT_DATA_4Gray[159];

/* [TUNE] frames. Every value can be overridden with -D<NAME>=<n> at build time. */
#ifndef T_ROW1
#define T_ROW1   3    /* light gray: VSH after the erase, frames */
#endif
#ifndef T_ROW2
#define T_ROW2   8    /* dark gray: cumulative VSH after the erase, frames (>= T_ROW1) */
#endif
#ifndef T_ROW3
#define T_ROW3   45   /* black: VSH that saturates a pixel to black from any state */
#endif
#ifndef T_ERASE
#define T_ERASE  40   /* VSL that saturates a pixel to white from any state */
#endif

/* [TUNE] 1 = leave the analog part powered after an update (0x22=0xC4) instead of
 * powering it up and down around every pass (0x22=0xC7, ~330 ms each). Call
 * EPD_2IN7_V2_4Gray_PowerOff() when the display goes idle. */
#ifndef EPD_4GRAY_KEEP_POWER
#define EPD_4GRAY_KEEP_POWER 1
#endif

/* [TUNE] full refresh: shake group 1 of the stock LUT is TPA=TPC=20 frames (+1+1),
 * repeated RP+1 = 4 times. RP=3/TP=20 is the stock 262-frame waveform;
 * RP=1 -> 178 frames (no ghosts on hardware), RP=0 -> 136 frames. */
#ifndef EPD_FULL_SHAKE_RP
#define EPD_FULL_SHAKE_RP  0
#endif
#ifndef EPD_FULL_SHAKE_TP
#define EPD_FULL_SHAKE_TP  20
#endif

/* [TUNE] pause after BUSY falls, ms (the stock library uses 20) */
#ifndef EPD_4GRAY_BUSY_SETTLE_MS
#define EPD_4GRAY_BUSY_SETTLE_MS 2
#endif

/* [TUNE] BUSY timeout. A stuck panel must not hang the MCU. */
#ifndef EPD_4GRAY_BUSY_TIMEOUT_MS
#define EPD_4GRAY_BUSY_TIMEOUT_MS  10000U
#endif

#define BYTES_PER_ROW   (EPD_2IN7_V2_WIDTH / 8)    /* 22 plane bytes / 44 image bytes */
#define IMG_STRIDE      (EPD_2IN7_V2_WIDTH / 4)

#ifdef EPD_4GRAY_PROFILE
EPD_4Gray_Profile epd_4gray_prof;
#define PROF(f)  do { epd_4gray_prof.f = (uint16_t)(HAL_GetTick() - prof_t); prof_t = HAL_GetTick(); } while (0)
#else
#define PROF(f)  do { } while (0)
#endif

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

/* returns 0 = ready, -1 = timeout */
static int g_busy(void)
{
    uint32_t t0 = HAL_GetTick();
    while (DEV_Digital_Read(EPD_BUSY_PIN) != 0) {       /* BUSY = 1 -> busy */
        if ((uint32_t)(HAL_GetTick() - t0) >= EPD_4GRAY_BUSY_TIMEOUT_MS)
            return -1;
        DEV_Delay_ms(1);
    }
#if EPD_4GRAY_BUSY_SETTLE_MS > 0
    DEV_Delay_ms(EPD_4GRAY_BUSY_SETTLE_MS);
#endif
    return 0;
}

/* Display update, LUT from register, mode 1.
 * 0xC7 = clock+analog on, display, analog+clock off   (stock, powers down)
 * 0xC4 = clock+analog on, display, stay powered       */
static int g_turn_on(UBYTE ctrl)
{
    g_cmd(0x22);
    g_data(ctrl);
    g_cmd(0x20);
    return g_busy();
}

/* ------------------------------ LUT ------------------------------------ */
/* Layout: [0..59] VS 5 rows x 12 groups (byte = phases A,B,C,D, 2 bit each,
 * 01=VSH1 10=VSL), [60..143] 12 groups x {TPA,TPB,SRAB,TPC,TPD,SRCD,RP},
 * [144..152] FR + XON (copied from the stock LUT). */
#define VSH_A   0x40
#define VSH_AB  0x50
#define VSL_A   0x80

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

/* pass 1: row 3 (black) VSH T_ROW3; rows 1,2 (everything else) VSL T_ERASE.
 * Both run from frame 0; the shorter one is simply idle afterwards. */
static void lut_build_p1(UBYTE *l)
{
    const int common = T_ERASE < T_ROW3 ? T_ERASE : T_ROW3;
    const int diff   = T_ERASE < T_ROW3 ? T_ROW3 - T_ERASE : T_ERASE - T_ROW3;
    int g = 0;

    lut_base(l);
    if (common > 0) {
        l[1 * 12 + g] = VSL_A;
        l[2 * 12 + g] = VSL_A;
        l[3 * 12 + g] = VSH_A;
        lut_group(l, g++, (UBYTE)common, 0, 0, 0, 0);
    }
    if (diff > 0) {
        if (T_ERASE > T_ROW3) { l[1 * 12 + g] = VSL_A; l[2 * 12 + g] = VSL_A; }
        else                  { l[3 * 12 + g] = VSH_A; }
        lut_group(l, g++, (UBYTE)diff, 0, 0, 0, 0);
    }
}

/* pass 2: row 1 VSH T_ROW1 frames, row 2 VSH T_ROW2 frames; rows 0 and 3 idle */
static void lut_build_p2(UBYTE *l)
{
    lut_base(l);
    l[1 * 12 + 0] = VSH_A;
    l[2 * 12 + 0] = VSH_AB;
    lut_group(l, 0, T_ROW1, T_ROW2 - T_ROW1, 0, 0, 0);
}

/* full refresh: stock LUT with an adjustable shake group (group 1) */
static void lut_build_full(UBYTE *l)
{
    memcpy(l, LUT_DATA_4Gray, 153);
    l[60 + 1 * 7 + 0] = EPD_FULL_SHAKE_TP;      /* TPA */
    l[60 + 1 * 7 + 3] = EPD_FULL_SHAKE_TP;      /* TPC */
    l[60 + 1 * 7 + 6] = EPD_FULL_SHAKE_RP;      /* RP  */
}

/* ---------------------------- RAM planes -------------------------------- */
/* Image pixel = 2 bits (11 white, 10 light, 01 dark, 00 black), 4 px per byte, MSB
 * first. A plane byte holds 8 pixels = 2 image bytes. These tables turn the 4 pixels
 * of one image byte into 4 plane bits (bit 3 = first pixel):
 *   t24: inverted low pixel bit  (RAM 0x24, as in EPD_2IN7_V2_4GrayDisplay)
 *   t26: inverted high pixel bit (RAM 0x26)
 *   tbk: pixel is black   tgr: pixel is light or dark gray                      */
static UBYTE t24[256], t26[256], tbk[256], tgr[256];
static UBYTE tables_ready;

static void tables_init(void)
{
    for (int b = 0; b < 256; b++) {
        UBYTE a = 0, c = 0, k = 0, g = 0;
        for (int j = 0; j < 4; j++) {
            UBYTE px = (UBYTE)((b >> (6 - 2 * j)) & 3);
            UBYTE bit = (UBYTE)(1u << (3 - j));
            if (!(px & 1)) a |= bit;
            if (!(px & 2)) c |= bit;
            if (px == 0)   k |= bit;
            if (px == 1 || px == 2) g |= bit;
        }
        t24[b] = a; t26[b] = c; tbk[b] = k; tgr[b] = g;
    }
    tables_ready = 1;
}

static inline UBYTE pk(const UBYTE *t, const UBYTE *p)
{
    return (UBYTE)((t[p[0]] << 4) | t[p[1]]);
}

typedef enum { PASS_FULL, PASS_1, PASS_2 } pass_t;

/* plane: 0 -> RAM 0x24, 1 -> RAM 0x26. Mask 1 = pixel is updated (NULL = all).
 * RAM pair (0x24,0x26) per pixel: (0,0) keep; PASS_1: black (1,1), other masked (1,0);
 * PASS_2: light/dark gray = the pair of the stock encoding, everything else (0,0). */
static void send_plane(UBYTE plane, pass_t pass, const UBYTE *img, const UBYTE *mask)
{
    UBYTE row[BYTES_PER_ROW];
    const UBYTE *t = plane ? t26 : t24;

    g_cmd(plane ? 0x26 : 0x24);
    g_burst_begin();
    for (UWORD y = 0; y < EPD_2IN7_V2_HEIGHT; y++) {
        const UBYTE *irow = img + (UDOUBLE)y * IMG_STRIDE;
        const UBYTE *mrow = mask ? mask + (UDOUBLE)y * BYTES_PER_ROW : NULL;
        for (UWORD xb = 0; xb < BYTES_PER_ROW; xb++) {
            const UBYTE *p = irow + xb * 2;
            UBYTE m = mrow ? mrow[xb] : 0xFF;
            UBYTE out;
            if (pass == PASS_FULL)  out = pk(t, p);
            else if (m == 0)        out = 0;                         /* keep */
            else if (pass == PASS_1) out = plane ? (UBYTE)(m & pk(tbk, p)) : m;
            else                    out = (UBYTE)(pk(t, p) & m & pk(tgr, p));
            row[xb] = out;
        }
        DEV_SPI_Write_nByte(row, BYTES_PER_ROW);
    }
    g_burst_end();
}

/* Registers common to all passes. border: 0x80 = no border drive (partial), 0x00 = stock. */
static void send_setup(UBYTE border)
{
    g_cmd(0x3C); g_data(border);

    g_cmd(0x2C); g_data(LUT_DATA_4Gray[158]);     /* same order as Init_4GRAY */
    g_cmd(0x3F); g_data(LUT_DATA_4Gray[153]);
    g_cmd(0x03); g_data(LUT_DATA_4Gray[154]);
    g_cmd(0x04); g_data(LUT_DATA_4Gray[155]);
                 g_data(LUT_DATA_4Gray[156]);
                 g_data(LUT_DATA_4Gray[157]);

    g_cmd(0x11); g_data(0x03);                    /* entry mode */
    g_cmd(0x44); g_data(0x00); g_data(0x15);      /* X: 0..21 bytes */
    g_cmd(0x45); g_data(0x00); g_data(0x00); g_data(0x07); g_data(0x01); /* Y: 0..263 */
}

/* LUT + RAM cursor to 0,0 (before the planes of every pass) */
static void send_lut_and_cursor(const UBYTE *lut153)
{
    g_cmd(0x32);
    g_burst_begin();
    DEV_SPI_Write_nByte((UBYTE *)lut153, 153);
    g_burst_end();

    g_cmd(0x4E); g_data(0x00);
    g_cmd(0x4F); g_data(0x00); g_data(0x00);
}

/* ------------------------------ API ------------------------------------ */
#if EPD_4GRAY_KEEP_POWER
#define CTRL_LAST  0xC4
#else
#define CTRL_LAST  0xC7
#endif

int EPD_2IN7_V2_4Gray_PartialMask(const UBYTE *Image, const UBYTE *Mask)
{
    UBYTE lut[153];
#ifdef EPD_4GRAY_PROFILE
    uint32_t prof_t = HAL_GetTick();
#endif

    if (!tables_ready) tables_init();

    lut_build_p1(lut);
    send_setup(0x80);
    send_lut_and_cursor(lut);
    PROF(setup1);
    send_plane(0, PASS_1, Image, Mask);
    send_plane(1, PASS_1, Image, Mask);
    PROF(spi1);
    if (g_turn_on(CTRL_LAST) != 0) return -1;
    PROF(wait1);

    lut_build_p2(lut);
    send_lut_and_cursor(lut);
    send_plane(0, PASS_2, Image, Mask);
    send_plane(1, PASS_2, Image, Mask);
    PROF(spi2);
    if (g_turn_on(CTRL_LAST) != 0) return -1;
    PROF(wait2);

    return 0;
}

int EPD_2IN7_V2_4Gray_Full(const UBYTE *Image)
{
    UBYTE lut[153];
#ifdef EPD_4GRAY_PROFILE
    uint32_t prof_t = HAL_GetTick();
#endif

    if (!tables_ready) tables_init();

    /* No hardware reset: the controller was initialised by EPD_2IN7_V2_Init_4GRAY(). */
    lut_build_full(lut);
    send_setup(0x00);
    send_lut_and_cursor(lut);
    PROF(setup1);
    send_plane(0, PASS_FULL, Image, NULL);
    send_plane(1, PASS_FULL, Image, NULL);
    PROF(spi1);
    int rc = g_turn_on(CTRL_LAST);
    PROF(wait1);
    return rc;
}

int EPD_2IN7_V2_4Gray_PowerOff(void)
{
    g_cmd(0x22);
    g_data(0x83);                                 /* clock on, analog off, clock off */
    g_cmd(0x20);
    return g_busy();
}

int EPD_2IN7_V2_4Gray_Partial(const UBYTE *Image,
                              UWORD Xstart, UWORD Ystart,
                              UWORD Xend,   UWORD Yend)
{
    static UBYTE mask[EPD_4GRAY_MASK_BYTES];

    if (Xend >= EPD_2IN7_V2_WIDTH)  Xend = EPD_2IN7_V2_WIDTH - 1;
    if (Yend >= EPD_2IN7_V2_HEIGHT) Yend = EPD_2IN7_V2_HEIGHT - 1;
    if (Xstart > Xend || Ystart > Yend) return 0;

    memset(mask, 0, sizeof mask);
    for (UWORD y = Ystart; y <= Yend; y++)
        for (UWORD x = Xstart; x <= Xend; x++)
            EPD_4GRAY_MASK_SET(mask, x, y);

    return EPD_2IN7_V2_4Gray_PartialMask(Image, mask);
}
