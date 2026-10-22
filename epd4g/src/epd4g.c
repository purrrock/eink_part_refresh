/**
 * @file epd4g.c
 * @brief Partial and fast full refresh in the 4-gray mode, Waveshare 2.7" e-Paper HAT V2.
 *
 * Why a special routine is needed
 *   The stock 4-gray LUT is an ABSOLUTE waveform: every pixel is shaken (VSH 20 /
 *   VSL 20 frames, x4) and then moved towards VSH by 0 / 3 / 8 / 25 frames (LUT rows
 *   0..3). All four LUT rows are used for the four gray levels, so there is no
 *   "do not touch this pixel" code, and the 0x20 trigger always updates the whole panel.
 *
 * Partial update = two passes with a custom LUT in register 0x32. LUT row 0 is empty
 * ("keep"): pixels outside the mask are written as RAM pair (0,0).
 *   pass 1 (max(T_WHITE,T_BLACK) frames, both at once):
 *     masked pixel, target black             -> row 3     : VSH T_BLACK frames
 *     masked pixel, any other target         -> rows 1, 2 : VSL T_WHITE frames
 *   pass 2 (T_DARK frames):
 *     masked pixel, target light / dark gray -> rows 1, 2 : VSH T_LIGHT / T_DARK frames
 * Black and white are saturated extremes, so they do not depend on the old gray and
 * need no erase first. Only the two intermediate grays must start from white, which
 * pass 1 provides.
 *
 * The RAM bits of a pixel are exactly the bits the stock 4-gray display routine writes,
 * so which gray lands on which LUT row is the same as in the stock waveform.
 *
 * Credits: the register sequence of epd4g_init() and the stock LUT table come from the
 * Waveshare e-Paper library (https://github.com/waveshareteam/e-Paper, MIT-style
 * license, (c) Waveshare team); see README.md.
 */
#include "epd4g.h"
#include "epd4g_port.h"
#include <string.h>

/* The conversion loops are ~10x slower at -O0 (Debug builds); force -O2 for this file. */
#if defined(__GNUC__) && !defined(__clang__) && !defined(__OPTIMIZE__) && !defined(EPD4G_NO_FORCE_O2)
#pragma GCC optimize ("O2")
#endif

/* ---------------------- compile-time configuration checks ---------------- */
#define EPD4G_STATIC_ASSERT(name, cond) typedef char epd4g_assert_##name[(cond) ? 1 : -1]
EPD4G_STATIC_ASSERT(light_le_dark,  EPD4G_T_LIGHT <= EPD4G_T_DARK);
EPD4G_STATIC_ASSERT(light_pos,      EPD4G_T_LIGHT >= 1);
EPD4G_STATIC_ASSERT(times_le_255,   EPD4G_T_BLACK <= 255 && EPD4G_T_WHITE <= 255 &&
                                    EPD4G_T_DARK <= 255 && EPD4G_FULL_SHAKE_TP <= 255 &&
                                    EPD4G_FULL_SHAKE_RP <= 255);
EPD4G_STATIC_ASSERT(black_white_pos, EPD4G_T_BLACK >= 1 && EPD4G_T_WHITE >= 1);

#ifdef EPD4G_PROFILE
#define PROF(field)  do { epd4g_prof.field = (uint16_t)(epd4g_port_tick_ms() - prof_t); \
                          prof_t = epd4g_port_tick_ms(); } while (0)
#define PROF_BEGIN() uint32_t prof_t = epd4g_port_tick_ms()
static epd4g_profile_t epd4g_prof;
const epd4g_profile_t *epd4g_get_profile(void) { return &epd4g_prof; }
#else
#define PROF(field)  do { } while (0)
#define PROF_BEGIN() do { } while (0)
#endif

/* ----------------------------- stock LUT --------------------------------- */
/* Waveshare LUT_DATA_4Gray: [0..152] LUT for register 0x32, then
 * [153] EOPQ (0x3F)  [154] VGH (0x03)  [155..157] VSH1, VSH2, VSL (0x04)  [158] VCOM (0x2C). */
static const uint8_t lut_stock[159] = {
    0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x48, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x48, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0A, 0x19, 0x00, 0x03, 0x08, 0x00, 0x00, 0x14, 0x01, 0x00, 0x14, 0x01,
    0x00, 0x03, 0x0A, 0x03, 0x00, 0x08, 0x19, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x22, 0x17, 0x41,
    0x00, 0x32, 0x1C
};

/* ---------------------------- driver state ------------------------------- */
static uint8_t s_ready;      /* epd4g_init() done and no fatal error since */
static uint8_t s_io_err;     /* sticky: an SPI write failed during the current call */

/* --------------------------- low level access ---------------------------- */
static void spi(const uint8_t *p, size_t n)
{
    if (epd4g_port_spi_write(p, n) != 0)
        s_io_err = 1;
}

static void g_cmd(uint8_t reg)
{
    epd4g_port_gpio_dc(0);
    epd4g_port_gpio_cs(0);
    spi(&reg, 1);
    epd4g_port_gpio_cs(1);
}

static void g_data(uint8_t d)
{
    epd4g_port_gpio_dc(1);
    epd4g_port_gpio_cs(0);
    spi(&d, 1);
    epd4g_port_gpio_cs(1);
}

static void g_burst_begin(void)
{
    epd4g_port_gpio_dc(1);
    epd4g_port_gpio_cs(0);
}

static void g_burst_end(void)
{
    epd4g_port_gpio_cs(1);
}

/* returns 0 = ready, EPD4G_ERR_TIMEOUT otherwise */
static int g_busy(const char *where)
{
    uint32_t t0 = epd4g_port_tick_ms();
    (void)where;                                             /* used by EPD4G_LOG only */
    while (epd4g_port_gpio_busy() != 0) {                    /* BUSY = 1 -> busy */
        if ((uint32_t)(epd4g_port_tick_ms() - t0) >= EPD4G_BUSY_TIMEOUT_MS) {
            EPD4G_LOG("[epd4g] BUSY timeout in %s\r\n", where);
            s_ready = 0;
            return EPD4G_ERR_TIMEOUT;
        }
        epd4g_port_delay_ms(EPD4G_BUSY_POLL_MS);
    }
#if EPD4G_BUSY_SETTLE_MS > 0
    epd4g_port_delay_ms(EPD4G_BUSY_SETTLE_MS);
#endif
    return EPD4G_OK;
}

/* Display update, LUT from register, mode 1.
 * 0xC7 = clock+analog on, display, analog+clock off (stock, powers down)
 * 0xC4 = clock+analog on, display, stay powered                         */
static int g_turn_on(const char *where)
{
    g_cmd(0x22);
#if EPD4G_KEEP_POWER
    g_data(0xC4);
#else
    g_data(0xC7);
#endif
    g_cmd(0x20);
    return g_busy(where);
}

/* ------------------------------- LUT ------------------------------------- */
/* Layout: [0..59] VS 5 rows x 12 groups (byte = phases A,B,C,D, 2 bit each,
 * 01=VSH1 10=VSL), [60..143] 12 groups x {TPA,TPB,SRAB,TPC,TPD,SRCD,RP},
 * [144..152] FR + XON (copied from the stock LUT). */
#define VSH_A   0x40
#define VSH_AB  0x50
#define VSL_A   0x80

static void lut_base(uint8_t *l)
{
    memset(l, 0, 153);
    memcpy(&l[144], &lut_stock[144], 9);
}

static void lut_group(uint8_t *l, int g, uint8_t tpa, uint8_t tpb, uint8_t tpc, uint8_t tpd, uint8_t rp)
{
    uint8_t *p = &l[60 + g * 7];
    p[0] = tpa; p[1] = tpb; p[2] = 0; p[3] = tpc; p[4] = tpd; p[5] = 0; p[6] = rp;
}

/* pass 1: row 3 (black) VSH T_BLACK; rows 1,2 (everything else) VSL T_WHITE.
 * Both start at frame 0; the shorter one is simply idle afterwards. */
static void lut_build_p1(uint8_t *l)
{
    const int common = EPD4G_T_WHITE < EPD4G_T_BLACK ? EPD4G_T_WHITE : EPD4G_T_BLACK;
    const int diff   = EPD4G_T_WHITE < EPD4G_T_BLACK ? EPD4G_T_BLACK - EPD4G_T_WHITE
                                                     : EPD4G_T_WHITE - EPD4G_T_BLACK;
    int g = 0;

    lut_base(l);
    l[1 * 12 + g] = VSL_A;
    l[2 * 12 + g] = VSL_A;
    l[3 * 12 + g] = VSH_A;
    lut_group(l, g++, (uint8_t)common, 0, 0, 0, 0);
    if (diff > 0) {
        if (EPD4G_T_WHITE > EPD4G_T_BLACK) { l[1 * 12 + g] = VSL_A; l[2 * 12 + g] = VSL_A; }
        else                               { l[3 * 12 + g] = VSH_A; }
        lut_group(l, g++, (uint8_t)diff, 0, 0, 0, 0);
    }
}

/* pass 2: row 1 VSH T_LIGHT frames, row 2 VSH T_DARK frames; rows 0 and 3 idle */
static void lut_build_p2(uint8_t *l)
{
    lut_base(l);
    l[1 * 12 + 0] = VSH_A;
    l[2 * 12 + 0] = VSH_AB;
    lut_group(l, 0, EPD4G_T_LIGHT, (uint8_t)(EPD4G_T_DARK - EPD4G_T_LIGHT), 0, 0, 0);
}

/* full refresh: stock LUT; FAST adjusts the shake group (group 1) */
static void lut_build_full(uint8_t *l, epd4g_full_mode_t mode)
{
    memcpy(l, lut_stock, 153);
    if (mode == EPD4G_FULL_FAST) {
        l[60 + 1 * 7 + 0] = EPD4G_FULL_SHAKE_TP;      /* TPA */
        l[60 + 1 * 7 + 3] = EPD4G_FULL_SHAKE_TP;      /* TPC */
        l[60 + 1 * 7 + 6] = EPD4G_FULL_SHAKE_RP;      /* RP  */
    }
}

/* ---------------------------- RAM planes --------------------------------- */
/* Image pixel = 2 bits (11 white, 10 light, 01 dark, 00 black), 4 px per byte, MSB first.
 * A plane byte holds 8 pixels = 2 image bytes. These tables turn the 4 pixels of one
 * image byte into 4 plane bits (bit 3 = first pixel):
 *   t24: inverted low pixel bit  (RAM 0x24, as in the stock 4-gray display routine)
 *   t26: inverted high pixel bit (RAM 0x26)
 *   tbk: pixel is black    tgr: pixel is light or dark gray                          */
static uint8_t t24[256], t26[256], tbk[256], tgr[256];

static void tables_init(void)
{
    for (int b = 0; b < 256; b++) {
        uint8_t a = 0, c = 0, k = 0, g = 0;
        for (int j = 0; j < 4; j++) {
            uint8_t px  = (uint8_t)((b >> (6 - 2 * j)) & 3);
            uint8_t bit = (uint8_t)(1u << (3 - j));
            if (!(px & 1)) a |= bit;
            if (!(px & 2)) c |= bit;
            if (px == 0)   k |= bit;
            if (px == 1 || px == 2) g |= bit;
        }
        t24[b] = a; t26[b] = c; tbk[b] = k; tgr[b] = g;
    }
}

static uint8_t pk(const uint8_t *t, const uint8_t *p)
{
    return (uint8_t)((t[p[0]] << 4) | t[p[1]]);
}

typedef enum { PASS_FULL, PASS_1, PASS_2 } pass_t;

/* plane: 0 -> RAM 0x24, 1 -> RAM 0x26. mask bit 1 = pixel is updated (NULL = all).
 * RAM pair (0x24,0x26) per pixel: (0,0) keep; PASS_1: black (1,1), other masked (1,0);
 * PASS_2: light/dark gray = the pair of the stock encoding, everything else (0,0). */
static void send_plane(uint8_t plane, pass_t pass, const uint8_t *fb, const uint8_t *mask)
{
    uint8_t row[EPD4G_MASK_STRIDE];
    const uint8_t *t = plane ? t26 : t24;

    g_cmd(plane ? 0x26 : 0x24);
    g_burst_begin();
    for (int y = 0; y < EPD4G_HEIGHT; y++) {
        const uint8_t *irow = fb + (size_t)y * EPD4G_FB_STRIDE;
        const uint8_t *mrow = mask ? mask + (size_t)y * EPD4G_MASK_STRIDE : NULL;
        for (int xb = 0; xb < EPD4G_MASK_STRIDE; xb++) {
            const uint8_t *p = irow + xb * 2;
            uint8_t m = mrow ? mrow[xb] : 0xFF;
            uint8_t out;
            if (pass == PASS_FULL)   out = pk(t, p);
            else if (m == 0)         out = 0;                              /* keep */
            else if (pass == PASS_1) out = plane ? (uint8_t)(m & pk(tbk, p)) : m;
            else                     out = (uint8_t)(pk(t, p) & m & pk(tgr, p));
            row[xb] = out;
        }
        spi(row, EPD4G_MASK_STRIDE);
    }
    g_burst_end();
}

/* Registers common to all passes. border: 0x80 = no border drive (partial), 0x00 = stock. */
static void send_setup(uint8_t border)
{
    g_cmd(0x3C); g_data(border);

    g_cmd(0x2C); g_data(lut_stock[158]);          /* same order as the stock init */
    g_cmd(0x3F); g_data(lut_stock[153]);
    g_cmd(0x03); g_data(lut_stock[154]);
    g_cmd(0x04); g_data(lut_stock[155]);
                 g_data(lut_stock[156]);
                 g_data(lut_stock[157]);

    g_cmd(0x11); g_data(0x03);                    /* entry mode */
    g_cmd(0x44); g_data(0x00); g_data(0x15);      /* X: 0..21 bytes */
    g_cmd(0x45); g_data(0x00); g_data(0x00); g_data(0x07); g_data(0x01); /* Y: 0..263 */
}

/* LUT + RAM cursor to 0,0 (before the planes of every pass) */
static void send_lut_and_cursor(const uint8_t *lut153)
{
    g_cmd(0x32);
    g_burst_begin();
    spi(lut153, 153);
    g_burst_end();

    g_cmd(0x4E); g_data(0x00);
    g_cmd(0x4F); g_data(0x00); g_data(0x00);
}

/* Common tail of every public call: map a sticky SPI error to a status. */
static int finish(int rc)
{
    if (rc == EPD4G_OK && s_io_err)
        rc = EPD4G_ERR_IO;
    s_io_err = 0;
    return rc;
}

/* ------------------------------- API ------------------------------------- */
int epd4g_init(void)
{
    int rc;

    s_ready = 0;
    s_io_err = 0;
    tables_init();

    /* hardware reset (same timing as the stock library) */
    epd4g_port_gpio_cs(1);
    epd4g_port_gpio_dc(0);
    epd4g_port_gpio_rst(1);
    epd4g_port_delay_ms(20);
    epd4g_port_gpio_rst(0);
    epd4g_port_delay_ms(2);
    epd4g_port_gpio_rst(1);
    epd4g_port_delay_ms(20);

    if ((rc = g_busy("reset")) != EPD4G_OK) return finish(rc);
    g_cmd(0x12);                                  /* soft reset */
    if ((rc = g_busy("soft reset")) != EPD4G_OK) return finish(rc);

    g_cmd(0x74); g_data(0x54);                    /* analog block control  */
    g_cmd(0x7E); g_data(0x3B);                    /* digital block control */
    g_cmd(0x01); g_data(0x07); g_data(0x01); g_data(0x00);   /* driver output control: 264 lines */
    g_cmd(0x11); g_data(0x03);                    /* data entry mode */
    g_cmd(0x44); g_data(0x00); g_data(0x15);      /* RAM X: 0..21 bytes */
    g_cmd(0x45); g_data(0x00); g_data(0x00); g_data(0x07); g_data(0x01);   /* RAM Y: 0..263 */
    g_cmd(0x3C); g_data(0x00);                    /* border waveform */
    g_cmd(0x2C); g_data(lut_stock[158]);          /* VCOM */
    g_cmd(0x3F); g_data(lut_stock[153]);          /* EOPQ */
    g_cmd(0x03); g_data(lut_stock[154]);          /* VGH  */
    g_cmd(0x04); g_data(lut_stock[155]);          /* VSH1 */
                 g_data(lut_stock[156]);          /* VSH2 */
                 g_data(lut_stock[157]);          /* VSL  */
    g_cmd(0x32);
    for (int i = 0; i < 153; i++) g_data(lut_stock[i]);      /* stock 4-gray LUT */
    g_cmd(0x4E); g_data(0x00);                    /* RAM cursor 0,0 */
    g_cmd(0x4F); g_data(0x00); g_data(0x00);
    if ((rc = g_busy("init")) != EPD4G_OK) return finish(rc);

    s_ready = 1;
    EPD4G_LOG("[epd4g] init done\r\n");
    return finish(EPD4G_OK);
}

int epd4g_partial(const uint8_t *fb, const uint8_t *mask)
{
    uint8_t lut[153];
    int rc;
    PROF_BEGIN();

    if (fb == NULL || mask == NULL) return EPD4G_ERR_PARAM;
    if (!s_ready) return EPD4G_ERR_NOT_INIT;
    s_io_err = 0;

    lut_build_p1(lut);
    send_setup(0x80);
    send_lut_and_cursor(lut);
    PROF(setup);
    send_plane(0, PASS_1, fb, mask);
    send_plane(1, PASS_1, fb, mask);
    PROF(spi1);
    if ((rc = g_turn_on("partial pass 1")) != EPD4G_OK) return finish(rc);
    PROF(wait1);

    lut_build_p2(lut);
    send_lut_and_cursor(lut);
    send_plane(0, PASS_2, fb, mask);
    send_plane(1, PASS_2, fb, mask);
    PROF(spi2);
    if ((rc = g_turn_on("partial pass 2")) != EPD4G_OK) return finish(rc);
    PROF(wait2);

    return finish(EPD4G_OK);
}

int epd4g_full(const uint8_t *fb, epd4g_full_mode_t mode)
{
    uint8_t lut[153];
    int rc;
    PROF_BEGIN();

    if (fb == NULL) return EPD4G_ERR_PARAM;
    if (!s_ready) return EPD4G_ERR_NOT_INIT;
    s_io_err = 0;

    lut_build_full(lut, mode);
    send_setup(0x00);
    send_lut_and_cursor(lut);
    PROF(setup);
    send_plane(0, PASS_FULL, fb, NULL);
    send_plane(1, PASS_FULL, fb, NULL);
    PROF(spi1);
    rc = g_turn_on("full refresh");
    PROF(wait1);
    return finish(rc);
}

int epd4g_power_off(void)
{
    int rc;

    if (!s_ready) return EPD4G_ERR_NOT_INIT;
    s_io_err = 0;
    g_cmd(0x22);
    g_data(0x83);                                 /* clock on, analog off, clock off */
    g_cmd(0x20);
    rc = g_busy("power off");
    return finish(rc);
}

int epd4g_sleep(void)
{
    if (!s_ready) return EPD4G_ERR_NOT_INIT;
#if EPD4G_KEEP_POWER
    {
        int rc = epd4g_power_off();
        if (rc != EPD4G_OK) return rc;
    }
#endif
    s_io_err = 0;
    g_cmd(0x10);
    g_data(0x01);                                 /* deep sleep mode 1; RAM is lost, init to wake */
    s_ready = 0;
    return finish(EPD4G_OK);
}
