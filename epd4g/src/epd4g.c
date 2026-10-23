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
 * Non-blocking operation: every public operation is a "job" run by a small cooperative state
 * machine (line-number coroutine, see the CR_ macros). epd4g_poll() advances the job until it
 * has to wait (BUSY pin, a timer) or has sent EPD4G_ROWS_PER_POLL rows of RAM data, then
 * returns. The blocking functions are start + poll loop, so there is a single implementation.
 *
 * 1-bit mode: the stock EPD_2in7_V2 sequences (Init, Init_Fast, Display_Base, Display_Fast,
 * Clear, Display_Partial, Sleep), see epd4g_bw.h.
 *
 * Credits: the register sequence of epd4g_init() and the stock LUT table come from the
 * Waveshare e-Paper library (https://github.com/waveshareteam/e-Paper, MIT-style
 * license, (c) Waveshare team); see README.md.
 */
#include "epd4g.h"
#include "epd4g_bw.h"
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
EPD4G_STATIC_ASSERT(rows_per_poll,  EPD4G_ROWS_PER_POLL >= 1);
EPD4G_STATIC_ASSERT(bw_threshold,   EPD4G_BW_THRESHOLD >= 1 && EPD4G_BW_THRESHOLD <= 3);
/* keeping the analog part on is incompatible with the hardware reset before a partial update */
EPD4G_STATIC_ASSERT(bw_keep_vs_reset, !(EPD4G_BW_KEEP_POWER && EPD4G_BW_PARTIAL_RESET));

#ifdef EPD4G_PROFILE
static epd4g_profile_t epd4g_prof;
static uint32_t prof_t;
const epd4g_profile_t *epd4g_get_profile(void) { return &epd4g_prof; }
#define PROF(field)  do { epd4g_prof.field = (uint16_t)(epd4g_port_tick_ms() - prof_t); \
                          prof_t = epd4g_port_tick_ms(); } while (0)
#define PROF_BEGIN() do { prof_t = epd4g_port_tick_ms(); memset(&epd4g_prof, 0, sizeof epd4g_prof); } while (0)
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
enum { MODE_NONE = 0, MODE_GRAY, MODE_BW };
enum { JOB_NONE = 0, JOB_INIT, JOB_FULL, JOB_PARTIAL, JOB_BW_INIT, JOB_BW_FULL, JOB_BW_PARTIAL,
       JOB_POWER_OFF, JOB_SLEEP };

static uint8_t s_mode;        /* MODE_xxx: set when an init job has finished, cleared by a timeout / sleep */
static uint8_t s_io_err;      /* sticky: an SPI write failed during the current job */
static uint8_t s_powered;     /* analog part left powered by the last update (KEEP_POWER variants) */
static uint8_t s_base;        /* 1-bit: a full update was done since the last init (RAM planes valid) */
static uint8_t s_win_full;    /* 1-bit: RAM window registers cover the whole panel */
static uint8_t s_border_part; /* 1-bit: border register still holds the partial-update value */
static uint8_t s_fast_lut;    /* 1-bit: the Init_Fast waveform is loaded (EPD4G_BW_FAST full is possible) */
static uint8_t s_asleep;      /* 1-bit: deep sleep entered; RAM planes kept, a partial update wakes the panel by its reset */
static uint8_t s_data;        /* the running job may still read the caller's buffers */
static uint8_t s_waiting;     /* the last poll stopped on a timer / BUSY wait (not on a data slice) */
static int     s_result;      /* status of the last finished job */
static epd4g_done_cb_t s_cb;
static void   *s_cb_ctx;

/* The running job. Everything a job needs after a yield lives here, locals do not survive. */
static struct {
    uint8_t  kind;            /* JOB_xxx */
    uint16_t pc;              /* coroutine resume point */
    int      rc;              /* result */
    uint32_t t0;              /* start of the current wait */
    int      bs;              /* busy_poll() result */
    const uint8_t *fb;        /* 4-gray frame, 2 bpp */
    const uint8_t *mask;
    const void    *img;       /* 1-bit job source */
    uint8_t  src;             /* epd4g_src_t */
    uint8_t  fullmode;        /* epd4g_full_mode_t */
    uint8_t  bwmode;          /* epd4g_bw_mode_t */
    uint8_t  any_gray;        /* partial: some masked pixel is gray -> pass 2 needed */
    int      xb0, xb1, y0, y1;  /* 1-bit partial window: byte columns, rows */
    uint8_t  ctl;             /* value for register 0x22 */
} J;

/* The RAM plane being sent, in slices of EPD4G_ROWS_PER_POLL rows. */
typedef enum { PASS_FULL, PASS_1, PASS_2, PASS_BW } pass_t;
static struct {
    uint8_t  plane;           /* 0 -> RAM 0x24, 1 -> RAM 0x26 */
    pass_t   pass;
    int      y, y1;
    int      xb0, xb1;        /* PASS_BW: byte columns */
} P;

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

/* ------------------------- coroutine helpers ------------------------------ */
/* A job function is `switch (J.pc) { case 0: ... }`; a wait stores a unique case label in J.pc and
 * returns, the next poll jumps back to it. __COUNTER__ makes the labels unique even when several
 * macros sit on one source line. A job function returns 1 (pending) or 0 (finished, result in J.rc). */
#define CR_BEGIN        switch (J.pc) { case 0:
#define CR_END          } J.pc = 0; return 0
#define CR_WAIT_N(cond, wait, n) \
    do { J.pc = (uint16_t)((n) + 1); /* fallthrough */ case (n) + 1: \
         if (!(cond)) { s_waiting = (wait); return 1; } } while (0)
#define CR_WAIT(cond)   CR_WAIT_N(cond, 1, __COUNTER__)    /* timer / BUSY: sleep a bit */
#define CR_SLICE(cond)  CR_WAIT_N(cond, 0, __COUNTER__)    /* more data to send: come back at once */
#define FAIL(code)      do { J.rc = (code); J.pc = 0; return 0; } while (0)

static int elapsed(uint32_t t0, uint32_t ms)
{
    return (uint32_t)(epd4g_port_tick_ms() - t0) >= ms;
}

#define WAIT_MS(ms)     do { J.t0 = epd4g_port_tick_ms(); CR_WAIT(elapsed(J.t0, (ms))); } while (0)

/* 0 = still busy, 1 = ready, -1 = timeout (the panel is then considered uninitialised) */
static int busy_poll(uint32_t t0, const char *where)
{
    (void)where;                                             /* used by EPD4G_LOG only */
    if (epd4g_port_gpio_busy() == 0)                         /* BUSY = 1 -> busy */
        return 1;
    if (elapsed(t0, EPD4G_BUSY_TIMEOUT_MS)) {
        EPD4G_LOG("[epd4g] BUSY timeout in %s\r\n", where);
        s_mode = MODE_NONE;
        s_powered = 0;
        return -1;
    }
    return 0;
}

/* wait for BUSY low, then the settle pause (stock Waveshare code uses 20 ms) */
#define WAIT_BUSY(where) \
    do { J.t0 = epd4g_port_tick_ms(); \
         CR_WAIT((J.bs = busy_poll(J.t0, (where))) != 0); \
         if (J.bs < 0) FAIL(EPD4G_ERR_TIMEOUT); \
         if (EPD4G_BUSY_SETTLE_MS > 0) WAIT_MS(EPD4G_BUSY_SETTLE_MS); } while (0)

/* hardware reset pulse (same timing as the stock library) */
#define HW_RESET() \
    do { epd4g_port_gpio_cs(1); epd4g_port_gpio_dc(0); \
         epd4g_port_gpio_rst(1); WAIT_MS(20); \
         epd4g_port_gpio_rst(0); WAIT_MS(2); \
         epd4g_port_gpio_rst(1); WAIT_MS(20); } while (0)

/* Start a display update with register 0x22 = ctl (LUT from register or OTP, see callers). */
static void g_trigger(uint8_t ctl)
{
    g_cmd(0x22);
    g_data(ctl);
    g_cmd(0x20);
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
 *   tbk: pixel is black    tgr: pixel is light or dark gray
 *   tw : pixel >= EPD4G_BW_THRESHOLD (white in 1-bit mode)                                   */
static uint8_t t24[256], t26[256], tbk[256], tgr[256], tw[256];

static void tables_init(void)
{
    for (int b = 0; b < 256; b++) {
        uint8_t a = 0, c = 0, k = 0, g = 0, w = 0;
        for (int j = 0; j < 4; j++) {
            uint8_t px  = (uint8_t)((b >> (6 - 2 * j)) & 3);
            uint8_t bit = (uint8_t)(1u << (3 - j));
            if (!(px & 1)) a |= bit;
            if (!(px & 2)) c |= bit;
            if (px == 0)   k |= bit;
            if (px == 1 || px == 2) g |= bit;
            if (px >= EPD4G_BW_THRESHOLD) w |= bit;
        }
        t24[b] = a; t26[b] = c; tbk[b] = k; tgr[b] = g; tw[b] = w;
    }
}

static uint8_t pk(const uint8_t *t, const uint8_t *p)
{
    return (uint8_t)((t[p[0]] << 4) | t[p[1]]);
}

/* 1 if some masked pixel has a gray target (pass 2 is needed) */
static uint8_t mask_has_gray(const uint8_t *fb, const uint8_t *mask)
{
    for (int y = 0; y < EPD4G_HEIGHT; y++) {
        const uint8_t *irow = fb + (size_t)y * EPD4G_FB_STRIDE;
        const uint8_t *mrow = mask + (size_t)y * EPD4G_MASK_STRIDE;
        for (int xb = 0; xb < EPD4G_MASK_STRIDE; xb++)
            if (mrow[xb] && (pk(tgr, irow + xb * 2) & mrow[xb]))
                return 1;
    }
    return 0;
}

/* Begin sending a plane (command + row counters); plane_step() then sends the rows.
 * plane: 0 -> RAM 0x24, 1 -> RAM 0x26. mask bit 1 = pixel is updated (NULL = all).
 * RAM pair (0x24,0x26) per pixel: (0,0) keep; PASS_1: black (1,1), other masked (1,0);
 * PASS_2: light/dark gray = the pair of the stock encoding, everything else (0,0).
 * PASS_BW: 1 bit per pixel, rows y0..y1, byte columns xb0..xb1 of the 1-bit image. */
static void plane_begin(uint8_t plane, pass_t pass, int y0, int y1, int xb0, int xb1)
{
    P.plane = plane; P.pass = pass;
    P.y = y0; P.y1 = y1; P.xb0 = xb0; P.xb1 = xb1;
    g_cmd(plane ? 0x26 : 0x24);
}

/* Send up to EPD4G_ROWS_PER_POLL rows; returns 1 while rows remain. CS is released after every
 * slice, so nothing is left half-way when the main loop does other work in between. */
static int plane_step(void)
{
    uint8_t row[EPD4G_MASK_STRIDE];
    int rows = EPD4G_ROWS_PER_POLL;

    g_burst_begin();
    while (P.y <= P.y1 && rows-- > 0) {
        const int y = P.y++;
        if (P.pass == PASS_BW) {
            const int n = P.xb1 - P.xb0 + 1;
            if (J.src == EPD4G_SRC_1BPP) {
                spi((const uint8_t *)J.img + (size_t)y * EPD4G_BW_STRIDE + P.xb0, (size_t)n);
            } else {
                if (J.src == EPD4G_SRC_WHITE) {
                    memset(row, 0xFF, (size_t)n);
                } else {
                    const uint8_t *irow = (const uint8_t *)J.img + (size_t)y * EPD4G_FB_STRIDE;
                    for (int i = 0; i < n; i++)
                        row[i] = pk(tw, irow + (P.xb0 + i) * 2);
                }
                spi(row, (size_t)n);
            }
        } else {
            const uint8_t *t = P.plane ? t26 : t24;
            const uint8_t *irow = J.fb + (size_t)y * EPD4G_FB_STRIDE;
            const uint8_t *mrow = J.mask ? J.mask + (size_t)y * EPD4G_MASK_STRIDE : NULL;
            for (int xb = 0; xb < EPD4G_MASK_STRIDE; xb++) {
                const uint8_t *p = irow + xb * 2;
                uint8_t m = mrow ? mrow[xb] : 0xFF;
                uint8_t out;
                if (P.pass == PASS_FULL) out = pk(t, p);
                else if (m == 0)         out = 0;                              /* keep */
                else if (P.pass == PASS_1) out = P.plane ? (uint8_t)(m & pk(tbk, p)) : m;
                else                     out = (uint8_t)(pk(t, p) & m & pk(tgr, p));
                row[xb] = out;
            }
            spi(row, EPD4G_MASK_STRIDE);
        }
    }
    g_burst_end();
    return P.y <= P.y1;
}

/* Registers common to all 4-gray passes. border: 0x80 = no border drive (partial), 0x00 = stock. */
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

/* RAM cursor to the window start */
static void send_cursor(int xb, int y)
{
    g_cmd(0x4E); g_data((uint8_t)xb);
    g_cmd(0x4F); g_data((uint8_t)(y & 0xFF)); g_data((uint8_t)((y >> 8) & 0x01));
}

/* LUT + RAM cursor to 0,0 (before the planes of every 4-gray pass) */
static void send_lut_and_cursor(const uint8_t *lut153)
{
    g_cmd(0x32);
    g_burst_begin();
    spi(lut153, 153);
    g_burst_end();

    send_cursor(0, 0);
}

/* 1-bit: RAM window registers */
static void send_window(int xb0, int xb1, int y0, int y1)
{
    g_cmd(0x44); g_data((uint8_t)xb0); g_data((uint8_t)xb1);
    g_cmd(0x45); g_data((uint8_t)(y0 & 0xFF)); g_data((uint8_t)((y0 >> 8) & 0x01));
                 g_data((uint8_t)(y1 & 0xFF)); g_data((uint8_t)((y1 >> 8) & 0x01));
}

/* Common tail of a job: map a sticky SPI error to a status. */
static int finish(int rc)
{
    if (rc == EPD4G_OK && s_io_err)
        rc = EPD4G_ERR_IO;
    s_io_err = 0;
    return rc;
}

/* ------------------------------- jobs ------------------------------------- */
/* power off, used before a reset when the analog part was left on */
#define POWER_OFF_SEQ(where) \
    do { g_trigger(0x83); WAIT_BUSY(where); s_powered = 0; } while (0)

static int job_init(void)
{
    CR_BEGIN
    if (s_powered) POWER_OFF_SEQ("pre-init power off");
    HW_RESET();
    WAIT_BUSY("reset");
    g_cmd(0x12);                                  /* soft reset */
    WAIT_BUSY("soft reset");

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
    g_burst_begin(); spi(lut_stock, 153); g_burst_end();      /* stock 4-gray LUT */
    send_cursor(0, 0);
    WAIT_BUSY("init");

    s_mode = MODE_GRAY;
    EPD4G_LOG("[epd4g] init done\r\n");
    CR_END;
}

static int job_full(void)
{
    uint8_t lut[153];
    CR_BEGIN
    PROF_BEGIN();
    lut_build_full(lut, (epd4g_full_mode_t)J.fullmode);
    send_setup(0x00);
    send_lut_and_cursor(lut);
    PROF(setup);
    plane_begin(0, PASS_FULL, 0, EPD4G_HEIGHT - 1, 0, 0);
    CR_SLICE(plane_step() == 0);
    plane_begin(1, PASS_FULL, 0, EPD4G_HEIGHT - 1, 0, 0);
    CR_SLICE(plane_step() == 0);
    s_data = 0;
    PROF(spi1);
    /* 0xC7 = clock+analog on, display, analog+clock off (stock, powers down)
     * 0xC4 = clock+analog on, display, stay powered                         */
    g_trigger(EPD4G_KEEP_POWER ? 0xC4 : 0xC7);
    s_powered = EPD4G_KEEP_POWER;
    WAIT_BUSY("full refresh");
    PROF(wait1);
    CR_END;
}

static int job_partial(void)
{
    uint8_t lut[153];
    CR_BEGIN
    PROF_BEGIN();
    J.any_gray = mask_has_gray(J.fb, J.mask);
    lut_build_p1(lut);
    send_setup(0x80);
    send_lut_and_cursor(lut);
    PROF(setup);
    plane_begin(0, PASS_1, 0, EPD4G_HEIGHT - 1, 0, 0);
    CR_SLICE(plane_step() == 0);
    plane_begin(1, PASS_1, 0, EPD4G_HEIGHT - 1, 0, 0);
    CR_SLICE(plane_step() == 0);
    if (!J.any_gray) s_data = 0;
    PROF(spi1);
    g_trigger(EPD4G_KEEP_POWER ? 0xC4 : 0xC7);
    s_powered = EPD4G_KEEP_POWER;
    WAIT_BUSY("partial pass 1");
    PROF(wait1);

    if (J.any_gray) {                             /* pass 2 only draws the gray levels */
        lut_build_p2(lut);
        send_lut_and_cursor(lut);
        plane_begin(0, PASS_2, 0, EPD4G_HEIGHT - 1, 0, 0);
        CR_SLICE(plane_step() == 0);
        plane_begin(1, PASS_2, 0, EPD4G_HEIGHT - 1, 0, 0);
        CR_SLICE(plane_step() == 0);
        s_data = 0;
        PROF(spi2);
        g_trigger(EPD4G_KEEP_POWER ? 0xC4 : 0xC7);
        WAIT_BUSY("partial pass 2");
        PROF(wait2);
    }
    CR_END;
}

static int job_power_off(void)
{
    CR_BEGIN
    g_trigger(0x83);                              /* clock on, analog off, clock off */
    WAIT_BUSY("power off");
    s_powered = 0;
    CR_END;
}

static int job_sleep(void)
{
    CR_BEGIN
    if ((s_mode == MODE_GRAY ? EPD4G_KEEP_POWER : EPD4G_BW_KEEP_POWER) || s_powered) {
        g_trigger(0x83);
        WAIT_BUSY("power off");
    }
    g_cmd(0x10);
    g_data(0x01);                                 /* deep sleep mode 1; RAM is lost, init to wake */
    s_powered = 0;
    if (s_mode == MODE_BW) {
        s_asleep = 1;                             /* RAM planes stay valid: partial updates can continue */
    } else {
        s_mode = MODE_NONE;
    }
    CR_END;
}

/* ---- 1-bit mode: stock EPD_2in7_V2 sequences ---- */
static int job_bw_init(void)
{
    CR_BEGIN
    if (s_powered) POWER_OFF_SEQ("pre-init power off");
    HW_RESET();
    WAIT_BUSY("reset");
    g_cmd(0x12);                                  /* SWRESET */
    WAIT_BUSY("soft reset");
    if (J.bwmode == EPD4G_BW_FAST) {
        g_cmd(0x18); g_data(0x80);                /* read built-in temperature sensor */
        g_cmd(0x22); g_data(0xB1);                /* load temperature value */
        g_cmd(0x20);
        WAIT_BUSY("temperature");
        g_cmd(0x1A); g_data(0x64); g_data(0x00);  /* write temperature register */
    }
    g_cmd(0x45); g_data(0x00); g_data(0x00); g_data(0x07); g_data(0x01);   /* RAM Y: 0..263 */
    g_cmd(0x4F); g_data(0x00); g_data(0x00);      /* RAM Y counter 0 */
    g_cmd(0x11); g_data(0x03);                    /* data entry mode */
    if (J.bwmode == EPD4G_BW_FAST) {
        g_cmd(0x22); g_data(0x91);                /* load the waveform for the temperature above */
        g_cmd(0x20);
        WAIT_BUSY("fast lut");
    }
    s_mode = MODE_BW;
    s_asleep = 0;
    s_base = 0;
    s_win_full = 1;                               /* power-on default of 0x44 is 0..21 bytes */
    s_border_part = 0;
    s_fast_lut = (J.bwmode == EPD4G_BW_FAST);
    CR_END;
}

static int job_bw_full(void)
{
    CR_BEGIN
    if (!s_win_full) {
        send_window(0, EPD4G_BW_STRIDE - 1, 0, EPD4G_HEIGHT - 1);
        s_win_full = 1;
    }
    if (s_border_part) {
        g_cmd(0x3C); g_data(EPD4G_BW_BORDER_FULL);
        s_border_part = 0;
    }
    /* stock Display_Base: the image goes to both planes, so the panel's "old" picture is right */
    send_cursor(0, 0);
    plane_begin(0, PASS_BW, 0, EPD4G_HEIGHT - 1, 0, EPD4G_BW_STRIDE - 1);
    CR_SLICE(plane_step() == 0);
    send_cursor(0, 0);
    plane_begin(1, PASS_BW, 0, EPD4G_HEIGHT - 1, 0, EPD4G_BW_STRIDE - 1);
    CR_SLICE(plane_step() == 0);
    s_data = 0;
    /* 0xF7: load temperature + LUT, display mode 1, power down (stock Display / Clear)
     * 0xC7: stock Display_Fast, uses the LUT loaded by Init_Fast                        */
    J.ctl = (J.bwmode == EPD4G_BW_FAST) ? 0xC7 : 0xF7;
    if (EPD4G_BW_KEEP_POWER) J.ctl = (uint8_t)(J.ctl & 0xFC);
    g_trigger(J.ctl);
    s_powered = EPD4G_BW_KEEP_POWER;
    WAIT_BUSY("bw full");
    s_base = 1;
    if (J.bwmode != EPD4G_BW_FAST) s_fast_lut = 0;      /* 0xF7 reloaded the normal waveform */
    CR_END;
}

static int job_bw_partial(void)
{
    CR_BEGIN
    if (EPD4G_BW_PARTIAL_RESET) {                 /* stock Display_Partial starts with a hardware reset */
        HW_RESET();                               /* ... which also wakes the panel from deep sleep */
    }
    s_asleep = 0;
    g_cmd(0x3C); g_data(0x80);                    /* border waveform */
    send_window(J.xb0, J.xb1, J.y0, J.y1);
    send_cursor(J.xb0, J.y0);
    plane_begin(0, PASS_BW, J.y0, J.y1, J.xb0, J.xb1);
    CR_SLICE(plane_step() == 0);
    s_data = 0;
    J.ctl = EPD4G_BW_KEEP_POWER ? 0xFC : 0xFF;    /* 0xFF: load LUT, display mode 2, power down */
    g_trigger(J.ctl);
    s_powered = EPD4G_BW_KEEP_POWER;
    s_win_full = (J.xb0 == 0 && J.xb1 == EPD4G_BW_STRIDE - 1 && J.y0 == 0 && J.y1 == EPD4G_HEIGHT - 1);
    s_border_part = 1;
    s_fast_lut = 0;                               /* the partial waveform replaced it */
    WAIT_BUSY("bw partial");
    CR_END;
}

/* ------------------------------ scheduler -------------------------------- */
static int job_run(void)
{
    switch (J.kind) {
    case JOB_INIT:       return job_init();
    case JOB_FULL:       return job_full();
    case JOB_PARTIAL:    return job_partial();
    case JOB_BW_INIT:    return job_bw_init();
    case JOB_BW_FULL:    return job_bw_full();
    case JOB_BW_PARTIAL: return job_bw_partial();
    case JOB_POWER_OFF:  return job_power_off();
    case JOB_SLEEP:      return job_sleep();
    default:             return 0;
    }
}

int epd4g_poll(void)
{
    int rc;

    if (J.kind == JOB_NONE)
        return 0;
    s_waiting = 0;
    if (job_run() != 0)
        return EPD4G_PENDING;

    rc = finish(J.rc);
    s_result = rc;
    s_data = 0;
    J.kind = JOB_NONE;
    J.pc = 0;
    if (s_cb != NULL)
        s_cb(rc, s_cb_ctx);
    return 0;
}

int epd4g_is_busy(void)            { return J.kind != JOB_NONE; }
int epd4g_data_busy(void)          { return s_data; }
int epd4g_result(void)             { return s_result; }
void epd4g_set_done_cb(epd4g_done_cb_t cb, void *ctx) { s_cb = cb; s_cb_ctx = ctx; }

int epd4g_wait(void)
{
    while (epd4g_poll() != 0) {
        if (s_waiting)
            epd4g_port_delay_ms(EPD4G_BUSY_POLL_MS);
    }
    return s_result;
}

void epd4g_abort(void)
{
    J.kind = JOB_NONE;
    J.pc = 0;
    s_mode = MODE_NONE;
    s_asleep = 0;
    s_powered = 0;
    s_data = 0;
    s_io_err = 0;
}

/* ------------------------------ start API -------------------------------- */
static int job_start(uint8_t kind)
{
    if (J.kind != JOB_NONE)
        return EPD4G_ERR_BUSY;
    J.kind = kind;
    J.pc = 0;
    J.rc = EPD4G_OK;
    s_io_err = 0;
    return EPD4G_OK;
}

static int need_mode(uint8_t mode)
{
    if (s_mode == MODE_NONE) return EPD4G_ERR_NOT_INIT;
    if (s_mode != mode)      return EPD4G_ERR_MODE;
    return EPD4G_OK;
}

int epd4g_start_init(void)
{
    int rc = job_start(JOB_INIT);
    if (rc != EPD4G_OK) return rc;
    s_mode = MODE_NONE;
    s_asleep = 0;
    tables_init();
    return EPD4G_OK;
}

int epd4g_start_full(const uint8_t *fb, epd4g_full_mode_t mode)
{
    int rc;
    if (fb == NULL) return EPD4G_ERR_PARAM;
    if (J.kind != JOB_NONE) return EPD4G_ERR_BUSY;
    if ((rc = need_mode(MODE_GRAY)) != EPD4G_OK) return rc;
    job_start(JOB_FULL); s_data = 1;
    J.fb = fb; J.mask = NULL; J.fullmode = (uint8_t)mode;
    return EPD4G_OK;
}

int epd4g_start_partial(const uint8_t *fb, const uint8_t *mask)
{
    int rc;
    if (fb == NULL || mask == NULL) return EPD4G_ERR_PARAM;
    if (J.kind != JOB_NONE) return EPD4G_ERR_BUSY;
    if ((rc = need_mode(MODE_GRAY)) != EPD4G_OK) return rc;
    job_start(JOB_PARTIAL); s_data = 1;
    J.fb = fb; J.mask = mask;
    return EPD4G_OK;
}

int epd4g_start_power_off(void)
{
    if (J.kind != JOB_NONE) return EPD4G_ERR_BUSY;
    if (s_mode == MODE_NONE) return EPD4G_ERR_NOT_INIT;
    if (s_asleep) { s_result = EPD4G_OK; return EPD4G_OK; }     /* already off */
    return job_start(JOB_POWER_OFF);
}

int epd4g_start_sleep(void)
{
    if (J.kind != JOB_NONE) return EPD4G_ERR_BUSY;
    if (s_mode == MODE_NONE) return EPD4G_ERR_NOT_INIT;
    if (s_asleep) { s_result = EPD4G_OK; return EPD4G_OK; }     /* already asleep: nothing to do */
    return job_start(JOB_SLEEP);
}

int epd4g_start_bw_init(epd4g_bw_mode_t mode)
{
    int rc = job_start(JOB_BW_INIT);
    if (rc != EPD4G_OK) return rc;
    s_mode = MODE_NONE;
    tables_init();
    J.bwmode = (uint8_t)mode;
    return EPD4G_OK;
}

int epd4g_start_bw_full(const void *img, epd4g_src_t src, epd4g_bw_mode_t mode)
{
    int rc;
    if (img == NULL && src != EPD4G_SRC_WHITE) return EPD4G_ERR_PARAM;
    if (src != EPD4G_SRC_1BPP && src != EPD4G_SRC_2BPP && src != EPD4G_SRC_WHITE) return EPD4G_ERR_PARAM;
    if (J.kind != JOB_NONE) return EPD4G_ERR_BUSY;
    if ((rc = need_mode(MODE_BW)) != EPD4G_OK) return rc;
    if (s_asleep) return EPD4G_ERR_NOT_INIT;                    /* a full refresh needs epd4g_bw_init() */
    if (mode == EPD4G_BW_FAST && !s_fast_lut) return EPD4G_ERR_MODE;
    job_start(JOB_BW_FULL); s_data = (src != EPD4G_SRC_WHITE);
    J.img = img; J.src = (uint8_t)src; J.bwmode = (uint8_t)mode;
    return EPD4G_OK;
}

int epd4g_start_bw_clear(epd4g_bw_mode_t mode)
{
    return epd4g_start_bw_full(NULL, EPD4G_SRC_WHITE, mode);
}

int epd4g_start_bw_partial(const void *img, epd4g_src_t src, const epd4g_rect_t *rect)
{
    int rc, x0 = 0, y0 = 0, x1 = EPD4G_WIDTH - 1, y1 = EPD4G_HEIGHT - 1;
    if (img == NULL && src != EPD4G_SRC_WHITE) return EPD4G_ERR_PARAM;
    if (src != EPD4G_SRC_1BPP && src != EPD4G_SRC_2BPP && src != EPD4G_SRC_WHITE) return EPD4G_ERR_PARAM;
    if (rect != NULL) {
        x0 = rect->x0; y0 = rect->y0; x1 = rect->x1; y1 = rect->y1;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 >= EPD4G_WIDTH)  x1 = EPD4G_WIDTH - 1;
        if (y1 >= EPD4G_HEIGHT) y1 = EPD4G_HEIGHT - 1;
        if (x0 > x1 || y0 > y1) return EPD4G_ERR_PARAM;
    }
    if (J.kind != JOB_NONE) return EPD4G_ERR_BUSY;
    if ((rc = need_mode(MODE_BW)) != EPD4G_OK) return rc;
    if (s_asleep && !EPD4G_BW_PARTIAL_RESET) return EPD4G_ERR_NOT_INIT;   /* no reset -> nothing wakes the panel */
    if (!s_base) return EPD4G_ERR_NO_BASE;
    job_start(JOB_BW_PARTIAL); s_data = (src != EPD4G_SRC_WHITE);
    J.img = img; J.src = (uint8_t)src;
    J.xb0 = x0 >> 3; J.xb1 = x1 >> 3; J.y0 = y0; J.y1 = y1;
    return EPD4G_OK;
}

/* ------------------------------ blocking API ----------------------------- */
static int blocking(int rc)
{
    return rc != EPD4G_OK ? rc : epd4g_wait();
}

int epd4g_init(void)                                 { return blocking(epd4g_start_init()); }
int epd4g_full(const uint8_t *fb, epd4g_full_mode_t m) { return blocking(epd4g_start_full(fb, m)); }
int epd4g_partial(const uint8_t *fb, const uint8_t *mask) { return blocking(epd4g_start_partial(fb, mask)); }
int epd4g_power_off(void)                            { return blocking(epd4g_start_power_off()); }
int epd4g_sleep(void)                                { return blocking(epd4g_start_sleep()); }
int epd4g_bw_init(epd4g_bw_mode_t m)                 { return blocking(epd4g_start_bw_init(m)); }
int epd4g_bw_full(const void *img, epd4g_src_t s, epd4g_bw_mode_t m) { return blocking(epd4g_start_bw_full(img, s, m)); }
int epd4g_bw_clear(epd4g_bw_mode_t m)                { return blocking(epd4g_start_bw_clear(m)); }
int epd4g_bw_partial(const void *img, epd4g_src_t s, const epd4g_rect_t *r) { return blocking(epd4g_start_bw_partial(img, s, r)); }
