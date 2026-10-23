/*
 * Host tests for epd4g: a simulated panel behind the port interface.
 *
 *  - byte-for-byte equality with the STOCK Waveshare library (golden.inc): Init_4GRAY
 *    register sequence, stock LUT, RAM planes of the 4-gray full display, Sleep;
 *  - an ink model that applies the LUT / RAM data the library sends to every pixel
 *    (VSH darkens, VSL lightens, saturating at both ends) and checks that after every
 *    partial update each pixel shows exactly its target gray and untouched pixels do not move;
 *  - waveform lengths, power commands, error handling, frame buffer / rotation / mask helpers.
 *
 * The 1-bit mode is checked against the stock Init / Init_Fast / Display_Base / Display_Fast /
 * Clear / Display_Partial / Sleep streams and with a model of the controller RAM (window, cursor, old/new
 * planes). The non-blocking interface is checked for bounded work per poll() call and for the same
 * results as the blocking one.
 *
 * Model assumption (cannot be seen from the host, verified visually on hardware): RAM pair
 * (0x24,0x26) selects LUT row (0,0)=0 (0,1)=1 (1,0)=2 (1,1)=3.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "epd4g.h"
#include "epd4g_fb.h"
#include "epd4g_bw.h"
#include "epd4g_port.h"
#include "golden.inc"

/* ------------------------------ simulated panel -------------------------- */
#define NAT_W EPD4G_WIDTH
#define NAT_H EPD4G_HEIGHT
#define DMAX  (EPD4G_T_WHITE < EPD4G_T_BLACK ? EPD4G_T_WHITE : EPD4G_T_BLACK)   /* ink saturation, frames of drive */

static int      dc, cs = 1, cur_cmd = -1;
static uint16_t stream[8192]; static int stream_n;           /* all SPI bytes; bit 8 = data */
static uint8_t  p24[EPD4G_MASK_BYTES], p26[EPD4G_MASK_BYTES]; static int n24, n26;
static uint8_t  lutb[400]; static int lutn;
static int      last_ctrl;
static int      rst_log[8], rst_n;
static uint32_t now_ms;
static int      busy_stuck, spi_fail, spi_calls;
static int      ctrl_c4, ctrl_c7, ctrl_other;
static uint32_t busy_until;              /* BUSY is high until this time (set by every 0x20) */
static long     spi_bytes;               /* data + command bytes sent in total */

/* 1-bit controller model: RAM window + cursor, old/new planes (1 = white), what the glass shows */
static int      sim_bw;
static uint8_t  bwn[EPD4G_BW_FB_BYTES], bwo[EPD4G_BW_FB_BYTES], bwshow[EPD4G_BW_FB_BYTES];
static int      wx0, wx1 = 21, wy0, wy1 = 263, cx, cy, nargs, args[8];
static int      bw_full_updates, bw_part_updates;

/* ink model state */
static int      dark[NAT_W * NAT_H];
static int      pass_no;                 /* 0 = next 0x20 is partial pass 1, 1 = pass 2 */
static int      mode_partial;            /* 1 while an epd4g_partial() call is being checked */
static const uint8_t *expect_fb;         /* picture the panel must show after the call */
static uint8_t  driven[NAT_W * NAT_H];   /* pixels touched by pass 1 */
static int      frames_pass[2];
static int      updates_seen;
static int      errors;

#define CHECK(cond, ...) do { if (!(cond)) { errors++; if (errors < 20) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

void epd4g_port_gpio_rst(int l)
{
    if (rst_n < 8) rst_log[rst_n++] = l;
    if (l == 0) { wx0 = 0; wx1 = 21; wy0 = 0; wy1 = 263; cx = cy = 0; }      /* controller defaults */
}
void epd4g_port_gpio_dc(int l)  { dc = l; }
void epd4g_port_gpio_cs(int l)  { cs = l; }
int  epd4g_port_gpio_busy(void) { return busy_stuck || now_ms < busy_until; }
void epd4g_port_delay_ms(uint32_t ms) { now_ms += ms; }
uint32_t epd4g_port_tick_ms(void) { return now_ms; }

static int pair(int x, int y) { int i = y * EPD4G_MASK_STRIDE + (x >> 3), b = 7 - (x & 7); return (((p24[i] >> b) & 1) << 1) | ((p26[i] >> b) & 1); }
static int code_of_pair(int pr) { switch (pr) { case 0: return 3; case 3: return 0; case 2: return 2; default: return 1; } }
static int dark_of_code(int c) { switch (c) { case 3: return 0; case 2: return EPD4G_T_DARK; case 1: return EPD4G_T_LIGHT; default: return DMAX; } }

static int lut_frames(void)
{
    int tot = 0;
    for (int g = 0; g < 12; g++) { uint8_t *t = &lutb[60 + g * 7]; tot += (t[0] + t[1] + t[3] + t[4]) * (t[6] + 1); }
    return tot;
}

static int run_lut(int row, int d)
{
    for (int g = 0; g < 12; g++) {
        uint8_t *t = &lutb[60 + g * 7];
        int dur[4] = { t[0], t[1], t[3], t[4] };
        for (int r = 0; r <= t[6]; r++)
            for (int ph = 0; ph < 4; ph++) {
                int v = (lutb[row * 12 + g] >> (6 - 2 * ph)) & 3;
                for (int f = 0; f < dur[ph]; f++) { if (v == 1) d++; else if (v == 2) d--; if (d < 0) d = 0; if (d > DMAX) d = DMAX; }
            }
    }
    return d;
}

static void bw_ram_write(int plane, uint8_t v)
{
    uint8_t *ram = plane ? bwo : bwn;
    if (cx >= 0 && cx < 22 && cy >= 0 && cy < 264) ram[cy * 22 + cx] = v;
    if (++cx > wx1) { cx = wx0; if (++cy > wy1) cy = wy0; }
}

static void bw_reg_arg(int cmd, int v)
{
    args[nargs < 8 ? nargs : 7] = v; nargs++;
    if (cmd == 0x44 && nargs == 2) { wx0 = args[0]; wx1 = args[1]; }
    if (cmd == 0x45 && nargs == 4) { wy0 = args[0] | (args[1] << 8); wy1 = args[2] | (args[3] << 8); }
    if (cmd == 0x4E && nargs == 1) cx = args[0];
    if (cmd == 0x4F && nargs == 2) cy = args[0] | (args[1] << 8);
}

static void bw_update(void)
{
    if (last_ctrl & 0x08) { /* display mode 2: only pixels whose new and old RAM differ are driven */
        for (int i = 0; i < EPD4G_BW_FB_BYTES; i++) bwshow[i] = (uint8_t)((bwshow[i] & ~(bwn[i] ^ bwo[i])) | (bwn[i] & (bwn[i] ^ bwo[i])));
        bw_part_updates++;
    } else {                /* mode 1: the whole panel shows the new RAM */
        memcpy(bwshow, bwn, sizeof bwshow);
        bw_full_updates++;
    }
    memcpy(bwo, bwn, sizeof bwo);
    updates_seen++;
}

static void display_update(void)
{
    if (sim_bw) { bw_update(); return; }
    updates_seen++;
    CHECK(lutn == 153, "LUT length %d", lutn);
    CHECK(n24 == EPD4G_MASK_BYTES && n26 == EPD4G_MASK_BYTES, "plane sizes %d %d", n24, n26);
    if (last_ctrl == 0xC4) ctrl_c4++; else if (last_ctrl == 0xC7) ctrl_c7++; else ctrl_other++;

    if (!mode_partial) {                               /* full refresh: panel shows the planes */
        for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++)
            dark[y * NAT_W + x] = dark_of_code(code_of_pair(pair(x, y)));
        return;
    }
    if (pass_no == 0) {                                /* pass 1 */
        frames_pass[0] = lut_frames();
        for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++) {
            int pr = pair(x, y), i = y * NAT_W + x;
            driven[i] = (pr != 0);
            if (pr) {
                CHECK((pr == 3) == (epd4g_fb_get(expect_fb, x, y) == EPD4G_BLACK), "pass 1 pair %d for target %d at %d,%d", pr, epd4g_fb_get(expect_fb, x, y), x, y);
                dark[i] = run_lut(pr, dark[i]);
            }
        }
        pass_no = 1;
    } else {                                           /* pass 2 */
        frames_pass[1] = lut_frames();
        for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++) {
            int pr = pair(x, y), i = y * NAT_W + x;
            CHECK(pr != 3, "pass 2 black pair at %d,%d", x, y);
            if (pr) { CHECK(driven[i], "pass 2 drives pixel not erased in pass 1 at %d,%d", x, y); dark[i] = run_lut(pr, dark[i]); }
        }
        pass_no = 0;
    }
}

int epd4g_port_spi_write(const uint8_t *d, size_t n)
{
    spi_calls++;
    if (spi_fail) return -1;
    if (cs) return 0;
    for (size_t k = 0; k < n; k++) {
        uint8_t v = d[k];
        if (stream_n < 8192) stream[stream_n++] = (uint16_t)(v | (dc ? 0x100 : 0));
        spi_bytes++;
        if (!dc) {
            cur_cmd = v; nargs = 0;
            if (v == 0x24) n24 = 0;
            if (v == 0x26) n26 = 0;
            if (v == 0x32) lutn = 0;
            if (v == 0x20) {
                busy_until = now_ms + (sim_bw ? ((last_ctrl & 0x08) ? 300u : 1500u) : (uint32_t)lut_frames() * 20u);
                display_update();
            }
        } else if (cur_cmd == 0x24 && n24 < EPD4G_MASK_BYTES) { p24[n24++] = v; if (sim_bw) bw_ram_write(0, v); }
        else if (cur_cmd == 0x26 && n26 < EPD4G_MASK_BYTES) { p26[n26++] = v; if (sim_bw) bw_ram_write(1, v); }
        else if (sim_bw && (cur_cmd == 0x44 || cur_cmd == 0x45 || cur_cmd == 0x4E || cur_cmd == 0x4F)) bw_reg_arg(cur_cmd, v);
        else if (cur_cmd == 0x32 && lutn < 400) lutb[lutn++] = v;
        else if (cur_cmd == 0x22) last_ctrl = v;
    }
    return 0;
}

static void clear_stream(void) { stream_n = 0; }

static uint32_t fnv(const uint8_t *p, int len) { uint32_t h = 2166136261u; while (len--) { h ^= *p++; h *= 16777619u; } return h; }

/* ------------------------------ helpers ---------------------------------- */
static uint32_t rng = 1;
static uint32_t rnd(void) { rng = rng * 1103515245u + 12345u; return rng >> 16; }

static void random_fb(uint8_t *fb) { for (int i = 0; i < EPD4G_FB_BYTES; i++) fb[i] = (uint8_t)rnd(); }

static void check_ink_equals(const uint8_t *fb, const char *what)
{
    for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++)
        if (dark[y * NAT_W + x] != dark_of_code(epd4g_fb_get(fb, x, y))) {
            CHECK(0, "%s: ink %d != target %d at %d,%d", what, dark[y * NAT_W + x], dark_of_code(epd4g_fb_get(fb, x, y)), x, y);
            return;
        }
}

/* ------------------------------- tests ----------------------------------- */
static void test_init_matches_stock(void)
{
    clear_stream(); rst_n = 0;
    CHECK(epd4g_init() == EPD4G_OK, "init failed");
    const int ng = (int)(sizeof golden_init / sizeof golden_init[0]);
    CHECK(stream_n == ng, "init stream length %d, stock %d", stream_n, ng);
    for (int i = 0; i < ng && i < stream_n; i++)
        if (stream[i] != golden_init[i]) { CHECK(0, "init stream differs from stock at byte %d: %03X vs %03X", i, stream[i], golden_init[i]); break; }
    CHECK(rst_n == 3 && rst_log[0] == 1 && rst_log[1] == 0 && rst_log[2] == 1, "reset pulse high-low-high");
}

static void test_full_deep_matches_stock(void)
{
    static uint8_t fb[EPD4G_FB_BYTES];
    rng = GOLDEN_IMAGE_SEED;
    for (int i = 0; i < EPD4G_FB_BYTES; i++) fb[i] = (uint8_t)rnd();   /* same generator as gen_golden.c */
    mode_partial = 0; clear_stream();
    CHECK(epd4g_full(fb, EPD4G_FULL_DEEP) == EPD4G_OK, "full deep failed");
    CHECK(n24 == GOLDEN_PLANE24_LEN && fnv(p24, n24) == GOLDEN_PLANE24_FNV, "RAM 0x24 differs from the stock 4-gray display (%08X)", fnv(p24, n24));
    CHECK(n26 == GOLDEN_PLANE26_LEN && fnv(p26, n26) == GOLDEN_PLANE26_FNV, "RAM 0x26 differs from the stock 4-gray display (%08X)", fnv(p26, n26));
    CHECK(lut_frames() == 262, "deep waveform is %d frames, stock has 262", lut_frames());
    /* the LUT sent is the stock one: compare with the 0x32 block of the stock init stream */
    int ng = (int)(sizeof golden_init / sizeof golden_init[0]), start = -1;
    for (int i = 0; i < ng; i++) if (golden_init[i] == 0x032) { start = i + 1; break; }
    for (int i = 0; start > 0 && i < 153; i++)
        if (lutb[i] != (golden_init[start + i] & 0xFF)) { CHECK(0, "deep LUT differs from stock at byte %d", i); break; }
    check_ink_equals(fb, "full deep");
}

static void test_full_fast(void)
{
    static uint8_t fb[EPD4G_FB_BYTES];
    random_fb(fb);
    mode_partial = 0;
    CHECK(epd4g_full(fb, EPD4G_FULL_FAST) == EPD4G_OK, "full fast failed");
    int expect = 46 + (2 * EPD4G_FULL_SHAKE_TP + 2) * (EPD4G_FULL_SHAKE_RP + 1) + 46 + 2;
    CHECK(lut_frames() == expect, "fast waveform %d frames, expected %d", lut_frames(), expect);
    check_ink_equals(fb, "full fast");
}

/* random partial updates: all 16 old->new gray transitions, masks of 1..100 % density */
static void test_partial_random(void)
{
    static uint8_t shown[EPD4G_FB_BYTES], next[EPD4G_FB_BYTES], mask[EPD4G_MASK_BYTES];
    random_fb(shown);
    mode_partial = 0;
    CHECK(epd4g_full(shown, EPD4G_FULL_DEEP) == EPD4G_OK, "full failed");
    for (int it = 0; it < 120; it++) {
        int dens = 1 + (int)(rnd() % 100);
        memcpy(next, shown, sizeof next);
        epd4g_mask_clear(mask);
        for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++)
            if ((int)(rnd() % 100) < dens) { epd4g_mask_set(mask, x, y); epd4g_fb_set(next, x, y, (uint8_t)(rnd() & 3)); }
        mode_partial = 1; pass_no = 0; expect_fb = next; clear_stream();
        CHECK(epd4g_partial(next, mask) == EPD4G_OK, "partial failed");
        mode_partial = 0;
        CHECK(pass_no == 0, "partial did not run two passes");
        check_ink_equals(next, "partial random");
        memcpy(shown, next, sizeof shown);
    }
    const int e1 = EPD4G_T_WHITE > EPD4G_T_BLACK ? EPD4G_T_WHITE : EPD4G_T_BLACK;
    CHECK(frames_pass[0] == e1, "pass 1 is %d frames, expected %d", frames_pass[0], e1);
    CHECK(frames_pass[1] == EPD4G_T_DARK, "pass 2 is %d frames, expected %d", frames_pass[1], EPD4G_T_DARK);
}

/* the demo pattern through epd4g_partial_diff(): only changed pixels are driven */
static void test_partial_diff(void)
{
    static uint8_t prev[EPD4G_FB_BYTES], cur[EPD4G_FB_BYTES];
    epd4g_fb_fill(prev, EPD4G_WHITE);
    for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++) epd4g_fb_set(prev, x, y, (uint8_t)(((x / 22) + (y / 22)) & 3));
    mode_partial = 0;
    CHECK(epd4g_full(prev, EPD4G_FULL_DEEP) == EPD4G_OK, "full failed");
    for (int step = 0; step < 30; step++) {
        memcpy(cur, prev, sizeof cur);
        int cx = 20 + (int)(rnd() % 130), cy = 20 + (int)(rnd() % 220);
        epd4g_rect_t roi = { cx - 10, cy - 10, cx + 10, cy + 10 };
        for (int y = roi.y0; y <= roi.y1; y++) for (int x = roi.x0; x <= roi.x1; x++)
            if ((x + y + step) % 3 == 0) epd4g_fb_set(cur, x, y, (uint8_t)(rnd() & 3));
        uint32_t changed = 0, brute = 0;
        for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++) brute += epd4g_fb_get(cur, x, y) != epd4g_fb_get(prev, x, y);
        mode_partial = 1; pass_no = 0; expect_fb = cur;
        int rc = epd4g_partial_diff(prev, cur, &roi, &changed);
        mode_partial = 0;
        CHECK(rc == EPD4G_OK, "partial_diff rc %d", rc);
        CHECK(changed == brute, "changed %u, brute force %u", changed, brute);
        if (changed) { int drv = 0; for (int i = 0; i < NAT_W * NAT_H; i++) drv += driven[i]; CHECK((uint32_t)drv == changed, "driven %d != changed %u", drv, changed); }
        check_ink_equals(cur, "partial_diff");
        memcpy(prev, cur, sizeof prev);
    }
    /* nothing changed -> panel is not touched */
    int before = updates_seen; uint32_t ch = 99;
    CHECK(epd4g_partial_diff(prev, prev, NULL, &ch) == EPD4G_OK && ch == 0 && updates_seen == before, "no-change update must not touch the panel");
    /* partial_rect */
    memcpy(cur, prev, sizeof cur);
    epd4g_rect_t r = { 40, 100, 60, 120 };
    for (int y = r.y0; y <= r.y1; y++) for (int x = r.x0; x <= r.x1; x++) epd4g_fb_set(cur, x, y, (uint8_t)(rnd() & 3));
    mode_partial = 1; pass_no = 0; expect_fb = cur;
    CHECK(epd4g_partial_rect(cur, &r) == EPD4G_OK, "partial_rect");
    mode_partial = 0;
    check_ink_equals(cur, "partial_rect");
}

static void test_power_and_sleep(void)
{
    static uint8_t fb[EPD4G_FB_BYTES];
    epd4g_fb_fill(fb, EPD4G_WHITE);
    CHECK(epd4g_init() == EPD4G_OK, "init");
    ctrl_c4 = ctrl_c7 = ctrl_other = 0; mode_partial = 0;
    CHECK(epd4g_full(fb, EPD4G_FULL_FAST) == EPD4G_OK, "full");
#if EPD4G_KEEP_POWER
    CHECK(ctrl_c4 == 1 && ctrl_c7 == 0, "KEEP_POWER=1 must use 0xC4 (c4=%d c7=%d)", ctrl_c4, ctrl_c7);
    clear_stream();
    CHECK(epd4g_power_off() == EPD4G_OK, "power_off");
    CHECK(stream_n == 3 && stream[0] == 0x022 && stream[1] == 0x183 && stream[2] == 0x020, "power-off stream");
    CHECK(epd4g_init() == EPD4G_OK, "re-init");
    clear_stream();
    CHECK(epd4g_sleep() == EPD4G_OK, "sleep");
    CHECK(stream_n == 5 && stream[3] == golden_sleep[0] && stream[4] == golden_sleep[1], "sleep tail differs from the stock Sleep()");
#else
    CHECK(ctrl_c7 == 1 && ctrl_c4 == 0, "KEEP_POWER=0 must use 0xC7 (c4=%d c7=%d)", ctrl_c4, ctrl_c7);
    CHECK(epd4g_sleep() == EPD4G_OK, "sleep");
#endif
    CHECK(epd4g_full(fb, EPD4G_FULL_FAST) == EPD4G_ERR_NOT_INIT, "update after sleep must report NOT_INIT");
    (void)ctrl_other;
}

static void test_errors(void)
{
    static uint8_t fb[EPD4G_FB_BYTES], mask[EPD4G_MASK_BYTES];
    epd4g_fb_fill(fb, EPD4G_WHITE);
    CHECK(epd4g_init() == EPD4G_OK, "init");
    CHECK(epd4g_full(NULL, EPD4G_FULL_FAST) == EPD4G_ERR_PARAM, "NULL fb");
    CHECK(epd4g_partial(fb, NULL) == EPD4G_ERR_PARAM, "NULL mask");
    epd4g_rect_t empty = { 10, 10, 5, 5 };
    CHECK(epd4g_partial_rect(fb, &empty) == EPD4G_ERR_PARAM, "empty rect");

    /* stuck BUSY: timeout within the configured time, then NOT_INIT until re-init */
    uint32_t t0 = now_ms; mode_partial = 0;
    busy_stuck = 1;
    CHECK(epd4g_full(fb, EPD4G_FULL_FAST) == EPD4G_ERR_TIMEOUT, "stuck BUSY must time out");
    CHECK(now_ms - t0 >= EPD4G_BUSY_TIMEOUT_MS && now_ms - t0 < EPD4G_BUSY_TIMEOUT_MS + 100, "timeout took %u ms", now_ms - t0);
    busy_stuck = 0;
    CHECK(epd4g_partial(fb, mask) == EPD4G_ERR_NOT_INIT, "after a timeout re-init is required");
    CHECK(epd4g_init() == EPD4G_OK, "re-init after timeout");
    busy_stuck = 1;
    CHECK(epd4g_init() == EPD4G_ERR_TIMEOUT, "init with a missing panel must time out");
    busy_stuck = 0;

    /* SPI failure */
    CHECK(epd4g_init() == EPD4G_OK, "init");
    spi_fail = 1;
    CHECK(epd4g_full(fb, EPD4G_FULL_FAST) == EPD4G_ERR_IO, "SPI error must be reported");
    spi_fail = 0;
}

static void test_fb_helpers(void)
{
    static uint8_t a[EPD4G_FB_BYTES], b[EPD4G_FB_BYTES], mask[EPD4G_MASK_BYTES];

    epd4g_fb_fill(a, EPD4G_DARK);
    CHECK(epd4g_fb_get(a, 0, 0) == EPD4G_DARK && epd4g_fb_get(a, 175, 263) == EPD4G_DARK, "fill");
    epd4g_fb_set(a, 5, 7, EPD4G_BLACK); epd4g_fb_set(a, 6, 7, EPD4G_LIGHT); epd4g_fb_set(a, 7, 7, EPD4G_WHITE);
    CHECK(epd4g_fb_get(a, 4, 7) == EPD4G_DARK && epd4g_fb_get(a, 5, 7) == EPD4G_BLACK && epd4g_fb_get(a, 6, 7) == EPD4G_LIGHT && epd4g_fb_get(a, 7, 7) == EPD4G_WHITE && epd4g_fb_get(a, 8, 7) == EPD4G_DARK, "set/get neighbours");
    epd4g_fb_set(a, -1, 0, 0); epd4g_fb_set(a, 176, 0, 0); epd4g_fb_set(a, 0, 264, 0);      /* ignored */
    CHECK(epd4g_fb_get(a, 0, 0) == EPD4G_DARK, "out of range writes ignored");

    for (int rot = 0; rot < 4; rot++) {                                  /* every logical pixel -> unique native pixel */
        int lw = epd4g_logical_width((epd4g_rot_t)rot), lh = epd4g_logical_height((epd4g_rot_t)rot);
        static uint8_t seen[NAT_W * NAT_H]; memset(seen, 0, sizeof seen);
        for (int y = 0; y < lh; y++) for (int x = 0; x < lw; x++) {
            int nx, ny;
            CHECK(epd4g_map((epd4g_rot_t)rot, x, y, &nx, &ny) == 0 && nx >= 0 && nx < NAT_W && ny >= 0 && ny < NAT_H, "map in range");
            seen[ny * NAT_W + nx]++;
        }
        int bad = 0; for (int i = 0; i < NAT_W * NAT_H; i++) bad += seen[i] != 1;
        CHECK(lw * lh == NAT_W * NAT_H && bad == 0, "rotation %d is not a bijection", rot);
        int nx, ny; CHECK(epd4g_map((epd4g_rot_t)rot, lw, 0, &nx, &ny) == -1 && epd4g_map((epd4g_rot_t)rot, 0, -1, &nx, &ny) == -1, "map rejects outside");
    }
    int nx, ny;                                                          /* the reference project's landscape mapping */
    epd4g_map(EPD4G_ROT_90, 60, 50, &nx, &ny);  CHECK(nx == 176 - 1 - 50 && ny == 60, "ROT_90 mapping");
    epd4g_map(EPD4G_ROT_270, 60, 50, &nx, &ny); CHECK(nx == 50 && ny == 264 - 1 - 60, "ROT_270 mapping");
    epd4g_fb_fill(a, EPD4G_WHITE); epd4g_fb_set_rot(a, EPD4G_ROT_90, 10, 20, EPD4G_BLACK);
    CHECK(epd4g_fb_get_rot(a, EPD4G_ROT_90, 10, 20) == EPD4G_BLACK && epd4g_fb_get_rot(a, EPD4G_ROT_90, 11, 20) == EPD4G_WHITE, "rot set/get");

    epd4g_rect_t r;
    CHECK(epd4g_map_rect(EPD4G_ROT_90, 50, 40, 80, 70, &r) == 0 && r.x0 == 176 - 1 - 70 && r.x1 == 176 - 1 - 40 && r.y0 == 50 && r.y1 == 80, "map_rect");
    CHECK(epd4g_map_rect(EPD4G_ROT_90, 300, 0, 310, 5, &r) == -1, "map_rect outside");

    random_fb(a); memcpy(b, a, sizeof b);                                /* mask_from_diff vs brute force */
    epd4g_rect_t roi = { 13, 30, 101, 200 };
    for (int i = 0; i < 700; i++) epd4g_fb_set(b, (int)(rnd() % NAT_W), (int)(rnd() % NAT_H), (uint8_t)(rnd() & 3));
    uint32_t n = epd4g_mask_from_diff(mask, a, b, &roi), brute = 0, bits = 0;
    for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++) {
        int in = x >= roi.x0 && x <= roi.x1 && y >= roi.y0 && y <= roi.y1;
        int diff = epd4g_fb_get(a, x, y) != epd4g_fb_get(b, x, y);
        int m = (mask[y * EPD4G_MASK_STRIDE + (x >> 3)] >> (7 - (x & 7))) & 1;
        if (in && diff) brute++;
        bits += m;
        if (m != (in && diff)) { CHECK(0, "mask mismatch at %d,%d", x, y); break; }
    }
    CHECK(n == brute && bits == brute, "mask_from_diff count %u / %u / %u", n, bits, brute);

    epd4g_fb_fill(a, EPD4G_WHITE); epd4g_fb_fill(b, EPD4G_BLACK);        /* copy_rect copies whole byte columns */
    epd4g_rect_t cr = { 9, 5, 14, 6 };
    epd4g_fb_copy_rect(a, b, &cr);
    CHECK(epd4g_fb_get(a, 9, 5) == EPD4G_BLACK && epd4g_fb_get(a, 14, 6) == EPD4G_BLACK && epd4g_fb_get(a, 9, 7) == EPD4G_WHITE && epd4g_fb_get(a, 7, 5) == EPD4G_WHITE, "copy_rect");
    epd4g_mask_clear(mask); epd4g_mask_set_rect(mask, &cr);
    uint32_t cnt = 0; for (int i = 0; i < EPD4G_MASK_BYTES; i++) for (int k = 0; k < 8; k++) cnt += (mask[i] >> k) & 1;
    CHECK(cnt == 12, "mask_set_rect set %u bits", cnt);
}


/* ============================ 1-bit mode ================================== */
static void bw_random(uint8_t *img) { for (int i = 0; i < EPD4G_BW_FB_BYTES; i++) img[i] = (uint8_t)rnd(); }

__attribute__((unused)) static uint32_t stream_fnv(int from)
{
    uint32_t h = 2166136261u;
    for (int i = from; i < stream_n; i++) { h ^= stream[i] & 0xFF; h *= 16777619u; h ^= stream[i] >> 8; h *= 16777619u; }
    return h;
}

static void check_bw_shows(const uint8_t *img, const char *what)
{
    for (int i = 0; i < EPD4G_BW_FB_BYTES; i++)
        if (bwshow[i] != img[i]) { CHECK(0, "%s: panel shows %02X, expected %02X at byte %d (x=%d y=%d)", what, bwshow[i], img[i], i, (i % 22) * 8, i / 22); return; }
}

static void test_bw_init_and_full_match_stock(void)
{
    static uint8_t img[EPD4G_BW_FB_BYTES];
    sim_bw = 1;
    epd4g_power_off();                       /* a previous 4-gray test may have left the analog part on */
    rng = GOLDEN_BW_SEED;
    for (int i = 0; i < EPD4G_BW_FB_BYTES; i++) img[i] = (uint8_t)rnd();     /* same generator as gen_golden.c */

    for (int fast = 0; fast < 2; fast++) {
        const uint16_t *g = fast ? golden_bw_init_fast : golden_bw_init;
        const int ng = fast ? (int)(sizeof golden_bw_init_fast / sizeof golden_bw_init_fast[0]) : (int)(sizeof golden_bw_init / sizeof golden_bw_init[0]);
        epd4g_power_off();                   /* the library would do it itself; keep the stream comparable */
        clear_stream(); rst_n = 0;
        CHECK(epd4g_bw_init(fast ? EPD4G_BW_FAST : EPD4G_BW_NORMAL) == EPD4G_OK, "bw init");
        CHECK(stream_n == ng, "bw init (fast=%d) stream length %d, stock %d", fast, stream_n, ng);
        for (int i = 0; i < ng && i < stream_n; i++)
            if (stream[i] != g[i]) { CHECK(0, "bw init (fast=%d) differs from stock at %d: %03X vs %03X", fast, i, stream[i], g[i]); break; }
        CHECK(rst_n == 3 && rst_log[0] == 1 && rst_log[1] == 0 && rst_log[2] == 1, "bw reset pulse");

        clear_stream(); bw_full_updates = 0;
        CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, fast ? EPD4G_BW_FAST : EPD4G_BW_NORMAL) == EPD4G_OK, "bw full");
        CHECK(n24 == GOLDEN_BWBASE_LEN && n26 == GOLDEN_BWBASE_LEN, "bw plane sizes %d %d", n24, n26);
        CHECK(fnv(p24, n24) == (fast ? GOLDEN_BWFAST_P24_FNV : GOLDEN_BWBASE_P24_FNV), "bw RAM 0x24 differs from stock (fast=%d)", fast);
        CHECK(fnv(p26, n26) == GOLDEN_BWBASE_P26_FNV, "bw RAM 0x26 differs from stock Display_Base");
#if !EPD4G_BW_KEEP_POWER
        CHECK(last_ctrl == (fast ? 0xC7 : 0xF7), "bw full control byte %02X", last_ctrl);
#endif
        CHECK(bw_full_updates == 1, "one full update");
        check_bw_shows(img, "bw full");
    }

    CHECK(epd4g_bw_init(EPD4G_BW_NORMAL) == EPD4G_OK, "bw init");
    CHECK(epd4g_bw_clear(EPD4G_BW_NORMAL) == EPD4G_OK, "bw clear");
    CHECK(fnv(p24, n24) == GOLDEN_BWCLEAR_P24_FNV, "bw clear plane differs from stock Clear");
    for (int i = 0; i < EPD4G_BW_FB_BYTES; i++) if (bwshow[i] != 0xFF) { CHECK(0, "clear is not white"); break; }

    /* 4-gray frame as the source: converted while sending */
    static uint8_t g4[EPD4G_FB_BYTES], ref[EPD4G_BW_FB_BYTES], ref2[EPD4G_BW_FB_BYTES];
    random_fb(g4);
    epd4g_fb_to_bw(ref, g4);
    for (int y = 0; y < NAT_H; y++) for (int x = 0; x < NAT_W; x++)
        epd4g_bw_fb_set(ref2, x, y, epd4g_fb_get(g4, x, y) >= EPD4G_BW_THRESHOLD);
    CHECK(memcmp(ref, ref2, sizeof ref) == 0, "fb_to_bw vs per-pixel threshold");
    CHECK(epd4g_bw_full(g4, EPD4G_SRC_2BPP, EPD4G_BW_NORMAL) == EPD4G_OK, "bw full from 2bpp");
    check_bw_shows(ref, "bw full 2bpp");
    sim_bw = 0;
}

static void test_bw_partial(void)
{
    static uint8_t shown[EPD4G_BW_FB_BYTES], img[EPD4G_BW_FB_BYTES], g4[EPD4G_FB_BYTES], conv[EPD4G_BW_FB_BYTES], want[EPD4G_BW_FB_BYTES];
    sim_bw = 1;
    CHECK(epd4g_bw_init(EPD4G_BW_NORMAL) == EPD4G_OK, "bw init");
    bw_random(shown);
    CHECK(epd4g_bw_full(shown, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_OK, "bw full");

    /* the stock Display_Partial stream, byte for byte (window x 16..87, y 40..99) */
    rng = GOLDEN_BW_SEED;
    for (int i = 0; i < EPD4G_BW_FB_BYTES; i++) img[i] = (uint8_t)rnd();
    epd4g_rect_t r0 = { 16, 40, 87, 99 };
    clear_stream(); rst_n = 0; bw_part_updates = 0;
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r0) == EPD4G_OK, "bw partial");
#if EPD4G_BW_PARTIAL_RESET && !EPD4G_BW_KEEP_POWER
    CHECK(stream_n == GOLDEN_BWPART_LEN && stream_fnv(0) == GOLDEN_BWPART_FNV, "partial stream differs from stock Display_Partial (len %d, fnv %08X)", stream_n, stream_fnv(0));
    CHECK(rst_n == 3 && rst_log[0] == 1 && rst_log[1] == 0 && rst_log[2] == 1, "partial starts with a hardware reset like the stock one");
#endif
    CHECK(bw_part_updates == 1, "one partial update");
    memcpy(want, shown, sizeof want);
    for (int y = 40; y <= 99; y++) memcpy(&want[y * 22 + 2], &img[y * 22 + 2], 9);
    check_bw_shows(want, "bw partial window");
    memcpy(shown, want, sizeof shown);

    /* random windows (unaligned x), both source formats. Only the window changes, the rest of
     * the passed image is ignored, so it may differ from what is on the panel. */
    for (int it = 0; it < 60; it++) {
        epd4g_rect_t r;
        r.x0 = (int)(rnd() % 176); r.x1 = r.x0 + (int)(rnd() % (176 - r.x0));
        r.y0 = (int)(rnd() % 264); r.y1 = r.y0 + (int)(rnd() % (264 - r.y0));
        int use2 = it & 1;
        if (use2) { random_fb(g4); epd4g_fb_to_bw(conv, g4); memcpy(img, conv, sizeof img); }
        else bw_random(img);
        CHECK(epd4g_bw_partial(use2 ? (const void *)g4 : (const void *)img, use2 ? EPD4G_SRC_2BPP : EPD4G_SRC_1BPP, &r) == EPD4G_OK, "bw partial random");
        memcpy(want, shown, sizeof want);
        for (int y = r.y0; y <= r.y1; y++) memcpy(&want[y * 22 + (r.x0 >> 3)], &img[y * 22 + (r.x0 >> 3)], (size_t)((r.x1 >> 3) - (r.x0 >> 3) + 1));
        check_bw_shows(want, "bw partial random");
        memcpy(shown, want, sizeof shown);
    }

    /* whole-screen partial (rect == NULL), then a full refresh after partials restores the window */
    bw_random(img);
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, NULL) == EPD4G_OK, "bw partial whole screen");
    check_bw_shows(img, "bw partial whole screen");
    epd4g_rect_t small = { 8, 8, 20, 20 };
    CHECK(epd4g_bw_partial(shown, EPD4G_SRC_1BPP, &small) == EPD4G_OK, "partial");
    bw_random(img);
    int fulls = bw_full_updates;
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_OK, "full after partials");
    CHECK(bw_full_updates == fulls + 1, "full update ran");
    check_bw_shows(img, "bw full after partials (window registers restored)");

    /* white source and the status codes */
    CHECK(epd4g_bw_partial(NULL, EPD4G_SRC_WHITE, &small) == EPD4G_OK, "white partial");
    CHECK(epd4g_bw_partial(NULL, EPD4G_SRC_1BPP, &small) == EPD4G_ERR_PARAM, "NULL image");
    epd4g_rect_t empty = { 10, 10, 5, 5 }, outside = { 200, 0, 220, 10 };
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &empty) == EPD4G_ERR_PARAM, "empty rect");
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &outside) == EPD4G_ERR_PARAM, "rect outside the panel");
    sim_bw = 0;
}

static void test_bw_modes_and_errors(void)
{
    static uint8_t img[EPD4G_BW_FB_BYTES], g4[EPD4G_FB_BYTES], mask[EPD4G_MASK_BYTES];
    epd4g_rect_t r = { 0, 0, 20, 20 };
    sim_bw = 1; epd4g_fb_fill(g4, EPD4G_WHITE); bw_random(img); epd4g_mask_clear(mask);

    CHECK(epd4g_bw_init(EPD4G_BW_FAST) == EPD4G_OK, "bw init fast");
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_ERR_NO_BASE, "partial without a base picture");
    CHECK(epd4g_full(g4, EPD4G_FULL_FAST) == EPD4G_ERR_MODE, "4-gray full in 1-bit mode");
    CHECK(epd4g_partial(g4, mask) == EPD4G_ERR_MODE, "4-gray partial in 1-bit mode");
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_FAST) == EPD4G_OK, "fast full after fast init");
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_OK, "partial");
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_FAST) == EPD4G_ERR_MODE, "fast full after a partial needs init_fast again");
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_OK, "normal full after a partial");

    sim_bw = 0; clear_stream();
    CHECK(epd4g_init() == EPD4G_OK, "switch to 4 gray");
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_ERR_MODE, "bw full in 4-gray mode");
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_ERR_MODE, "bw partial in 4-gray mode");
    CHECK(epd4g_full(g4, EPD4G_FULL_FAST) == EPD4G_OK, "4-gray full after the switch");

    /* switching away while the analog part is left powered: the library powers it down first */
    sim_bw = 1; clear_stream();
    CHECK(epd4g_bw_init(EPD4G_BW_NORMAL) == EPD4G_OK, "back to 1 bit");
#if EPD4G_KEEP_POWER
    CHECK(stream_n >= 3 && stream[0] == 0x022 && stream[1] == 0x183 && stream[2] == 0x020, "power-off before the reset");
#endif
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_ERR_NO_BASE, "the 1-bit base is gone after a mode switch");

    /* sleep / wake from the 1-bit mode */
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_OK, "full");
    clear_stream();
    CHECK(epd4g_sleep() == EPD4G_OK, "bw sleep");
    const int ns = (int)(sizeof golden_sleep / sizeof golden_sleep[0]);
    CHECK(stream_n >= ns && stream[stream_n - 2] == golden_sleep[0] && stream[stream_n - 1] == golden_sleep[1], "bw sleep tail differs from the stock Sleep()");
#if !EPD4G_BW_KEEP_POWER
    CHECK(stream_n == ns, "stock Sleep() is the whole sequence (the panel powers itself down after an update)");
#endif
    CHECK(epd4g_sleep() == EPD4G_OK && epd4g_power_off() == EPD4G_OK, "sleeping twice is harmless");
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_ERR_NOT_INIT, "full refresh after sleep needs init");
#if EPD4G_BW_PARTIAL_RESET
    /* stock flow Display_Base, Sleep, Display_Partial: the RAM planes survive and the reset wakes the panel */
    static uint8_t w2[EPD4G_BW_FB_BYTES];
    memcpy(w2, img, sizeof w2);
    for (int y = r.y0; y <= r.y1; y++) memset(&w2[y * 22], (y & 1) ? 0x00 : 0xA5, 3);
    clear_stream(); rst_n = 0;
    CHECK(epd4g_bw_partial(w2, EPD4G_SRC_1BPP, &r) == EPD4G_OK, "partial after sleep");
    CHECK(rst_n == 3, "partial after sleep starts with the reset pulse");
    for (int y = r.y0; y <= r.y1; y++) memcpy(&img[y * 22], &w2[y * 22], 3);
    check_bw_shows(img, "partial after sleep");
    CHECK(epd4g_sleep() == EPD4G_OK, "sleep again");
#else
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_ERR_NOT_INIT, "no reset in partial: nothing wakes the panel");
#endif
    CHECK(epd4g_bw_init(EPD4G_BW_NORMAL) == EPD4G_OK, "wake");

    /* BUSY timeout in the 1-bit mode */
    busy_stuck = 1;
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_ERR_TIMEOUT, "stuck BUSY");
    busy_stuck = 0;
    CHECK(epd4g_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_ERR_NOT_INIT, "after a timeout re-init is required");
    CHECK(epd4g_bw_init(EPD4G_BW_NORMAL) == EPD4G_OK, "re-init");
    spi_fail = 1;
    CHECK(epd4g_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_ERR_IO, "SPI error is reported");
    spi_fail = 0;
}

/* gray partial with black/white pixels only: pass 2 is skipped */
static void test_gray_skip_pass2(void)
{
    static uint8_t shown[EPD4G_FB_BYTES], next[EPD4G_FB_BYTES], mask[EPD4G_MASK_BYTES];
    sim_bw = 0; random_fb(shown);
    CHECK(epd4g_init() == EPD4G_OK, "init");
    mode_partial = 0;
    CHECK(epd4g_full(shown, EPD4G_FULL_DEEP) == EPD4G_OK, "full");
    memcpy(next, shown, sizeof next); epd4g_mask_clear(mask);
    for (int y = 30; y < 60; y++) for (int x = 20; x < 90; x++) { epd4g_mask_set(mask, x, y); epd4g_fb_set(next, x, y, ((x + y) & 1) ? EPD4G_BLACK : EPD4G_WHITE); }
    mode_partial = 1; pass_no = 0; expect_fb = next;
    int before = updates_seen;
    CHECK(epd4g_partial(next, mask) == EPD4G_OK, "partial");
    mode_partial = 0;
    CHECK(updates_seen == before + 1, "black/white only: %d panel updates, expected 1", updates_seen - before);
    check_ink_equals(next, "partial without gray");
    pass_no = 0;
    /* one gray pixel brings pass 2 back */
    epd4g_mask_set(mask, 100, 100); epd4g_fb_set(next, 100, 100, EPD4G_LIGHT);
    mode_partial = 1; pass_no = 0; expect_fb = next; before = updates_seen;
    CHECK(epd4g_partial(next, mask) == EPD4G_OK, "partial");
    mode_partial = 0;
    CHECK(updates_seen == before + 2, "with a gray pixel: %d panel updates, expected 2", updates_seen - before);
    check_ink_equals(next, "partial with gray");
}

/* ======================= non-blocking interface =========================== */
static int cb_calls, cb_rc;
static void done_cb(int rc, void *ctx) { cb_calls++; cb_rc = rc; (void)ctx; }

/* run a job with poll(): the "main loop" does other work and time passes by itself. Returns the
 * largest number of SPI bytes sent by one poll call. */
static long run_async(long *loops, int *max_pending_ms)
{
    long worst = 0, n = 0;
    uint32_t t_start = now_ms;
    while (epd4g_is_busy()) {
        long b0 = spi_bytes;
        uint32_t t0 = now_ms;
        int pending = epd4g_poll();
        CHECK(now_ms == t0, "poll() must not sleep or advance time itself");
        if (spi_bytes - b0 > worst) worst = spi_bytes - b0;
        CHECK(pending == 0 || pending == EPD4G_PENDING, "poll status");
        n++;                                    /* the rest of the main loop: GNSS, buttons, ... */
        now_ms += 1;                            /* 1 ms per main loop pass */
        CHECK(now_ms - t_start < 60000, "job does not finish");
        if (now_ms - t_start >= 60000) break;
    }
    if (loops) *loops = n;
    if (max_pending_ms) *max_pending_ms = (int)(now_ms - t_start);
    return worst;
}

static void test_async(void)
{
    static uint8_t fb[EPD4G_FB_BYTES], fb2[EPD4G_FB_BYTES], mask[EPD4G_MASK_BYTES], img[EPD4G_BW_FB_BYTES];
    long loops, worst; int ms;
    const long slice_limit = (long)EPD4G_ROWS_PER_POLL * EPD4G_FB_STRIDE + 400;   /* rows + a few register writes */

    sim_bw = 0; busy_until = 0;
    random_fb(fb);
    CHECK(epd4g_start_init() == EPD4G_OK, "start init");
    CHECK(epd4g_is_busy(), "busy after start");
    CHECK(epd4g_start_init() == EPD4G_ERR_BUSY, "second job while running");
    CHECK(epd4g_start_full(fb, EPD4G_FULL_FAST) == EPD4G_ERR_BUSY, "full while init is running");
    CHECK(epd4g_start_sleep() == EPD4G_ERR_BUSY, "sleep while running");
    worst = run_async(&loops, &ms);
    CHECK(!epd4g_is_busy() && epd4g_result() == EPD4G_OK, "init result %d", epd4g_result());
    CHECK(worst <= slice_limit, "init: %ld bytes in one poll", worst);

    /* a full refresh: bounded work per poll, the main loop keeps running during the waveform */
    epd4g_set_done_cb(done_cb, NULL); cb_calls = 0;
    mode_partial = 0;
    CHECK(epd4g_start_full(fb, EPD4G_FULL_FAST) == EPD4G_OK, "start full");
    worst = run_async(&loops, &ms);
    CHECK(epd4g_result() == EPD4G_OK && cb_calls == 1 && cb_rc == EPD4G_OK, "full result / callback");
    CHECK(worst <= slice_limit, "full: %ld bytes in one poll (limit %ld)", worst, slice_limit);
    CHECK(ms >= 100 * 20 && loops > 100, "full: %d ms, %ld loop passes: main loop starved?", ms, loops);
    check_ink_equals(fb, "async full");
    epd4g_set_done_cb(NULL, NULL);

    /* the frame buffer is free again as soon as its data is sent, long before the job ends */
    {
        static uint8_t keep[EPD4G_FB_BYTES];
        memcpy(keep, fb, sizeof keep);
        CHECK(!epd4g_data_busy(), "no data in use when idle");
        mode_partial = 0;
        CHECK(epd4g_start_full(fb, EPD4G_FULL_FAST) == EPD4G_OK && epd4g_data_busy(), "data in use after start");
        long released_at = -1, polls = 0;
        while (epd4g_is_busy()) {
            epd4g_poll(); polls++; now_ms++;
            if (released_at < 0 && !epd4g_data_busy()) { released_at = polls; epd4g_fb_fill(fb, EPD4G_BLACK); }   /* next frame is drawn meanwhile */
            if (polls > 100000) break;
        }
        CHECK(released_at > 0 && released_at < polls / 4, "data released at poll %ld of %ld", released_at, polls);
        CHECK(epd4g_result() == EPD4G_OK, "full result");
        check_ink_equals(keep, "full with the buffer overwritten after release");
        memcpy(fb, keep, sizeof fb);
    }

    /* partial through the diff helper: same picture as the blocking call, no panel access if equal */
    memcpy(fb2, fb, sizeof fb2);
    for (int y = 50; y < 80; y++) for (int x = 40; x < 100; x++) epd4g_fb_set(fb2, x, y, (uint8_t)((x * 3 + y) & 3));
    uint32_t changed = 0;
    mode_partial = 1; pass_no = 0; expect_fb = fb2;
    clear_stream();
    CHECK(epd4g_start_partial_diff(fb, fb2, NULL, &changed) == EPD4G_OK && changed > 0, "start partial diff");
    CHECK(epd4g_start_partial_diff(fb, fb2, NULL, &changed) == EPD4G_ERR_BUSY, "diff while the mask is in use");
    uint32_t t_mid = now_ms;
    worst = run_async(&loops, &ms);
    mode_partial = 0;
    CHECK(epd4g_result() == EPD4G_OK, "partial result");
    CHECK(worst <= slice_limit, "partial: %ld bytes in one poll", worst);
    CHECK(now_ms - t_mid >= 40 * 20 && loops > 40, "partial waveform wait: %u ms", now_ms - t_mid);
    check_ink_equals(fb2, "async partial");
    CHECK(!epd4g_data_busy(), "data free after the job");
    CHECK(epd4g_start_partial_diff(fb2, fb2, NULL, &changed) == EPD4G_OK && changed == 0 && !epd4g_is_busy(), "no change: no job");

    /* the blocking twin sends exactly the same bytes */
    {
        static uint16_t a[8192]; int na;
        memcpy(fb, fb2, sizeof fb);
        for (int y = 120; y < 140; y++) for (int x = 10; x < 60; x++) epd4g_fb_set(fb, x, y, (uint8_t)((x + y * 5) & 3));
        mode_partial = 1; pass_no = 0; expect_fb = fb; clear_stream();
        CHECK(epd4g_start_partial_diff(fb2, fb, NULL, &changed) == EPD4G_OK, "start");
        run_async(NULL, NULL);
        na = stream_n < 8192 ? stream_n : 8192; memcpy(a, stream, (size_t)na * sizeof a[0]);
        /* panel state is now "fb"; redo the same update (fb2 -> fb) from the old picture in blocking mode */
        mode_partial = 0;
        CHECK(epd4g_full(fb2, EPD4G_FULL_FAST) == EPD4G_OK, "restore");
        mode_partial = 1; pass_no = 0; expect_fb = fb; clear_stream();
        CHECK(epd4g_partial_diff(fb2, fb, NULL, &changed) == EPD4G_OK, "blocking diff");
        CHECK(stream_n == na && memcmp(stream, a, (size_t)na * sizeof a[0]) == 0, "blocking and non-blocking streams differ");
        mode_partial = 0;
    }

    /* 1-bit jobs: chained from the callback, as a UI state machine would */
    sim_bw = 1; bw_random(img);
    CHECK(epd4g_start_bw_init(EPD4G_BW_NORMAL) == EPD4G_OK, "start bw init");
    CHECK(epd4g_start_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_ERR_BUSY, "bw full while init");
    worst = run_async(&loops, &ms);
    CHECK(epd4g_result() == EPD4G_OK, "bw init result");
    CHECK(epd4g_start_bw_full(img, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL) == EPD4G_OK, "start bw full");
    worst = run_async(&loops, &ms);
    CHECK(worst <= (long)EPD4G_ROWS_PER_POLL * EPD4G_BW_STRIDE + 400, "bw full: %ld bytes in one poll", worst);
    check_bw_shows(img, "async bw full");
    epd4g_rect_t r = { 30, 60, 90, 90 };
    bw_random(img);
    uint32_t t1 = now_ms;
    CHECK(epd4g_start_bw_partial(img, EPD4G_SRC_1BPP, &r) == EPD4G_OK, "start bw partial");
    worst = run_async(&loops, &ms);
    CHECK(epd4g_result() == EPD4G_OK, "bw partial result");
    CHECK(now_ms - t1 >= 300 && loops > 200, "bw partial: main loop ran %ld passes in %u ms", loops, now_ms - t1);
    sim_bw = 0;

    /* abort drops the job and requires a new init */
    CHECK(epd4g_start_init() == EPD4G_OK, "start");
    epd4g_abort();
    CHECK(!epd4g_is_busy(), "abort");
    CHECK(epd4g_start_full(fb, EPD4G_FULL_FAST) == EPD4G_ERR_NOT_INIT, "after abort");

    /* timeout in the non-blocking interface */
    busy_stuck = 0; busy_until = 0;
    CHECK(epd4g_start_init() == EPD4G_OK, "start");
    run_async(NULL, NULL);
    busy_stuck = 1;
    CHECK(epd4g_start_full(fb, EPD4G_FULL_FAST) == EPD4G_OK, "start");
    uint32_t t2 = now_ms;
    run_async(NULL, NULL);
    CHECK(epd4g_result() == EPD4G_ERR_TIMEOUT && now_ms - t2 >= EPD4G_BUSY_TIMEOUT_MS && now_ms - t2 < EPD4G_BUSY_TIMEOUT_MS + 1000, "timeout: rc %d after %u ms", epd4g_result(), now_ms - t2);
    busy_stuck = 0; busy_until = 0;
    (void)mask;
}

int main(void)
{
    test_init_matches_stock();
    test_full_deep_matches_stock();
    test_full_fast();
    test_partial_random();
    test_partial_diff();
    test_power_and_sleep();
    test_errors();
    test_fb_helpers();
    test_gray_skip_pass2();
    test_bw_init_and_full_match_stock();
    test_bw_partial();
    test_bw_modes_and_errors();
    test_async();
    printf("epd4g host tests: %d display updates simulated, %s (%d failure%s)\n",
           updates_seen, errors ? "FAILED" : "all passed", errors, errors == 1 ? "" : "s");
    return errors ? 1 : 0;
}
