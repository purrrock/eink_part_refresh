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
 * Model assumption (cannot be seen from the host, verified visually on hardware): RAM pair
 * (0x24,0x26) selects LUT row (0,0)=0 (0,1)=1 (1,0)=2 (1,1)=3.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "epd4g.h"
#include "epd4g_fb.h"
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

void epd4g_port_gpio_rst(int l) { if (rst_n < 8) rst_log[rst_n++] = l; }
void epd4g_port_gpio_dc(int l)  { dc = l; }
void epd4g_port_gpio_cs(int l)  { cs = l; }
int  epd4g_port_gpio_busy(void) { return busy_stuck; }
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

static void display_update(void)
{
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
        if (!dc) {
            cur_cmd = v;
            if (v == 0x24) n24 = 0;
            if (v == 0x26) n26 = 0;
            if (v == 0x32) lutn = 0;
            if (v == 0x20) display_update();
        } else if (cur_cmd == 0x24 && n24 < EPD4G_MASK_BYTES) p24[n24++] = v;
        else if (cur_cmd == 0x26 && n26 < EPD4G_MASK_BYTES) p26[n26++] = v;
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
    printf("epd4g host tests: %d display updates simulated, %s (%d failure%s)\n",
           updates_seen, errors ? "FAILED" : "all passed", errors, errors == 1 ? "" : "s");
    return errors ? 1 : 0;
}
