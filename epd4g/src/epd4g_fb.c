/**
 * @file epd4g_fb.c
 * @brief Frame buffer, rotation and update-mask helpers (optional part of epd4g).
 */
#include "epd4g_fb.h"
#include <string.h>

#if defined(__GNUC__) && !defined(__clang__) && !defined(__OPTIMIZE__) && !defined(EPD4G_NO_FORCE_O2)
#pragma GCC optimize ("O2")
#endif

/* ------------------------------ rotation --------------------------------- */
int epd4g_logical_width(epd4g_rot_t rot)
{
    return (rot == EPD4G_ROT_90 || rot == EPD4G_ROT_270) ? EPD4G_HEIGHT : EPD4G_WIDTH;
}

int epd4g_logical_height(epd4g_rot_t rot)
{
    return (rot == EPD4G_ROT_90 || rot == EPD4G_ROT_270) ? EPD4G_WIDTH : EPD4G_HEIGHT;
}

int epd4g_map(epd4g_rot_t rot, int x, int y, int *nx, int *ny)
{
    const int lw = epd4g_logical_width(rot), lh = epd4g_logical_height(rot);
    if (x < 0 || y < 0 || x >= lw || y >= lh)
        return -1;
    switch (rot) {
    case EPD4G_ROT_90:  *nx = lh - 1 - y; *ny = x;           break;
    case EPD4G_ROT_180: *nx = lw - 1 - x; *ny = lh - 1 - y;  break;
    case EPD4G_ROT_270: *nx = y;          *ny = lw - 1 - x;  break;
    default:            *nx = x;          *ny = y;           break;
    }
    return 0;
}

int epd4g_map_rect(epd4g_rot_t rot, int x0, int y0, int x1, int y1, epd4g_rect_t *out)
{
    int ax, ay, bx, by;
    const int lw = epd4g_logical_width(rot), lh = epd4g_logical_height(rot);

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= lw) x1 = lw - 1;
    if (y1 >= lh) y1 = lh - 1;
    if (x0 > x1 || y0 > y1 || out == NULL)
        return -1;
    epd4g_map(rot, x0, y0, &ax, &ay);
    epd4g_map(rot, x1, y1, &bx, &by);
    out->x0 = ax < bx ? ax : bx;  out->x1 = ax < bx ? bx : ax;
    out->y0 = ay < by ? ay : by;  out->y1 = ay < by ? by : ay;
    return 0;
}

/* ----------------------------- frame buffer ------------------------------ */
void epd4g_fb_fill(uint8_t *fb, uint8_t level)
{
    uint8_t v = (uint8_t)(level & 3);
    memset(fb, (v << 6) | (v << 4) | (v << 2) | v, EPD4G_FB_BYTES);
}

void epd4g_fb_set(uint8_t *fb, int x, int y, uint8_t level)
{
    if (x < 0 || y < 0 || x >= EPD4G_WIDTH || y >= EPD4G_HEIGHT)
        return;
    size_t i = (size_t)y * EPD4G_FB_STRIDE + (size_t)(x >> 2);
    int sh = 6 - 2 * (x & 3);
    fb[i] = (uint8_t)((fb[i] & ~(3u << sh)) | ((unsigned)(level & 3) << sh));
}

uint8_t epd4g_fb_get(const uint8_t *fb, int x, int y)
{
    if (x < 0 || y < 0 || x >= EPD4G_WIDTH || y >= EPD4G_HEIGHT)
        return EPD4G_WHITE;
    return (uint8_t)((fb[(size_t)y * EPD4G_FB_STRIDE + (size_t)(x >> 2)] >> (6 - 2 * (x & 3))) & 3);
}

void epd4g_fb_set_rot(uint8_t *fb, epd4g_rot_t rot, int x, int y, uint8_t level)
{
    int nx, ny;
    if (epd4g_map(rot, x, y, &nx, &ny) == 0)
        epd4g_fb_set(fb, nx, ny, level);
}

uint8_t epd4g_fb_get_rot(const uint8_t *fb, epd4g_rot_t rot, int x, int y)
{
    int nx, ny;
    if (epd4g_map(rot, x, y, &nx, &ny) != 0)
        return EPD4G_WHITE;
    return epd4g_fb_get(fb, nx, ny);
}

void epd4g_fb_copy_rect(uint8_t *dst, const uint8_t *src, const epd4g_rect_t *r)
{
    int x0 = r->x0 < 0 ? 0 : r->x0, x1 = r->x1 >= EPD4G_WIDTH ? EPD4G_WIDTH - 1 : r->x1;
    int y0 = r->y0 < 0 ? 0 : r->y0, y1 = r->y1 >= EPD4G_HEIGHT ? EPD4G_HEIGHT - 1 : r->y1;
    if (x0 > x1 || y0 > y1)
        return;
    int xb0 = x0 >> 2, xb1 = x1 >> 2;
    for (int y = y0; y <= y1; y++) {
        size_t o = (size_t)y * EPD4G_FB_STRIDE + (size_t)xb0;
        memcpy(&dst[o], &src[o], (size_t)(xb1 - xb0 + 1));
    }
}

/* -------------------------------- masks ---------------------------------- */
void epd4g_mask_clear(uint8_t *mask)
{
    memset(mask, 0, EPD4G_MASK_BYTES);
}

void epd4g_mask_set(uint8_t *mask, int x, int y)
{
    if (x < 0 || y < 0 || x >= EPD4G_WIDTH || y >= EPD4G_HEIGHT)
        return;
    mask[(size_t)y * EPD4G_MASK_STRIDE + (size_t)(x >> 3)] |= (uint8_t)(0x80u >> (x & 7));
}

void epd4g_mask_set_rect(uint8_t *mask, const epd4g_rect_t *r)
{
    int x0 = r->x0 < 0 ? 0 : r->x0, x1 = r->x1 >= EPD4G_WIDTH ? EPD4G_WIDTH - 1 : r->x1;
    int y0 = r->y0 < 0 ? 0 : r->y0, y1 = r->y1 >= EPD4G_HEIGHT ? EPD4G_HEIGHT - 1 : r->y1;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            epd4g_mask_set(mask, x, y);
}

uint32_t epd4g_mask_from_diff(uint8_t *mask, const uint8_t *prev, const uint8_t *cur,
                              const epd4g_rect_t *roi)
{
    int x0 = 0, y0 = 0, x1 = EPD4G_WIDTH - 1, y1 = EPD4G_HEIGHT - 1;
    uint32_t n = 0;

    epd4g_mask_clear(mask);
    if (roi != NULL) {
        if (roi->x0 > x0) x0 = roi->x0;
        if (roi->y0 > y0) y0 = roi->y0;
        if (roi->x1 < x1) x1 = roi->x1;
        if (roi->y1 < y1) y1 = roi->y1;
    }
    if (x0 > x1 || y0 > y1)
        return 0;

    for (int y = y0; y <= y1; y++) {
        const size_t row = (size_t)y * EPD4G_FB_STRIDE;
        for (int xb = x0 >> 2; xb <= (x1 >> 2); xb++) {
            uint8_t a = prev[row + (size_t)xb], b = cur[row + (size_t)xb];
            if (a == b)
                continue;                                  /* 4 pixels equal */
            for (int k = 0; k < 4; k++) {
                int x = xb * 4 + k;
                if (x < x0 || x > x1)
                    continue;
                int sh = 6 - 2 * k;
                if (((a ^ b) >> sh) & 3) {
                    mask[(size_t)y * EPD4G_MASK_STRIDE + (size_t)(x >> 3)] |= (uint8_t)(0x80u >> (x & 7));
                    n++;
                }
            }
        }
    }
    return n;
}

#if EPD4G_INTERNAL_MASK
/* One scratch mask shared by the partial_diff / partial_rect helpers. A running job reads it, so the
 * start functions refuse (EPD4G_ERR_BUSY) while a job is in progress. */
static uint8_t s_mask[EPD4G_MASK_BYTES];

int epd4g_start_partial_diff(const uint8_t *prev, const uint8_t *cur,
                             const epd4g_rect_t *roi, uint32_t *changed)
{
    uint32_t n;

    if (prev == NULL || cur == NULL)
        return EPD4G_ERR_PARAM;
    if (epd4g_is_busy())
        return EPD4G_ERR_BUSY;
    n = epd4g_mask_from_diff(s_mask, prev, cur, roi);
    if (changed != NULL)
        *changed = n;
    if (n == 0)
        return EPD4G_OK;
    return epd4g_start_partial(cur, s_mask);
}

int epd4g_start_partial_rect(const uint8_t *fb, const epd4g_rect_t *rect)
{
    if (fb == NULL || rect == NULL)
        return EPD4G_ERR_PARAM;
    if (rect->x0 > rect->x1 || rect->y0 > rect->y1 ||
        rect->x1 < 0 || rect->y1 < 0 || rect->x0 >= EPD4G_WIDTH || rect->y0 >= EPD4G_HEIGHT)
        return EPD4G_ERR_PARAM;
    if (epd4g_is_busy())
        return EPD4G_ERR_BUSY;
    epd4g_mask_clear(s_mask);
    epd4g_mask_set_rect(s_mask, rect);
    return epd4g_start_partial(fb, s_mask);
}

int epd4g_partial_diff(const uint8_t *prev, const uint8_t *cur,
                       const epd4g_rect_t *roi, uint32_t *changed)
{
    int rc = epd4g_start_partial_diff(prev, cur, roi, changed);
    if (rc != EPD4G_OK || !epd4g_is_busy())
        return rc;                                  /* error, or nothing changed */
    return epd4g_wait();
}

int epd4g_partial_rect(const uint8_t *fb, const epd4g_rect_t *rect)
{
    int rc = epd4g_start_partial_rect(fb, rect);
    return rc != EPD4G_OK ? rc : epd4g_wait();
}
#endif

/* ------------------------------ 1-bit frame ------------------------------- */
void epd4g_bw_fb_fill(uint8_t *fb, int white)
{
    memset(fb, white ? 0xFF : 0x00, EPD4G_BW_FB_BYTES);
}

void epd4g_bw_fb_set(uint8_t *fb, int x, int y, int white)
{
    if (x < 0 || y < 0 || x >= EPD4G_WIDTH || y >= EPD4G_HEIGHT)
        return;
    size_t i = (size_t)y * EPD4G_BW_STRIDE + (size_t)(x >> 3);
    uint8_t m = (uint8_t)(0x80u >> (x & 7));
    if (white) fb[i] |= m; else fb[i] &= (uint8_t)~m;
}

int epd4g_bw_fb_get(const uint8_t *fb, int x, int y)
{
    if (x < 0 || y < 0 || x >= EPD4G_WIDTH || y >= EPD4G_HEIGHT)
        return EPD4G_BW_WHITE;
    return (fb[(size_t)y * EPD4G_BW_STRIDE + (size_t)(x >> 3)] >> (7 - (x & 7))) & 1;
}

void epd4g_bw_fb_set_rot(uint8_t *fb, epd4g_rot_t rot, int x, int y, int white)
{
    int nx, ny;
    if (epd4g_map(rot, x, y, &nx, &ny) == 0)
        epd4g_bw_fb_set(fb, nx, ny, white);
}

int epd4g_bw_fb_get_rot(const uint8_t *fb, epd4g_rot_t rot, int x, int y)
{
    int nx, ny;
    if (epd4g_map(rot, x, y, &nx, &ny) != 0)
        return EPD4G_BW_WHITE;
    return epd4g_bw_fb_get(fb, nx, ny);
}

void epd4g_bw_fb_copy_rect(uint8_t *dst, const uint8_t *src, const epd4g_rect_t *r)
{
    int x0 = r->x0 < 0 ? 0 : r->x0, x1 = r->x1 >= EPD4G_WIDTH ? EPD4G_WIDTH - 1 : r->x1;
    int y0 = r->y0 < 0 ? 0 : r->y0, y1 = r->y1 >= EPD4G_HEIGHT ? EPD4G_HEIGHT - 1 : r->y1;
    if (x0 > x1 || y0 > y1)
        return;
    for (int y = y0; y <= y1; y++) {
        size_t o = (size_t)y * EPD4G_BW_STRIDE + (size_t)(x0 >> 3);
        memcpy(&dst[o], &src[o], (size_t)((x1 >> 3) - (x0 >> 3) + 1));
    }
}

void epd4g_fb_to_bw(uint8_t *bw, const uint8_t *fb)
{
    for (int y = 0; y < EPD4G_HEIGHT; y++)
        for (int xb = 0; xb < EPD4G_BW_STRIDE; xb++) {
            uint8_t v = 0;
            for (int k = 0; k < 8; k++) {
                int x = xb * 8 + k;
                if (epd4g_fb_get(fb, x, y) >= EPD4G_BW_THRESHOLD)
                    v |= (uint8_t)(0x80u >> k);
            }
            bw[(size_t)y * EPD4G_BW_STRIDE + (size_t)xb] = v;
        }
}
