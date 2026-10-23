/**
 * @file epd4g_fb.h
 * @brief Optional helpers: frame buffer pixels, screen rotation, update masks.
 */
#ifndef EPD4G_FB_H
#define EPD4G_FB_H

#include "epd4g.h"
#include "epd4g_bw.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Screen orientation. The panel is native portrait 176x264; the other
 *  orientations give a logical coordinate system rotated onto it. */
typedef enum {
    EPD4G_ROT_0   = 0,  /**< portrait, 176 x 264, connector at the top (native)        */
    EPD4G_ROT_90  = 1,  /**< landscape, 264 x 176 (the reference project's default)     */
    EPD4G_ROT_180 = 2,  /**< portrait upside down, 176 x 264                            */
    EPD4G_ROT_270 = 3   /**< landscape upside down, 264 x 176                           */
} epd4g_rot_t;

/** Logical width / height for an orientation. */
int epd4g_logical_width(epd4g_rot_t rot);
int epd4g_logical_height(epd4g_rot_t rot);

/** Logical -> native coordinates. Returns 0, or -1 if (x,y) is outside the screen. */
int epd4g_map(epd4g_rot_t rot, int x, int y, int *nx, int *ny);

/** Bounding rectangle (native) of a logical rectangle (inclusive, clipped). Returns 0, or -1 if empty. */
int epd4g_map_rect(epd4g_rot_t rot, int x0, int y0, int x1, int y1, epd4g_rect_t *out);

/** Fill the whole buffer with one level (EPD4G_BLACK..EPD4G_WHITE). */
void epd4g_fb_fill(uint8_t *fb, uint8_t level);

/** Pixel access in NATIVE coordinates; out-of-range access is ignored / returns EPD4G_WHITE. */
void    epd4g_fb_set(uint8_t *fb, int x, int y, uint8_t level);
uint8_t epd4g_fb_get(const uint8_t *fb, int x, int y);

/** Pixel access in LOGICAL coordinates of the given orientation. */
void    epd4g_fb_set_rot(uint8_t *fb, epd4g_rot_t rot, int x, int y, uint8_t level);
uint8_t epd4g_fb_get_rot(const uint8_t *fb, epd4g_rot_t rot, int x, int y);

/** Copy the rows of @p rect (full byte columns, so a few neighbouring pixels may be copied too)
 *  from @p src to @p dst: restore a background under a moved sprite. */
void epd4g_fb_copy_rect(uint8_t *dst, const uint8_t *src, const epd4g_rect_t *rect);

/* ----- update masks (EPD4G_MASK_BYTES bytes, 1 bit per pixel) ----- */
void epd4g_mask_clear(uint8_t *mask);
void epd4g_mask_set(uint8_t *mask, int x, int y);                  /**< native coordinates */
void epd4g_mask_set_rect(uint8_t *mask, const epd4g_rect_t *rect);

/** Build a mask of the pixels that differ between @p prev and @p cur inside @p roi
 *  (NULL = whole panel). The mask is cleared first. Returns the number of set bits. */
uint32_t epd4g_mask_from_diff(uint8_t *mask, const uint8_t *prev, const uint8_t *cur,
                              const epd4g_rect_t *roi);

/* ----- 1-bit frame helpers (EPD4G_SRC_1BPP buffers, EPD4G_BW_FB_BYTES bytes, 1 = white) ----- */
void epd4g_bw_fb_fill(uint8_t *fb, int white);
void epd4g_bw_fb_set(uint8_t *fb, int x, int y, int white);        /**< native coordinates */
int  epd4g_bw_fb_get(const uint8_t *fb, int x, int y);
void epd4g_bw_fb_set_rot(uint8_t *fb, epd4g_rot_t rot, int x, int y, int white);   /**< logical */
int  epd4g_bw_fb_get_rot(const uint8_t *fb, epd4g_rot_t rot, int x, int y);
void epd4g_bw_fb_copy_rect(uint8_t *dst, const uint8_t *src, const epd4g_rect_t *rect);

/** Convert a whole 4-gray frame to a 1-bit one (pixel >= EPD4G_BW_THRESHOLD -> white). Normally not
 *  needed: pass the 4-gray frame with EPD4G_SRC_2BPP and it is converted while it is sent. */
void epd4g_fb_to_bw(uint8_t *bw, const uint8_t *fb);

#ifdef __cplusplus
}
#endif

#endif /* EPD4G_FB_H */
