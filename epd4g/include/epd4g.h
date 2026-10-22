/**
 * @file epd4g.h
 * @brief Partial and fast full refresh in the 4-gray mode of the Waveshare
 *        2.7" e-Paper HAT (V2), 264x176.
 *
 * The vendor library can refresh a part of the screen only in 1-bit mode. This
 * library does it in 4 grays: pixels outside the update mask get zero drive, so the
 * rest of the screen does not flash. Pixel-exact masks are supported.
 *
 * Typical use (see README.md):
 * @code
 *   static uint8_t fb[EPD4G_FB_BYTES], prev[EPD4G_FB_BYTES];
 *   epd4g_init();
 *   draw_everything(fb);
 *   epd4g_full(fb, EPD4G_FULL_DEEP);                 // clean first picture
 *   memcpy(prev, fb, sizeof fb);
 *   for (;;) {
 *       change_a_few_pixels(fb);
 *       epd4g_partial_diff(prev, fb, NULL, NULL);    // ~1.4 s, no full-screen flash
 *       memcpy(prev, fb, sizeof fb);
 *   }
 * @endcode
 *
 * Not re-entrant, every call blocks until the panel is done (partial ~1.4 s,
 * full ~2.9 s with the default configuration). Not thread-safe.
 */
#ifndef EPD4G_H
#define EPD4G_H

#include <stddef.h>
#include <stdint.h>
#include "epd4g_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- geometry --------------------------------- */
/** Native (portrait) panel size in pixels. Use epd4g_fb.h for rotated coordinates. */
#define EPD4G_WIDTH         176
#define EPD4G_HEIGHT        264

/** Frame buffer: 2 bits per pixel, 4 pixels per byte, first pixel in the top 2 bits,
 *  44 bytes per row, 264 rows, 11616 bytes. Same format as the Waveshare 4-gray buffer. */
#define EPD4G_FB_STRIDE     (EPD4G_WIDTH / 4)
#define EPD4G_FB_BYTES      (EPD4G_FB_STRIDE * EPD4G_HEIGHT)

/** Update mask: 1 bit per pixel, MSB first, 22 bytes per row, 5808 bytes. 1 = update. */
#define EPD4G_MASK_STRIDE   (EPD4G_WIDTH / 8)
#define EPD4G_MASK_BYTES    (EPD4G_MASK_STRIDE * EPD4G_HEIGHT)

/* ------------------------------ levels ---------------------------------- */
/** 2-bit pixel values of the frame buffer. */
enum {
    EPD4G_BLACK = 0,   /**< 00 */
    EPD4G_DARK  = 1,   /**< 01 dark gray  */
    EPD4G_LIGHT = 2,   /**< 10 light gray */
    EPD4G_WHITE = 3    /**< 11 */
};

/* ------------------------------ status ---------------------------------- */
enum {
    EPD4G_OK            =  0,
    EPD4G_ERR_TIMEOUT   = -1,  /**< BUSY did not fall within EPD4G_BUSY_TIMEOUT_MS; call epd4g_init() again */
    EPD4G_ERR_NOT_INIT  = -2,  /**< epd4g_init() was not called (or the panel is asleep / timed out) */
    EPD4G_ERR_PARAM     = -3,  /**< NULL pointer or empty rectangle */
    EPD4G_ERR_IO        = -4   /**< epd4g_port_spi_write() failed */
};

typedef enum {
    EPD4G_FULL_FAST = 0,  /**< shortened shake group (EPD4G_FULL_SHAKE_*): ~2.9 s with defaults */
    EPD4G_FULL_DEEP = 1   /**< stock Waveshare 262-frame waveform: ~5.3 s, best ghost removal   */
} epd4g_full_mode_t;

/** Inclusive pixel rectangle in native panel coordinates (x 0..175, y 0..263). */
typedef struct {
    int x0, y0, x1, y1;
} epd4g_rect_t;

/* ------------------------------- API ------------------------------------ */

/** Hardware reset and register setup (identical to the stock Init_4GRAY sequence).
 *  Call once at start and after epd4g_sleep() or a timeout. */
int epd4g_init(void);

/** Full refresh of the whole panel from @p fb (the screen flashes).
 *  Requires epd4g_init(). */
int epd4g_full(const uint8_t *fb, epd4g_full_mode_t mode);

/** Partial refresh: only the pixels set in @p mask are redrawn, taking their target
 *  grays from @p fb. Pixels with a 0 mask bit must hold on the panel what @p fb
 *  holds for them. Requires a previous epd4g_full() so that the panel content is known. */
int epd4g_partial(const uint8_t *fb, const uint8_t *mask);

#if EPD4G_INTERNAL_MASK
/* The two helpers below live in epd4g_fb.c and share one internal 5808-byte mask buffer. */

/** Partial refresh of a rectangle (native coordinates, inclusive). */
int epd4g_partial_rect(const uint8_t *fb, const epd4g_rect_t *rect);

/** Convenience: compare @p prev (what is on the panel) with @p cur (what should be there),
 *  redraw exactly the pixels that differ. @p roi limits the comparison to a rectangle
 *  (NULL = whole panel). @p changed (optional) receives the number of differing pixels;
 *  if there are none the panel is not touched and 0 is returned. */
int epd4g_partial_diff(const uint8_t *prev, const uint8_t *cur,
                       const epd4g_rect_t *roi, uint32_t *changed);
#endif

/** Power the panel's analog part down. Only needed with EPD4G_KEEP_POWER=1 when the
 *  display goes idle; the next update powers it up again (adds ~300 ms once). */
int epd4g_power_off(void);

/** Power down and enter deep sleep (lowest consumption). Call epd4g_init() to wake up. */
int epd4g_sleep(void);

#ifdef EPD4G_PROFILE
/** Phase durations of the last update, ms. A full refresh fills setup/spi1/wait1 only. */
typedef struct {
    uint16_t setup;    /**< registers + LUT of pass 1            */
    uint16_t spi1;     /**< RAM planes of pass 1                 */
    uint16_t wait1;    /**< 0x20 -> BUSY low: waveform of pass 1 */
    uint16_t spi2;     /**< LUT + RAM planes of pass 2           */
    uint16_t wait2;    /**< waveform of pass 2                   */
} epd4g_profile_t;
const epd4g_profile_t *epd4g_get_profile(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* EPD4G_H */
