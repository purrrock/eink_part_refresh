/**
 * @file epd4g_bw.h
 * @brief 1-bit (black/white) mode of the Waveshare 2.7" e-Paper HAT V2, 264x176.
 *
 * Replaces the stock EPD_2in7_V2 driver: the init, fast init, full, fast full, clear, partial
 * and sleep sequences are the stock ones (verified byte for byte by tests/ against the vendor
 * library), with the same non-blocking state machine as the 4-gray mode (epd4g.h).
 *
 * Typical use:
 * @code
 *   epd4g_start_bw_init(EPD4G_BW_NORMAL);   // or the blocking epd4g_bw_init()
 *   ... poll until done ...
 *   epd4g_start_bw_full(img, EPD4G_SRC_2BPP, EPD4G_BW_NORMAL);   // baseline picture, flashes
 *   ... poll until done ...
 *   epd4g_start_bw_partial(img, EPD4G_SRC_2BPP, &rect);          // ~0.3-0.5 s, only that window
 * @endcode
 *
 * The panel keeps the old and the new picture in two RAM planes and compares them itself, so
 * no copy of the previous frame is needed on the MCU. A partial update draws the window rows
 * of the image you pass; everything outside the window is untouched.
 *
 * Sleep: epd4g_sleep() in the 1-bit mode keeps the RAM planes. The stock flow "Init + Display_Base +
 * Sleep ... Display_Partial + Sleep" therefore works: epd4g_bw_partial() wakes the panel with its
 * hardware reset (EPD4G_BW_PARTIAL_RESET=1). A full refresh after sleep needs epd4g_bw_init() first.
 *
 * Mode switching: epd4g_init() (4 gray) and epd4g_bw_init() both reset the panel. After either
 * of them the first update must be a full one (epd4g_full() / epd4g_bw_full()).
 */
#ifndef EPD4G_BW_H
#define EPD4G_BW_H

#include "epd4g.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 1-bit frame buffer: 1 bit per pixel, MSB first, 1 = white, 0 = black, 22 bytes per row,
 *  264 rows, 5808 bytes (the stock Waveshare format, native portrait orientation). */
#define EPD4G_BW_STRIDE     (EPD4G_WIDTH / 8)
#define EPD4G_BW_FB_BYTES   (EPD4G_BW_STRIDE * EPD4G_HEIGHT)
#define EPD4G_BW_BLACK      0
#define EPD4G_BW_WHITE      1

/** Format of the image passed to the 1-bit update functions. */
typedef enum {
    EPD4G_SRC_1BPP  = 0,   /**< EPD4G_BW_FB_BYTES buffer, see above                                  */
    EPD4G_SRC_2BPP  = 1,   /**< a 4-gray frame buffer (EPD4G_FB_BYTES), converted row by row while
                                it is sent: pixel >= EPD4G_BW_THRESHOLD -> white. No extra buffer.  */
    EPD4G_SRC_WHITE = 2    /**< all white, the image pointer is ignored (may be NULL)                */
} epd4g_src_t;

typedef enum {
    EPD4G_BW_NORMAL = 0,   /**< stock Init / Display: temperature-compensated OTP waveform, ~2 s     */
    EPD4G_BW_FAST   = 1    /**< stock Init_Fast / Display_Fast: shorter waveform, ~1 s               */
} epd4g_bw_mode_t;

/* ----------------------------- blocking API ------------------------------- */

/** Reset and register setup (stock EPD_2IN7_V2_Init, or _Init_Fast for EPD4G_BW_FAST).
 *  Switches the panel to 1-bit mode. */
int epd4g_bw_init(epd4g_bw_mode_t mode);

/** Full refresh from @p img, written to both RAM planes (stock Display_Base). EPD4G_BW_FAST needs
 *  epd4g_bw_init(EPD4G_BW_FAST) and no partial update since (a partial update loads another
 *  waveform): otherwise EPD4G_ERR_MODE. */
int epd4g_bw_full(const void *img, epd4g_src_t src, epd4g_bw_mode_t mode);

/** Full refresh to white (stock Clear). */
int epd4g_bw_clear(epd4g_bw_mode_t mode);

/** Partial refresh of @p rect (native coordinates, inclusive; x is widened to whole bytes, i.e.
 *  8 pixel columns; NULL = whole screen). The rows of @p img inside the window are sent, so
 *  @p img is a full-screen frame. Needs a previous epd4g_bw_full() after epd4g_bw_init(),
 *  otherwise EPD4G_ERR_NO_BASE. */
int epd4g_bw_partial(const void *img, epd4g_src_t src, const epd4g_rect_t *rect);

/* --------------------------- non-blocking API ---------------------------- */
/* Same arguments and checks as above; see epd4g.h (epd4g_poll(), epd4g_result(), ...).
 * @p img must stay unchanged until the job is finished. */
int epd4g_start_bw_init(epd4g_bw_mode_t mode);
int epd4g_start_bw_full(const void *img, epd4g_src_t src, epd4g_bw_mode_t mode);
int epd4g_start_bw_clear(epd4g_bw_mode_t mode);
int epd4g_start_bw_partial(const void *img, epd4g_src_t src, const epd4g_rect_t *rect);

/* Deep sleep and power-off work in both modes: epd4g_sleep(), epd4g_power_off() (epd4g.h). */

#ifdef __cplusplus
}
#endif

#endif /* EPD4G_BW_H */
