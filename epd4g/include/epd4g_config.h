/**
 * @file epd4g_config.h
 * @brief Compile-time configuration of the epd4g library.
 *
 * Every macro can be overridden in three ways (first one wins):
 *   1. -DNAME=value on the compiler command line / in target_compile_definitions();
 *   2. a project header: compile with -DEPD4G_USER_CONFIG="my_epd4g_cfg.h";
 *   3. edit the default below (not recommended for a git submodule).
 */
#ifndef EPD4G_CONFIG_H
#define EPD4G_CONFIG_H

#ifdef EPD4G_USER_CONFIG
#include EPD4G_USER_CONFIG
#endif

/* ------------------------------------------------------------------------- *
 * Partial update waveform, in panel frames (1 frame = 20 ms at the stock
 * frame rate). A partial update is two passes:
 *   pass 1: black pixels get VSH for EPD4G_T_BLACK frames, every other updated
 *           pixel gets VSL for EPD4G_T_WHITE frames (saturates to white);
 *   pass 2: light / dark gray pixels get VSH for EPD4G_T_LIGHT / EPD4G_T_DARK frames.
 * Duration = max(T_WHITE, T_BLACK) + T_DARK frames  (45 + 8 = 53 -> ~1.06 s).
 * ------------------------------------------------------------------------- */
#ifndef EPD4G_T_LIGHT
#define EPD4G_T_LIGHT   3    /**< light gray: VSH after the erase                     */
#endif
#ifndef EPD4G_T_DARK
#define EPD4G_T_DARK    8    /**< dark gray: cumulative VSH after the erase (>= LIGHT) */
#endif
#ifndef EPD4G_T_BLACK
#define EPD4G_T_BLACK   45   /**< black: VSH that saturates a pixel to black. 25 (stock) under-drives black */
#endif
#ifndef EPD4G_T_WHITE
#define EPD4G_T_WHITE   40   /**< white: VSL that saturates a pixel to white from any state */
#endif

/* ------------------------------------------------------------------------- *
 * Full refresh, EPD4G_FULL_FAST mode. The stock 262-frame waveform (used by
 * EPD4G_FULL_DEEP) has a "shake" group of VSH/VSL pulses of TP frames, repeated
 * RP+1 times. Fewer repeats = faster, weaker ghost removal.
 *   RP=3  stock, 262 frames (~5.2 s)      RP=1  178 frames (~3.6 s)
 *   RP=0  136 frames (~2.7 s), default
 * ------------------------------------------------------------------------- */
#ifndef EPD4G_FULL_SHAKE_RP
#define EPD4G_FULL_SHAKE_RP  0
#endif
#ifndef EPD4G_FULL_SHAKE_TP
#define EPD4G_FULL_SHAKE_TP  20
#endif

/* ------------------------------------------------------------------------- *
 * Power management.
 * 1 = after an update the panel's analog part stays on (0x22=0xC4). Saves ~330 ms
 *     per power-up/power-down cycle; call epd4g_power_off() when going idle.
 * 0 = every update powers the analog part up and down (0x22=0xC7, safe default
 *     for rarely updated displays, but ~330 ms slower per pass).
 * ------------------------------------------------------------------------- */
#ifndef EPD4G_KEEP_POWER
#define EPD4G_KEEP_POWER 1
#endif

/* ------------------------------------------------------------------------- *
 * BUSY handling
 * ------------------------------------------------------------------------- */
#ifndef EPD4G_BUSY_TIMEOUT_MS
#define EPD4G_BUSY_TIMEOUT_MS  10000U   /**< a stuck panel returns EPD4G_ERR_TIMEOUT instead of hanging */
#endif
#ifndef EPD4G_BUSY_POLL_MS
#define EPD4G_BUSY_POLL_MS     1U       /**< epd4g_port_delay_ms() between BUSY reads (RTOS yield point) */
#endif
#ifndef EPD4G_BUSY_SETTLE_MS
#define EPD4G_BUSY_SETTLE_MS   2U       /**< pause after BUSY falls (stock Waveshare code uses 20) */
#endif

/* ------------------------------------------------------------------------- *
 * Optional features / diagnostics (all off by default, zero cost when off)
 * ------------------------------------------------------------------------- */

/** Define EPD4G_PROFILE to record per-phase timings: see epd4g_get_profile(). */
/* #define EPD4G_PROFILE */

/** Define EPD4G_LOG(...) as a printf-like function to get library messages
 *  (init steps, errors). Default: no output. The definition must be self-contained,
 *  e.g. in a user config header: #include <stdio.h>  #define EPD4G_LOG(...) printf(__VA_ARGS__) */
#ifndef EPD4G_LOG
#define EPD4G_LOG(...) ((void)0)
#endif

/** 1 = epd4g_partial_diff() and epd4g_partial_rect() are built (in epd4g_fb.c) and share one
 *  static 5808-byte mask buffer. 0 = omitted: build your own mask and call epd4g_partial(). */
#ifndef EPD4G_INTERNAL_MASK
#define EPD4G_INTERNAL_MASK 1
#endif

/** The library forces -O2 for itself under GCC when the project is built without
 *  optimisation (Debug, -O0): the bit conversion loops are ~10x slower at -O0.
 *  Define EPD4G_NO_FORCE_O2 to disable that. */
/* #define EPD4G_NO_FORCE_O2 */

#endif /* EPD4G_CONFIG_H */
