/*
 * eink_demo.c — демонстрация библиотеки epd4g (epd4g/): частичное обновление
 * Waveshare 2.7" V2 в режиме 4 gray без мигания всего экрана.
 *
 * Что делает:
 *   1. epd4g_init() + epd4g_full(EPD4G_FULL_DEEP): один раз рисует фон (квадраты 22x22 px,
 *      4 полутона) штатной 262-кадровой волной - самая чистая картинка.
 *   2. Каждый шаг двигает чёрный треугольник (10 px) с белой обводкой и вызывает
 *      epd4g_partial_diff(): перерисовываются только пиксели, которые изменились.
 *   3. Каждое EINK_FULL_REFRESH_EVERY-е обновление (по умолчанию 8-е) - полное
 *      обновление (epd4g_full(EPD4G_FULL_FAST)), чтобы сбросить накопленные следы.
 *
 * Это пример использования; библиотека от этого файла не зависит.
 *
 * Настройки (-D... в CMakeLists.txt):
 *   EINK_DIAG=1              печать журнала в UART (printf), по умолчанию 0
 *   EINK_PROFILE=1           + разбивка времени по фазам (нужен EPD4G_PROFILE), только при EINK_DIAG
 *   EINK_ROT                 ориентация, EPD4G_ROT_90 (альбомная) по умолчанию
 *   EINK_FULL_REFRESH_EVERY  период полного обновления, 0 - никогда
 *   EINK_STEP_X / EINK_STEP_Y  шаг треугольника за кадр, px
 */

#include "eink_demo.h"
#include "epd4g.h"
#include "epd4g_fb.h"
#include "main.h"           /* HAL_GetTick() */
#include <string.h>

#ifndef EINK_DIAG
#define EINK_DIAG           0
#endif
#if EINK_DIAG
#include <stdio.h>
#define DIAG(...)           printf(__VA_ARGS__)
#else
#define DIAG(...)           ((void)0)
#endif

#ifndef EINK_PROFILE
#define EINK_PROFILE        0
#endif

/* Ориентация логического экрана. Старый переключатель EINK_ROTATE_180=1 тоже работает. */
#ifndef EINK_ROT
#if defined(EINK_ROTATE_180) && EINK_ROTATE_180
#define EINK_ROT            EPD4G_ROT_270
#else
#define EINK_ROT            EPD4G_ROT_90
#endif
#endif

/* Каждое N-е обновление - полное (с миганием). 0 - никогда не делать полное.
 * Обновления считаются после стартового полного: 1..N-1 частичные, N-е полное, ... */
#ifndef EINK_FULL_REFRESH_EVERY
#define EINK_FULL_REFRESH_EVERY 8
#endif

/* Шаг треугольника за одно обновление (px). Скорость ограничена временем update. */
#ifndef EINK_STEP_X
#define EINK_STEP_X         4
#endif
#ifndef EINK_STEP_Y
#define EINK_STEP_Y         3
#endif

#define CELL        22                      /* 264/22 = 12, 176/22 = 8 */
#define TRI_R       10                      /* запас вокруг центра треугольника */

static uint8_t bg[EPD4G_FB_BYTES];          /* фон */
static uint8_t cur[EPD4G_FB_BYTES];         /* текущий кадр (фон + треугольник) */
static uint8_t prev[EPD4G_FB_BYTES];        /* кадр, который сейчас на панели */

static int s_x = 60, s_y = 50, s_dx = EINK_STEP_X, s_dy = EINK_STEP_Y;
static uint32_t s_last_ms;
static uint32_t s_upd_cnt;                  /* обновлений с последнего полного */

/* ------------------------------- рисование -------------------------------- */

static void put_logical(uint8_t *fb, int lx, int ly, uint8_t level)
{
    epd4g_fb_set_rot(fb, EINK_ROT, lx, ly, level);
}

static void render_background(void)
{
    const int lw = epd4g_logical_width(EINK_ROT), lh = epd4g_logical_height(EINK_ROT);
    for (int ly = 0; ly < lh; ly++) {
        for (int lx = 0; lx < lw; lx++) {
            /* уровни 0..3 = EPD4G_BLACK, EPD4G_DARK, EPD4G_LIGHT, EPD4G_WHITE */
            put_logical(bg, lx, ly, (uint8_t)(((lx / CELL) + (ly / CELL)) & 3));
        }
    }
}

/* Расстояние от точки до ребра (положительное внутри). Длины рёбер заданы константами,
 * чтобы не тянуть libm. Вершины: A(-5,-5) B(-5,5) C(5,0), обход такой, что внутри cross <= 0. */
static float edge_d(float ax, float ay, float bx, float by, float len, float px, float py)
{
    float cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    return -cross / len;
}

static void draw_triangle(int cx, int cy)
{
    const float L_AB = 10.0f, L_BC = 11.1803f, L_CA = 11.1803f;
    const float OUTLINE = 1.5f;     /* толщина белой обводки, px */

    for (int dy = -9; dy <= 9; dy++) {
        for (int dx = -9; dx <= 9; dx++) {
            float px = dx + 0.5f, py = dy + 0.5f;
            float d1 = edge_d(-5, -5, -5,  5, L_AB, px, py);
            float d2 = edge_d(-5,  5,  5,  0, L_BC, px, py);
            float d3 = edge_d( 5,  0, -5, -5, L_CA, px, py);
            float d  = d1 < d2 ? d1 : d2;
            if (d3 < d) d = d3;
            if (d >= 0.0f)           put_logical(cur, cx + dx, cy + dy, EPD4G_BLACK);
            else if (d >= -OUTLINE)  put_logical(cur, cx + dx, cy + dy, EPD4G_WHITE);   /* обводка */
        }
    }
}

/* ------------------------- прямоугольники обновления ---------------------- */

static epd4g_rect_t rect_union(epd4g_rect_t a, epd4g_rect_t b)
{
    epd4g_rect_t r;
    r.x0 = a.x0 < b.x0 ? a.x0 : b.x0;  r.x1 = a.x1 > b.x1 ? a.x1 : b.x1;
    r.y0 = a.y0 < b.y0 ? a.y0 : b.y0;  r.y1 = a.y1 > b.y1 ? a.y1 : b.y1;
    return r;
}

static epd4g_rect_t rect_around(int cx, int cy)
{
    epd4g_rect_t r = {0, 0, 0, 0};
    epd4g_map_rect(EINK_ROT, cx - TRI_R, cy - TRI_R, cx + TRI_R, cy + TRI_R, &r);
    return r;
}

/* ------------------------------ обновление -------------------------------- */

static int full_refresh(void)
{
    DIAG("[UPDATE] FULL refresh (every %d-th update)\r\n", EINK_FULL_REFRESH_EVERY);
    uint32_t t0 = HAL_GetTick();
    int rc = epd4g_full(cur, EPD4G_FULL_FAST);
    s_last_ms = HAL_GetTick() - t0;
    if (rc != EPD4G_OK)
        DIAG("[ERROR][UPDATE] FULL failed rc=%d after %lu ms\r\n", rc, (unsigned long)s_last_ms);
    else
        DIAG("[UPDATE] FULL complete in %lu ms\r\n", (unsigned long)s_last_ms);
    return rc;
}

/* Обновляет панель: cur отличается от prev только внутри roi.
 * Каждое EINK_FULL_REFRESH_EVERY-е обновление вместо этого - полное. */
static int push_update(const epd4g_rect_t *roi)
{
    uint32_t changed = 0;

    s_upd_cnt++;
#if EINK_FULL_REFRESH_EVERY > 0
    if (s_upd_cnt >= (uint32_t)EINK_FULL_REFRESH_EVERY) {
        s_upd_cnt = 0;
        return full_refresh() == EPD4G_OK ? 0 : -1;
    }
#endif

    uint32_t t0 = HAL_GetTick();            /* время только вызова библиотеки */
    int rc = epd4g_partial_diff(prev, cur, roi, &changed);
    s_last_ms = changed ? HAL_GetTick() - t0 : 0;
    DIAG("[UPDATE] #%lu rect X=%d..%d Y=%d..%d, pixels in mask=%lu\r\n",
         (unsigned long)s_upd_cnt, roi->x0, roi->x1, roi->y0, roi->y1, (unsigned long)changed);
    if (rc != EPD4G_OK) {
        DIAG("[ERROR][UPDATE] partial failed rc=%d after %lu ms\r\n", rc, (unsigned long)s_last_ms);
        return -1;
    }
    if (changed)
        DIAG("[UPDATE] complete in %lu ms\r\n", (unsigned long)s_last_ms);
#if EINK_DIAG && EINK_PROFILE && defined(EPD4G_PROFILE)
    if (changed) {
        const epd4g_profile_t *p = epd4g_get_profile();
        DIAG("[PROF] setup=%u spi1=%u wait1=%u spi2=%u wait2=%u ms\r\n",
             p->setup, p->spi1, p->wait1, p->spi2, p->wait2);
    }
#endif
    return 0;
}

/* --------------------------------- API ------------------------------------ */

int eink_demo_init(void)
{
    int rc;

    DIAG("[INIT] epd4g_init()\r\n");
    if ((rc = epd4g_init()) != EPD4G_OK) {
        DIAG("[ERROR][INIT] epd4g_init rc=%d\r\n", rc);
        return -1;
    }

    DIAG("[INIT] rendering %dx%d 4-gray background (%lu bytes)\r\n",
         epd4g_logical_width(EINK_ROT), epd4g_logical_height(EINK_ROT), (unsigned long)EPD4G_FB_BYTES);
    render_background();
    memcpy(cur, bg, sizeof cur);

    DIAG("[INIT] full 4-gray refresh (stock 262-frame waveform)\r\n");
    if ((rc = epd4g_full(cur, EPD4G_FULL_DEEP)) != EPD4G_OK) {
        DIAG("[ERROR][INIT] epd4g_full rc=%d\r\n", rc);
        return -1;
    }
    memcpy(prev, cur, sizeof prev);
    s_upd_cnt = 0;

    /* первый кадр с треугольником - уже частичным обновлением */
    epd4g_rect_t r = rect_around(s_x, s_y);
    draw_triangle(s_x, s_y);
    if (push_update(&r) != 0) {
        DIAG("[ERROR][INIT] initial triangle update failed\r\n");
        return -1;
    }
    DIAG("[INIT] complete; initial triangle=(%d,%d)\r\n", s_x, s_y);
    return 0;
}

int eink_demo_step(void)
{
    const int lw = epd4g_logical_width(EINK_ROT), lh = epd4g_logical_height(EINK_ROT);
    int ox = s_x, oy = s_y;

    memcpy(prev, cur, sizeof prev);         /* то, что сейчас на панели */

    s_x += s_dx;
    s_y += s_dy;
    if (s_x < 14 || s_x > lw - 15) { s_dx = -s_dx; s_x += 2 * s_dx; }
    if (s_y < 14 || s_y > lh - 15) { s_dy = -s_dy; s_y += 2 * s_dy; }

    epd4g_rect_t r = rect_union(rect_around(ox, oy), rect_around(s_x, s_y));
    DIAG("[FRAME] triangle (%d,%d)->(%d,%d)\r\n", ox, oy, s_x, s_y);
    epd4g_fb_copy_rect(cur, bg, &r);        /* стереть старый треугольник фоном */
    draw_triangle(s_x, s_y);
    if (push_update(&r) != 0) {
        DIAG("[ERROR][FRAME] frame update failed; aborting animation\r\n");
        return -1;
    }
    return 0;
}

uint32_t eink_demo_last_update_ms(void)
{
    return s_last_ms;
}
