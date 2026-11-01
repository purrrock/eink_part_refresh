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
 *   4. Основной цикл НЕ блокируется: eink_demo_task() вызывается из while(1) в main.c и только
 *      продвигает автомат библиотеки (epd4g_poll()); пока панель рисует, main-цикл свободен
 *      (приём GNSS, кнопки и т. п.). Каждые EINK_BW_EVERY кадров демо переключается в 1-bit режим
 *      («меню» с курсором, быстрое частичное обновление), затем возвращается в 4 gray.
 *
 * Это пример использования; библиотека от этого файла не зависит.
 *
 * Настройки (-D... в CMakeLists.txt):
 *   EINK_DIAG=1              печать журнала в UART (printf), по умолчанию 0
 *   EINK_PROFILE=1           + разбивка времени по фазам (нужен EPD4G_PROFILE), только при EINK_DIAG
 *   EINK_ROT                 ориентация, EPD4G_ROT_90 (альбомная) по умолчанию
 *   EINK_FULL_REFRESH_EVERY  период полного обновления, 0 - никогда
 *   EINK_STEP_X / EINK_STEP_Y  шаг треугольника за кадр, px
 *   EINK_BW_DEMO             1 - показывать 1-bit «меню» (по умолчанию 1), 0 - только 4 gray
 *   EINK_BW_EVERY            через сколько кадров 4 gray переключаться в 1-bit меню
 *   EINK_BW_STEPS            сколько шагов курсора показать в 1-bit меню
 */

#include "eink_demo.h"
#include "epd4g.h"
#include "epd4g_fb.h"
#include "epd4g_bw.h"
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

#ifndef EINK_BW_DEMO
#define EINK_BW_DEMO        1
#endif
#ifndef EINK_BW_EVERY
#define EINK_BW_EVERY       12
#endif
#ifndef EINK_BW_STEPS
#define EINK_BW_STEPS       10
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

/* ---------------------- неблокирующий автомат демо ------------------------- */

/* Что запустить следующим, когда библиотека освободится. */
typedef enum {
    A_GRAY_FRAME,       /* кадр анимации 4 gray (или переход в 1-bit меню)         */
    A_BW_FULL,          /* 1-bit init выполнен: полное обновление меню             */
    A_BW_STEP,          /* шаг курсора меню частичным обновлением                  */
    A_GRAY_FULL         /* 4-gray init выполнен: полное обновление 4 gray          */
} action_t;

#define MENU_ROWS   6
#define MENU_ROW_H  26
#define MENU_Y0     6

static uint8_t bw[EPD4G_BW_FB_BYTES];       /* кадр 1-bit меню */
static action_t s_act = A_GRAY_FRAME;
static int  s_job;                          /* запущена задача, результат ещё не проверен */
static uint32_t s_job_t0;
static uint32_t s_loops;                    /* проходов main-цикла за время задачи */
static int  s_frames, s_bw_steps, s_sel;

static void bw_rect_fill(int x0, int y0, int x1, int y1, int white)
{
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            epd4g_bw_fb_set_rot(bw, EINK_ROT, x, y, white);
}

/* строка меню: рамка 2 px; выбранная - залита чёрным с белой «меткой» слева */
static epd4g_rect_t bw_draw_row(int i, int selected)
{
    const int lw = epd4g_logical_width(EINK_ROT);
    const int y0 = MENU_Y0 + i * MENU_ROW_H, y1 = y0 + MENU_ROW_H - 3;
    epd4g_rect_t r = {0, 0, 0, 0};

    bw_rect_fill(6, y0, lw - 7, y1, 1);
    bw_rect_fill(8, y0, lw - 9, y1, 0);                 /* чёрный прямоугольник */
    if (!selected)
        bw_rect_fill(10, y0 + 2, lw - 11, y1 - 2, 1);   /* внутри белый: остаётся рамка */
    else
        bw_rect_fill(14, y0 + 6, 30, y1 - 6, 1);        /* метка */
    bw_rect_fill(40, y0 + 8, 40 + 20 * (i + 2), y0 + 12, selected ? 1 : 0);   /* «текст» */
    epd4g_map_rect(EINK_ROT, 6, y0, lw - 7, y1, &r);
    return r;
}

static void bw_render_menu(void)
{
    epd4g_bw_fb_fill(bw, 1);
    for (int i = 0; i < MENU_ROWS; i++)
        bw_draw_row(i, i == s_sel);
}

static int job_started(int rc)
{
    if (rc != EPD4G_OK) {
        DIAG("[ERROR][TASK] start failed rc=%d\r\n", rc);
        return -1;
    }
    if (epd4g_is_busy()) {
        s_job = 1; s_job_t0 = HAL_GetTick(); s_loops = 0;
    }
    return 0;
}

/* Запускает обновление текущего кадра 4 gray (prev -> cur). */
static int start_gray_update(const epd4g_rect_t *roi)
{
    uint32_t changed = 0;

    s_upd_cnt++;
#if EINK_FULL_REFRESH_EVERY > 0
    if (s_upd_cnt >= (uint32_t)EINK_FULL_REFRESH_EVERY) {
        s_upd_cnt = 0;
        DIAG("[UPDATE] FULL refresh (every %d-th update)\r\n", EINK_FULL_REFRESH_EVERY);
        return job_started(epd4g_start_full(cur, EPD4G_FULL_FAST));
    }
#endif
    int rc = epd4g_start_partial_diff(prev, cur, roi, &changed);
    DIAG("[UPDATE] #%lu rect X=%d..%d Y=%d..%d, pixels in mask=%lu\r\n",
         (unsigned long)s_upd_cnt, roi->x0, roi->x1, roi->y0, roi->y1, (unsigned long)changed);
    return job_started(rc);
}

static int start_next(void)
{
    switch (s_act) {
    case A_GRAY_FRAME: {
#if EINK_BW_DEMO
        if (s_frames >= EINK_BW_EVERY) {
            DIAG("[MODE] 4 gray -> 1 bit\r\n");
            s_frames = 0; s_bw_steps = 0; s_sel = 0;
            s_act = A_BW_FULL;
            return job_started(epd4g_start_bw_init(EPD4G_BW_NORMAL));
        }
#endif
        const int lw = epd4g_logical_width(EINK_ROT), lh = epd4g_logical_height(EINK_ROT);
        int ox = s_x, oy = s_y;

        s_frames++;
        memcpy(prev, cur, sizeof prev);     /* то, что сейчас на панели */
        s_x += s_dx;
        s_y += s_dy;
        if (s_x < 14 || s_x > lw - 15) { s_dx = -s_dx; s_x += 2 * s_dx; }
        if (s_y < 14 || s_y > lh - 15) { s_dy = -s_dy; s_y += 2 * s_dy; }

        epd4g_rect_t r = rect_union(rect_around(ox, oy), rect_around(s_x, s_y));
        DIAG("[FRAME] triangle (%d,%d)->(%d,%d)\r\n", ox, oy, s_x, s_y);
        epd4g_fb_copy_rect(cur, bg, &r);    /* стереть старый треугольник фоном */
        draw_triangle(s_x, s_y);
        return start_gray_update(&r);       /* cur и prev не трогаем, пока задача не закончится */
    }
    case A_BW_FULL:
        bw_render_menu();
        s_act = A_BW_STEP;
        DIAG("[MODE] 1-bit full refresh\r\n");
        return job_started(epd4g_start_bw_full(bw, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL));
    case A_BW_STEP: {
        if (s_bw_steps >= EINK_BW_STEPS) {
            DIAG("[MODE] 1 bit -> 4 gray\r\n");
            s_act = A_GRAY_FULL;
            return job_started(epd4g_start_init());
        }
        epd4g_rect_t a = bw_draw_row(s_sel, 0);
        s_sel = (s_sel + 1) % MENU_ROWS;
        epd4g_rect_t b = bw_draw_row(s_sel, 1);
        epd4g_rect_t r = rect_union(a, b);
        s_bw_steps++;
        DIAG("[UPDATE] 1-bit cursor -> row %d\r\n", s_sel);
        return job_started(epd4g_start_bw_partial(bw, EPD4G_SRC_1BPP, &r));
    }
    case A_GRAY_FULL:
        s_act = A_GRAY_FRAME;
        memcpy(prev, cur, sizeof prev);
        s_upd_cnt = 0;
        return job_started(epd4g_start_full(cur, EPD4G_FULL_DEEP));
    }
    return -1;
}

/* Вызывать из while(1) в main.c как можно чаще. Возвращает 0, либо -1 при ошибке дисплея. */
int eink_demo_task(void)
{
    s_loops++;
    epd4g_poll();
    if (epd4g_is_busy())
        return 0;                           /* панель рисует, main-цикл свободен */

    if (s_job) {                            /* задача только что закончилась */
        int rc = epd4g_result();
        s_job = 0;
        s_last_ms = HAL_GetTick() - s_job_t0;
        if (rc != EPD4G_OK) {
            DIAG("[ERROR][TASK] job failed rc=%d after %lu ms\r\n", rc, (unsigned long)s_last_ms);
            return -1;
        }
        DIAG("[UPDATE] complete in %lu ms, main loop passes meanwhile: %lu\r\n",
             (unsigned long)s_last_ms, (unsigned long)s_loops);
    }
    return start_next();
}

uint32_t eink_demo_last_update_ms(void)
{
    return s_last_ms;
}
