/*
 * eink_demo.c — демонстрация библиотеки epd4g (epd4g/): частичное обновление
 * Waveshare 2.7" V2 в режиме 4 gray без мигания всего экрана.
 *
 * Что делает (экран «как у навигатора»):
 *   1. render_map(): один раз рисует карту в 4 gray: белый фон, светло-серые улочки и парки,
 *      тёмно-серые дороги и водоём, чёрные контуры домов, железная дорога и кромка маршрута.
 *   2. epd4g_init() + epd4g_full(EPD4G_FULL_DEEP): первая картинка штатной 262-кадровой волной.
 *   3. Чёрный маркёр (стрелка с белой обводкой) едет по замкнутому маршруту и оставляет за собой
 *      чёрный трек. Каждый шаг - epd4g_partial_diff(): перерисовываются только изменившиеся
 *      пиксели (маркёр на новом месте, стёртый старый маркёр, новый кусок трека), остальная
 *      карта остаётся нетронутой и не мигает.
 *   4. Когда маркёр проехал круг, трек стирается, карта перерисовывается полным обновлением.
 *   5. Каждое EINK_FULL_REFRESH_EVERY-е обновление - полное (epd4g_full(EPD4G_FULL_FAST)),
 *      чтобы сбросить накопленные следы.
 *
 *   Основной цикл НЕ блокируется: eink_demo_task() вызывается из while(1) в main.c и только
 *   продвигает автомат библиотеки (epd4g_poll()); пока панель рисует, main-цикл свободен
 *   (приём GNSS, кнопки и т. п.). Каждые EINK_BW_EVERY кадров демо переключается в 1-bit режим
 *   («меню» с курсором, быстрое частичное обновление), затем возвращается в 4 gray.
 *
 * Слои кадра (каждый - буфер EPD4G_FB_BYTES):
 *   bg    карта без трека (то, что рисует render_map)
 *   base  bg + проехавшийся трек (то, что должно быть под маркёром)
 *   cur   base + маркёр (то, что должно быть на панели)
 *   prev  то, что сейчас на панели
 *
 * Это пример использования; библиотека от этого файла не зависит.
 *
 * Настройки (-D... в CMakeLists.txt):
 *   EINK_DIAG=1              печать журнала в UART (printf), по умолчанию 0
 *   EINK_PROFILE=1           + разбивка времени по фазам (нужен EPD4G_PROFILE), только при EINK_DIAG
 *   EINK_ROT                 ориентация, EPD4G_ROT_90 (альбомная) по умолчанию
 *   EINK_FULL_REFRESH_EVERY  период полного обновления, 0 - никогда
 *   EINK_STEP_PX             путь маркёра за одно обновление, px (по умолчанию 6)
 *   EINK_TRACK_W             толщина трека, px (1, 2, 3, 5 или 7; по умолчанию 2)
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

/* 1 - сборка теста тока (eink_power_test.c): файл подключается целиком ради рисования карты,
 * маркёра и трека, а демо-автомат и eink_demo_xxx() исключаются (их заменяет тест). */
#ifndef EINK_POWER_TEST
#define EINK_POWER_TEST     0
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

/* Путь маркёра за одно обновление (px). Скорость ограничена временем update. */
#ifndef EINK_STEP_PX
#define EINK_STEP_PX        6
#endif
/* Толщина чёрного трека, px: 1, 2, 3, 5 или 7. */
#ifndef EINK_TRACK_W
#define EINK_TRACK_W        2
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

#define MK_R        10                      /* запас вокруг центра маркёра (его размер +-9 px) */

static uint8_t bg[EPD4G_FB_BYTES];          /* карта без трека */
static uint8_t base[EPD4G_FB_BYTES];        /* карта + трек */
static uint8_t cur[EPD4G_FB_BYTES];         /* текущий кадр (карта + трек + маркёр) */
static uint8_t prev[EPD4G_FB_BYTES];        /* кадр, который сейчас на панели */

static uint32_t s_last_ms;
static uint32_t s_upd_cnt;                  /* обновлений с последнего полного */

/* ------------------------- примитивы рисования ----------------------------- */
/* Все координаты - логические (с учётом EINK_ROT), выход за экран отсекается. */

static void put_px(uint8_t *fb, int x, int y, uint8_t level)
{
    epd4g_fb_set_rot(fb, EINK_ROT, x, y, level);
}

static int iabs_(int v) { return v < 0 ? -v : v; }

/* Целочисленный корень (чтобы не тянуть libm). */
static uint32_t isqrt_u32(uint32_t v)
{
    uint32_t r = 0, b = 1u << 30;
    while (b > v) b >>= 2;
    while (b) {
        if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return r;
}

/* Длина вектора (dx,dy) с точностью 1/16 px. */
static float vec_len(int dx, int dy)
{
    return (float)isqrt_u32((uint32_t)(dx * dx + dy * dy) * 256u) / 16.0f;
}

/* «кисть»: квадрат 1x1 / 2x2 или диск диаметром w (3, 5, 7). */
static void stamp(uint8_t *fb, int x, int y, uint8_t level, int w)
{
    if (w <= 1) { put_px(fb, x, y, level); return; }
    if (w == 2) {
        put_px(fb, x, y, level);     put_px(fb, x + 1, y, level);
        put_px(fb, x, y + 1, level); put_px(fb, x + 1, y + 1, level);
        return;
    }
    const int r = w / 2;
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx * dx + dy * dy <= r * r + r)
                put_px(fb, x + dx, y + dy, level);
}

/* Линия Брезенхэма кистью толщины w. */
static void line(uint8_t *fb, int x0, int y0, int x1, int y1, uint8_t level, int w)
{
    const int dx = iabs_(x1 - x0), sx = x0 < x1 ? 1 : -1;
    const int dy = -iabs_(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        stamp(fb, x0, y0, level, w);
        if (x0 == x1 && y0 == y1)
            break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* Ломаная из n точек, v = {x0,y0, x1,y1, ...}. */
static void polyline(uint8_t *fb, const int16_t *v, int n, uint8_t level, int w)
{
    for (int i = 0; i + 1 < n; i++)
        line(fb, v[2 * i], v[2 * i + 1], v[2 * i + 2], v[2 * i + 3], level, w);
}

/* Заливка многоугольника (чётно-нечётное правило) и контур. */
static void poly_fill(uint8_t *fb, const int16_t *v, int n, uint8_t level)
{
    int x0 = v[0], x1 = v[0], y0 = v[1], y1 = v[1];
    for (int i = 1; i < n; i++) {
        if (v[2 * i] < x0) x0 = v[2 * i];
        if (v[2 * i] > x1) x1 = v[2 * i];
        if (v[2 * i + 1] < y0) y0 = v[2 * i + 1];
        if (v[2 * i + 1] > y1) y1 = v[2 * i + 1];
    }
    for (int y = y0; y <= y1; y++) {
        const float py = (float)y + 0.5f;
        for (int x = x0; x <= x1; x++) {
            const float px = (float)x + 0.5f;
            int in = 0;
            for (int i = 0, j = n - 1; i < n; j = i++) {
                const float xi = v[2 * i], yi = v[2 * i + 1], xj = v[2 * j], yj = v[2 * j + 1];
                if (((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
                    in = !in;
            }
            if (in)
                put_px(fb, x, y, level);
        }
    }
}

static void poly_outline(uint8_t *fb, const int16_t *v, int n, uint8_t level)
{
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        line(fb, v[2 * i], v[2 * i + 1], v[2 * j], v[2 * j + 1], level, 1);
    }
}

#define ARRAY_N(a)  ((int)(sizeof(a) / sizeof((a)[0])))

/* ------------------------------ данные карты ------------------------------- */
/* Экран 264x176. Все списки - {x, y, x, y, ...}. */

/* Маршрут - замкнутый контур (последняя точка соединяется с первой). */
typedef struct { int16_t x, y; } pt_t;
static const pt_t ROUTE[] = {
    { 24, 26}, {104, 26}, {104, 64}, {124, 64}, {160,100}, {236,100},
    {236,152}, {140,152}, {140,112}, { 84,112}, { 84, 84}, { 24, 84}
};
#define ROUTE_N     ARRAY_N(ROUTE)

/* Водоём (тёмно-серый) и парки (светло-серые). */
static const int16_t LAKE[]  = { 12,118, 30,108, 52,110, 68,122, 64,142, 44,156, 22,152, 10,136 };
static const int16_t PARK1[] = {176, 12, 226,  8, 252, 22, 248, 44, 214, 56, 184, 46 };
static const int16_t PARK2[] = {156,114, 222,112, 224,134, 196,142, 160,138 };
static const int16_t PARK3[] = {  6,  4,  44,  2,  52, 14,  30, 20,  10, 16 };

/* Дома (светло-серая заливка, чёрный контур). */
static const int16_t B1[]  = { 33, 35, 44, 34, 45, 49, 32, 50 };
static const int16_t B2[]  = { 56, 33, 70, 32, 71, 47, 57, 49 };
static const int16_t B3[]  = { 80, 34, 96, 35, 95, 48, 81, 47 };
static const int16_t B4[]  = { 33, 62, 45, 61, 46, 77, 32, 76 };
static const int16_t B5[]  = { 56, 62, 70, 63, 69, 76, 57, 77 };
static const int16_t B6[]  = { 92, 63, 98, 66, 96, 77, 90, 75 };
static const int16_t B7[]  = {112, 34, 134, 33, 136, 52, 114, 54 };
static const int16_t B8[]  = {142, 32, 166, 34, 164, 52, 144, 50 };
static const int16_t B9[]  = {112,  6, 128,  4, 130, 16, 114, 18 };
static const int16_t B10[] = {136,  5, 152,  6, 150, 17, 134, 16 };
static const int16_t B11[] = {172, 70, 192, 68, 194, 86, 174, 88 };
static const int16_t B12[] = {206, 68, 226, 70, 224, 86, 208, 88 };
static const int16_t B13[] = {240, 68, 258, 66, 260, 84, 242, 86 };
static const int16_t B14[] = {100,122, 118,120, 120,138, 102,140 };
static const int16_t B15[] = { 92,128,  96,128,  96,134,  92,134 };
static const int16_t B16[] = {240,114, 256,112, 258,134, 244,136 };
static const int16_t B17[] = {104,150, 118,152, 116,164, 102,162 };
static const int16_t B18[] = {  8, 66,  18, 64,  18, 76,   8, 78 };

typedef struct { const int16_t *v; uint8_t n; } poly_t;
#define POLY(a)     { (a), (uint8_t)(ARRAY_N(a) / 2) }
static const poly_t BUILDINGS[] = {
    POLY(B1), POLY(B2), POLY(B3), POLY(B4), POLY(B5), POLY(B6), POLY(B7), POLY(B8), POLY(B9),
    POLY(B10), POLY(B11), POLY(B12), POLY(B13), POLY(B14), POLY(B15), POLY(B16), POLY(B17), POLY(B18)
};

/* Улочки: светло-серые линии 1 px. */
static const int16_t ST_L1[] = {  0, 40,  24, 40 };
static const int16_t ST_L2[] = { 24, 55, 104, 55 };
static const int16_t ST_L3[] = { 48, 26,  48, 84 };
static const int16_t ST_L4[] = { 76, 26,  76, 84 };
static const int16_t ST_L5[] = {104, 45, 176, 45 };
static const int16_t ST_L6[] = {124, 20, 124, 64 };
static const int16_t ST_L7[] = {  0, 94,  84, 94 };
static const int16_t ST_L8[] = { 40, 84,  40, 108 };
static const int16_t ST_L9[] = {160, 20, 160,100 };
static const int16_t ST_L10[]= {176, 60, 176,100 };
static const int16_t ST_L11[]= {200, 60, 200,100 };
static const int16_t ST_L12[]= {236, 60, 236,100 };
static const int16_t ST_L13[]= {100,112, 100,152 };
static const int16_t ST_L14[]= { 84,126, 140,126 };
static const int16_t ST_L15[]= { 84,146, 140,146 };
static const int16_t ST_L16[]= {104,  0, 104, 26 };
static const int16_t ST_L17[]= {236,  0, 236, 12 };
static const int16_t ST_L18[]= {  0,170, 260,170 };

/* Дороги второго класса: тёмно-серые линии 2 px. */
static const int16_t RD_A[] = {  0, 60,  70, 60 };
static const int16_t RD_B[] = {104, 64,  60,  0 };
static const int16_t RD_C[] = {160, 100, 160, 100 };
static const int16_t RD_D[] = {  0, 94,   0, 94 };
static const int16_t RD_E[] = {164, 64, 200, 60, 264, 60 };
static const int16_t RD_F[] = {196, 100, 196, 176 };
static const int16_t RD_G[] = {160,  0, 164, 64 };
static const int16_t RD_H[] = {  0,106,  52,106,  84,126 };
static const int16_t RD_I[] = {236, 152, 264, 130 };
static const int16_t RD_J[] = { 24,  84,   0, 120 };

/* Железная дорога: чёрная линия со шпалами. */
static const int16_t RAIL[] = {  0,165, 130,165, 170,158, 264,158 };

/* ------------------------------- карта ------------------------------------- */

#define ROUTE_CASE_W   7                    /* чёрная кромка маршрута (толщина) */
#define ROUTE_FILL_W   5                    /* светло-серая заливка внутри кромки */

static void draw_rail(uint8_t *fb, const int16_t *v, int n)
{
    polyline(fb, v, n, EPD4G_BLACK, 1);
    for (int i = 0; i + 1 < n; i++) {
        const int ax = v[2 * i], ay = v[2 * i + 1], bx = v[2 * i + 2], by = v[2 * i + 3];
        const float len = vec_len(bx - ax, by - ay);
        const float ux = (bx - ax) / len, uy = (by - ay) / len;
        for (float t = 3.0f; t < len; t += 7.0f) {
            const float cx = ax + ux * t, cy = ay + uy * t;
            line(fb, (int)(cx + uy * 3.0f + 0.5f), (int)(cy - ux * 3.0f + 0.5f),
                     (int)(cx - uy * 3.0f + 0.5f), (int)(cy + ux * 3.0f + 0.5f), EPD4G_BLACK, 1);
        }
    }
}

static void render_map(uint8_t *fb)
{
    epd4g_fb_fill(fb, EPD4G_WHITE);

    /* площади */
    poly_fill(fb, PARK1, ARRAY_N(PARK1) / 2, EPD4G_LIGHT);
    poly_outline(fb, PARK1, ARRAY_N(PARK1) / 2, EPD4G_DARK);
    poly_fill(fb, PARK2, ARRAY_N(PARK2) / 2, EPD4G_LIGHT);
    poly_outline(fb, PARK2, ARRAY_N(PARK2) / 2, EPD4G_DARK);
    poly_fill(fb, PARK3, ARRAY_N(PARK3) / 2, EPD4G_LIGHT);
    poly_outline(fb, PARK3, ARRAY_N(PARK3) / 2, EPD4G_DARK);
    poly_fill(fb, LAKE, ARRAY_N(LAKE) / 2, EPD4G_DARK);
    poly_outline(fb, LAKE, ARRAY_N(LAKE) / 2, EPD4G_BLACK);

    /* улочки (светло-серые) */
    polyline(fb, ST_L1, 2, EPD4G_LIGHT, 1);   polyline(fb, ST_L2, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L3, 2, EPD4G_LIGHT, 1);   polyline(fb, ST_L4, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L5, 2, EPD4G_LIGHT, 1);   polyline(fb, ST_L6, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L7, 2, EPD4G_LIGHT, 1);   polyline(fb, ST_L8, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L9, 2, EPD4G_LIGHT, 1);   polyline(fb, ST_L10, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L11, 2, EPD4G_LIGHT, 1);  polyline(fb, ST_L12, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L13, 2, EPD4G_LIGHT, 1);  polyline(fb, ST_L14, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L15, 2, EPD4G_LIGHT, 1);  polyline(fb, ST_L16, 2, EPD4G_LIGHT, 1);
    polyline(fb, ST_L17, 2, EPD4G_LIGHT, 1);  polyline(fb, ST_L18, 2, EPD4G_LIGHT, 1);

    /* дороги второго класса (тёмно-серые) */
    polyline(fb, RD_A, 2, EPD4G_DARK, 2);
    polyline(fb, RD_B, 2, EPD4G_DARK, 2);
    polyline(fb, RD_E, 3, EPD4G_DARK, 2);
    polyline(fb, RD_F, 2, EPD4G_DARK, 2);
    polyline(fb, RD_G, 2, EPD4G_DARK, 2);
    polyline(fb, RD_H, 3, EPD4G_DARK, 2);
    polyline(fb, RD_I, 2, EPD4G_DARK, 2);
    polyline(fb, RD_J, 2, EPD4G_DARK, 2);
    (void)RD_C; (void)RD_D;

    /* железная дорога */
    draw_rail(fb, RAIL, ARRAY_N(RAIL) / 2);

    /* дома */
    for (int i = 0; i < ARRAY_N(BUILDINGS); i++) {
        poly_fill(fb, BUILDINGS[i].v, BUILDINGS[i].n, EPD4G_LIGHT);
        poly_outline(fb, BUILDINGS[i].v, BUILDINGS[i].n, EPD4G_BLACK);
    }

    /* маршрут: сначала вся чёрная кромка, потом светло-серая заливка внутри */
    for (int i = 0; i < (int)ROUTE_N; i++) {
        const int j = (i + 1) % ROUTE_N;
        line(fb, ROUTE[i].x, ROUTE[i].y, ROUTE[j].x, ROUTE[j].y, EPD4G_BLACK, ROUTE_CASE_W);
    }
    for (int i = 0; i < (int)ROUTE_N; i++) {
        const int j = (i + 1) % ROUTE_N;
        line(fb, ROUTE[i].x, ROUTE[i].y, ROUTE[j].x, ROUTE[j].y, EPD4G_LIGHT, ROUTE_FILL_W);
    }
}

/* ------------------------------ движение по маршруту ----------------------- */

static float rt_cum[ROUTE_N + 1];           /* пройденный путь до начала сегмента i */
static float rt_ux[ROUTE_N], rt_uy[ROUTE_N];/* единичный вектор сегмента i */
static float s_pos;                         /* пройденный путь по маршруту, px */
static int   s_cx, s_cy;                    /* центр маркёра на экране */
static float s_hx = 1.0f, s_hy;             /* направление маркёра */

static void route_init(void)
{
    rt_cum[0] = 0.0f;
    for (int i = 0; i < (int)ROUTE_N; i++) {
        const int j = (i + 1) % ROUTE_N;
        const int dx = ROUTE[j].x - ROUTE[i].x, dy = ROUTE[j].y - ROUTE[i].y;
        const float len = vec_len(dx, dy);
        rt_ux[i] = dx / len;
        rt_uy[i] = dy / len;
        rt_cum[i + 1] = rt_cum[i] + len;
    }
}

/* Точка маршрута на расстоянии s от начала: центр (округлён до пикселя) и направление. */
static void route_at(float s, int *x, int *y, float *ux, float *uy)
{
    int i = 0;
    while (i < (int)ROUTE_N - 1 && s >= rt_cum[i + 1])
        i++;
    const float t = s - rt_cum[i];
    *x = (int)(ROUTE[i].x + rt_ux[i] * t + 0.5f);
    *y = (int)(ROUTE[i].y + rt_uy[i] * t + 0.5f);
    *ux = rt_ux[i];
    *uy = rt_uy[i];
}

/* Дорисовывает в base чёрный трек от пути a до пути b (шаг 1 px, чтобы повторять изгибы). */
static void track_draw(uint8_t *fb, float a, float b)
{
    int px, py, x, y;
    float ux, uy;

    route_at(a, &px, &py, &ux, &uy);
    for (float s = a + 1.0f; ; s += 1.0f) {
        if (s > b) s = b;
        route_at(s, &x, &y, &ux, &uy);
        line(fb, px, py, x, y, EPD4G_BLACK, EINK_TRACK_W);
        px = x; py = y;
        if (s >= b)
            break;
    }
}

/* ------------------------------- маркёр ------------------------------------ */
/* Стрелка навигатора. В системе (f - вперёд, q - вбок): остриё (7.5, 0), крылья (-5.5, +-5),
 * вырез сзади (-2.5, 0). Чёрная, с белой обводкой ~1.3 px, чтобы читалась на чёрном треке. */

static float edge_fn(float ax, float ay, float bx, float by, float px, float py)
{
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

static int in_tri(float px, float py, float ax, float ay, float bx, float by, float cx, float cy)
{
    const float d1 = edge_fn(ax, ay, bx, by, px, py);
    const float d2 = edge_fn(bx, by, cx, cy, px, py);
    const float d3 = edge_fn(cx, cy, ax, ay, px, py);
    return !((d1 < 0 || d2 < 0 || d3 < 0) && (d1 > 0 || d2 > 0 || d3 > 0));
}

/* (px,py) - смещение от центра маркёра в пикселях экрана, (hx,hy) - единичное направление. */
static int in_arrow(float px, float py, float hx, float hy)
{
    const float f = px * hx + py * hy;
    const float q = -px * hy + py * hx;
    return in_tri(f, q,  7.5f,  0.0f, -5.5f,  5.0f, -2.5f, 0.0f) ||
           in_tri(f, q,  7.5f,  0.0f, -2.5f,  0.0f, -5.5f, -5.0f);
}

static void draw_marker(uint8_t *fb, int cx, int cy, float hx, float hy)
{
    static const float nb[8][2] = {
        { 1.3f, 0.0f}, {-1.3f, 0.0f}, {0.0f, 1.3f}, {0.0f, -1.3f},
        { 0.92f, 0.92f}, {-0.92f, 0.92f}, {0.92f, -0.92f}, {-0.92f, -0.92f}
    };

    for (int dy = -9; dy <= 9; dy++) {
        for (int dx = -9; dx <= 9; dx++) {
            const float px = (float)dx, py = (float)dy;
            if (in_arrow(px, py, hx, hy)) {
                put_px(fb, cx + dx, cy + dy, EPD4G_BLACK);
                continue;
            }
            for (int k = 0; k < 8; k++) {
                if (in_arrow(px + nb[k][0], py + nb[k][1], hx, hy)) {
                    put_px(fb, cx + dx, cy + dy, EPD4G_WHITE);      /* обводка */
                    break;
                }
            }
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
    epd4g_map_rect(EINK_ROT, cx - MK_R, cy - MK_R, cx + MK_R, cy + MK_R, &r);
    return r;
}

#if !EINK_POWER_TEST

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

/* Маркёр в начало маршрута, чистая карта без трека: cur = bg (маркёр дорисовывает вызывающий). */
static void restart_route(void)
{
    s_pos = 0.0f;
    memcpy(base, bg, sizeof base);
    memcpy(cur, bg, sizeof cur);
    route_at(s_pos, &s_cx, &s_cy, &s_hx, &s_hy);
}

int eink_demo_init(void)
{
    int rc;

    DIAG("[INIT] epd4g_init()\r\n");
    if ((rc = epd4g_init()) != EPD4G_OK) {
        DIAG("[ERROR][INIT] epd4g_init rc=%d\r\n", rc);
        return -1;
    }

    DIAG("[INIT] rendering %dx%d 4-gray map (%lu bytes)\r\n",
         epd4g_logical_width(EINK_ROT), epd4g_logical_height(EINK_ROT), (unsigned long)EPD4G_FB_BYTES);
    route_init();
    render_map(bg);
    restart_route();

    DIAG("[INIT] full 4-gray refresh (stock 262-frame waveform)\r\n");
    if ((rc = epd4g_full(cur, EPD4G_FULL_DEEP)) != EPD4G_OK) {
        DIAG("[ERROR][INIT] epd4g_full rc=%d\r\n", rc);
        return -1;
    }
    memcpy(prev, cur, sizeof prev);
    s_upd_cnt = 0;

    /* первый кадр с маркёром - уже частичным обновлением */
    epd4g_rect_t r = rect_around(s_cx, s_cy);
    draw_marker(cur, s_cx, s_cy, s_hx, s_hy);
    if (push_update(&r) != 0) {
        DIAG("[ERROR][INIT] initial marker update failed\r\n");
        return -1;
    }
    DIAG("[INIT] complete; route length %d px, marker=(%d,%d)\r\n", (int)rt_cum[ROUTE_N], s_cx, s_cy);
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

/* Один шаг маркёра по маршруту. Когда круг пройден - трек стирается и карта
 * перерисовывается полным обновлением (DEEP: заодно убирает остатки трека). */
static int start_gray_frame(void)
{
    const float total = rt_cum[ROUTE_N];
    const float s_new = s_pos + (float)EINK_STEP_PX;

    memcpy(prev, cur, sizeof prev);         /* то, что сейчас на панели */

    if (s_new >= total) {
        DIAG("[FRAME] loop done, clearing track\r\n");
        restart_route();
        draw_marker(cur, s_cx, s_cy, s_hx, s_hy);
        memcpy(prev, cur, sizeof prev);     /* после полного обновления панель = cur */
        s_upd_cnt = 0;
        return job_started(epd4g_start_full(cur, EPD4G_FULL_DEEP));
    }

    const int ox = s_cx, oy = s_cy;
    track_draw(base, s_pos, s_new);         /* трек - в base, он постоянный */
    s_pos = s_new;
    route_at(s_pos, &s_cx, &s_cy, &s_hx, &s_hy);

    epd4g_rect_t r = rect_union(rect_around(ox, oy), rect_around(s_cx, s_cy));
    DIAG("[FRAME] marker (%d,%d)->(%d,%d), s=%d\r\n", ox, oy, s_cx, s_cy, (int)s_pos);
    epd4g_fb_copy_rect(cur, base, &r);      /* стереть старый маркёр, показать новый кусок трека */
    draw_marker(cur, s_cx, s_cy, s_hx, s_hy);
    return start_gray_update(&r);           /* cur и prev не трогаем, пока задача не закончится */
}

static int start_next(void)
{
    switch (s_act) {
    case A_GRAY_FRAME:
#if EINK_BW_DEMO
        if (s_frames >= EINK_BW_EVERY) {
            DIAG("[MODE] 4 gray -> 1 bit\r\n");
            s_frames = 0; s_bw_steps = 0; s_sel = 0;
            s_act = A_BW_FULL;
            return job_started(epd4g_start_bw_init(EPD4G_BW_NORMAL));
        }
#endif
        s_frames++;
        return start_gray_frame();
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

uint32_t eink_demo_wait_ms(int *on_busy)
{
    if (on_busy != NULL)
        *on_busy = 0;
    if (!epd4g_is_busy())                   /* задачи нет: следующий eink_demo_task() запустит новую */
        return 0;
    return epd4g_wait_hint(on_busy);        /* идущая задача не вернёт EPD4G_WAIT_FOREVER */
}

uint32_t eink_demo_last_update_ms(void)
{
    return s_last_ms;
}

#endif /* !EINK_POWER_TEST */
