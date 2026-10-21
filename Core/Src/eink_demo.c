/*
 * eink_demo.c — тест частичного обновления Waveshare 2.7" V2 в режиме 4 gray
 * БЕЗ мигания всего экрана.
 *
 * Что делает:
 *   1. EPD_2IN7_V2_Init_4GRAY() + EPD_2IN7_V2_4GrayDisplay(): один раз рисует фон
 *      (квадраты 22x22 px, 4 полутона) штатным полным обновлением Waveshare.
 *   2. Каждый шаг двигает чёрный треугольник (10 px) с белой обводкой, строит маску
 *      ТОЛЬКО из пикселей, которые реально изменились, и вызывает
 *      EPD_2IN7_V2_4Gray_PartialMask() (Core/Src/EPD_2in7_V2_4gray_partial.c):
 *      два прохода с собственным LUT, у пикселей вне маски нулевое напряжение.
 *
 * Буфер кадра - в штатном формате Waveshare для 4 gray: 2 бита на пиксель,
 * 4 пикселя в байте, старший пиксель в старших битах, 44 байта на строку,
 * 11616 байт. Коды: 11 = белый, 10 = светло-серый, 01 = тёмно-серый, 00 = чёрный.
 *
 * Экран логически альбомный 264x176, физически панель портретная 176x264.
 */

#include "eink_demo.h"
#include "DEV_Config.h"
#include "EPD_2in7_V2.h"
#include "EPD_2in7_V2_4gray_partial.h"
#include <string.h>
#include <stdio.h>

/* ------------------------- настройки эксперимента ------------------------- */

/* Поворот логического экрана на 180°, если картинка перевёрнута */
#ifndef EINK_ROTATE_180
#define EINK_ROTATE_180     0
#endif

/* 0 - в маску попадают только изменившиеся пиксели (меньше всего трогаем экран),
 * 1 - вся прямоугольная область вокруг старого и нового положения (для сравнения;
 *     если на границе треугольника остаются "шлейфы", попробуйте 1) */
#ifndef EINK_MASK_FULL_RECT
#define EINK_MASK_FULL_RECT 0
#endif

/* Шаг треугольника за одно обновление (px). Скорость ограничена временем update. */
#ifndef EINK_STEP_X
#define EINK_STEP_X         4
#endif
#ifndef EINK_STEP_Y
#define EINK_STEP_Y         3
#endif

/* Уровни демо -> 2-битный код пикселя Waveshare.
 * 0 = чёрный, 3 = белый. Если два серых на экране идут не в том порядке
 * (светлый/тёмный) - поменяйте местами значения 1 и 2. */
static const uint8_t LVL_CODE[4] = {0, 1, 2, 3};

/* ------------------------------- геометрия -------------------------------- */

#define NAT_W       EPD_2IN7_V2_WIDTH       /* 176, ширина панели (портрет) */
#define NAT_H       EPD_2IN7_V2_HEIGHT      /* 264 */
#define IMG_STRIDE  (NAT_W / 4)             /* 44 байта на строку */
#define IMG_SZ      (IMG_STRIDE * NAT_H)    /* 11616 */

#define LW          264                     /* логическая ширина (альбом) */
#define LH          176
#define CELL        22                      /* 264/22 = 12, 176/22 = 8 */
#define TRI_R       10                      /* запас вокруг центра треугольника */

typedef struct { int x0, x1, y0, y1; } nrect_t;   /* в пикселях панели, включительно */

static uint8_t bg[IMG_SZ];                  /* фон */
static uint8_t cur[IMG_SZ];                 /* текущий кадр (фон + треугольник) */
static uint8_t prev[IMG_SZ];                /* кадр, который сейчас на панели */
static uint8_t mask[EPD_4GRAY_MASK_BYTES];  /* 1 бит/пиксель, 22 байта на строку */

static int s_x = 60, s_y = 50, s_dx = EINK_STEP_X, s_dy = EINK_STEP_Y;
static uint32_t s_last_ms;

/* ------------------------------ пиксели ----------------------------------- */

static void l2n(int lx, int ly, int *nx, int *ny)
{
#if EINK_ROTATE_180
    *nx = ly;
    *ny = LW - 1 - lx;
#else
    *nx = LH - 1 - ly;
    *ny = lx;
#endif
}

static uint8_t get_native(const uint8_t *img, int nx, int ny)
{
    return (uint8_t)((img[(uint32_t)ny * IMG_STRIDE + (nx >> 2)] >> (6 - 2 * (nx & 3))) & 3);
}

static void put_logical(uint8_t *img, int lx, int ly, uint8_t lvl)
{
    if (lx < 0 || ly < 0 || lx >= LW || ly >= LH) return;
    int nx, ny;
    l2n(lx, ly, &nx, &ny);
    uint32_t idx = (uint32_t)ny * IMG_STRIDE + (nx >> 2);
    int sh = 6 - 2 * (nx & 3);
    img[idx] = (uint8_t)((img[idx] & ~(3 << sh)) | (LVL_CODE[lvl] << sh));
}

/* -------------------------------- рисование ------------------------------- */

static void render_background(void)
{
    for (int ly = 0; ly < LH; ly++) {
        for (int lx = 0; lx < LW; lx++) {
            uint8_t lvl = (uint8_t)(((lx / CELL) + (ly / CELL)) & 3);
            put_logical(bg, lx, ly, lvl);
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
            if (d >= 0.0f)           put_logical(cur, cx + dx, cy + dy, 0);  /* чёрный */
            else if (d >= -OUTLINE)  put_logical(cur, cx + dx, cy + dy, 3);  /* белая обводка */
        }
    }
}

/* ------------------------- прямоугольники обновления ---------------------- */

static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }

static nrect_t rect_logical(int lx0, int ly0, int lx1, int ly1)
{
    lx0 = imax(lx0, 0); ly0 = imax(ly0, 0);
    lx1 = imin(lx1, LW - 1); ly1 = imin(ly1, LH - 1);
    int ax, ay, bx, by;
    l2n(lx0, ly0, &ax, &ay);
    l2n(lx1, ly1, &bx, &by);
    nrect_t r;
    r.x0 = imin(ax, bx); r.x1 = imax(ax, bx);
    r.y0 = imin(ay, by); r.y1 = imax(ay, by);
    return r;
}

static nrect_t rect_union(nrect_t a, nrect_t b)
{
    nrect_t r;
    r.x0 = imin(a.x0, b.x0); r.x1 = imax(a.x1, b.x1);
    r.y0 = imin(a.y0, b.y0); r.y1 = imax(a.y1, b.y1);
    return r;
}

static nrect_t rect_around(int cx, int cy)
{
    return rect_logical(cx - TRI_R, cy - TRI_R, cx + TRI_R, cy + TRI_R);
}

/* Копирует фон в прямоугольник (с округлением до байта по X: пиксели вне rect
 * и так равны фону, потому что треугольник целиком внутри rect). */
static void restore_from_bg(const nrect_t *r)
{
    int xb0 = r->x0 >> 2, xb1 = r->x1 >> 2;
    for (int y = r->y0; y <= r->y1; y++) {
        uint32_t o = (uint32_t)y * IMG_STRIDE + (uint32_t)xb0;
        memcpy(&cur[o], &bg[o], (size_t)(xb1 - xb0 + 1));
    }
}

/* ------------------------------ обновление -------------------------------- */

/* Строит маску по отличиям cur от prev внутри rect и обновляет панель. */
static int push_diff(const nrect_t *r)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t changed = 0;

    memset(mask, 0, sizeof mask);
    for (int y = r->y0; y <= r->y1; y++) {
        for (int x = r->x0; x <= r->x1; x++) {
#if EINK_MASK_FULL_RECT
            int diff = 1;
#else
            int diff = get_native(cur, x, y) != get_native(prev, x, y);
#endif
            if (diff) {
                EPD_4GRAY_MASK_SET(mask, x, y);
                changed++;
            }
        }
    }
    printf("[UPDATE] rect X=%d..%d Y=%d..%d, pixels in mask=%lu, mode=%s\r\n",
           r->x0, r->x1, r->y0, r->y1, (unsigned long)changed,
           EINK_MASK_FULL_RECT ? "full-rect" : "changed-only");

    if (changed == 0) {
        s_last_ms = 0;
        return 0;
    }
    if (EPD_2IN7_V2_4Gray_PartialMask(cur, mask) != 0) {
        s_last_ms = HAL_GetTick() - t0;
        printf("[ERROR][UPDATE] BUSY timeout after %lu ms\r\n", (unsigned long)s_last_ms);
        return -1;
    }
    s_last_ms = HAL_GetTick() - t0;
    printf("[UPDATE] complete in %lu ms\r\n", (unsigned long)s_last_ms);
    return 0;
}

/* --------------------------------- API ------------------------------------ */

int eink_demo_init(void)
{
    printf("[INIT] DEV_Module_Init()\r\n");
    DEV_Module_Init();
    printf("[INIT] EPD_2IN7_V2_Init_4GRAY(); BUSY=%u\r\n",
           (unsigned)DEV_Digital_Read(EPD_BUSY_PIN));
    EPD_2IN7_V2_Init_4GRAY();               /* сброс + загрузка 4-gray LUT (Waveshare) */

    printf("[INIT] rendering %dx%d 4-gray background (%lu bytes)\r\n",
           LW, LH, (unsigned long)IMG_SZ);
    render_background();
    memcpy(cur, bg, IMG_SZ);

    printf("[INIT] full 4-gray refresh (stock Waveshare)\r\n");
    EPD_2IN7_V2_4GrayDisplay(cur);          /* единственное полное обновление */
    memcpy(prev, cur, IMG_SZ);

    /* первый кадр с треугольником - уже частичным обновлением */
    nrect_t r = rect_around(s_x, s_y);
    draw_triangle(s_x, s_y);
    if (push_diff(&r) != 0) {
        printf("[ERROR][INIT] initial triangle update failed\r\n");
        return -1;
    }
    printf("[INIT] complete; initial triangle=(%d,%d)\r\n", s_x, s_y);
    return 0;
}

int eink_demo_step(void)
{
    int ox = s_x, oy = s_y;

    memcpy(prev, cur, IMG_SZ);              /* то, что сейчас на панели */

    s_x += s_dx;
    s_y += s_dy;
    if (s_x < 14 || s_x > LW - 15) { s_dx = -s_dx; s_x += 2 * s_dx; }
    if (s_y < 14 || s_y > LH - 15) { s_dy = -s_dy; s_y += 2 * s_dy; }

    nrect_t r = rect_union(rect_around(ox, oy), rect_around(s_x, s_y));
    printf("[FRAME] triangle (%d,%d)->(%d,%d)\r\n", ox, oy, s_x, s_y);
    restore_from_bg(&r);
    draw_triangle(s_x, s_y);
    if (push_diff(&r) != 0) {
        printf("[ERROR][FRAME] frame update failed; aborting animation\r\n");
        return -1;
    }
    return 0;
}

uint32_t eink_demo_last_update_ms(void)
{
    return s_last_ms;
}
