/*
 * eink_demo.c — тест частичного обновления Waveshare 2.7" V2 (SSD1680-подобный контроллер)
 * в режиме 4 gray.
 *
 * Что делает:
 *   1. Init_4GRAY из библиотеки Waveshare (она загружает 4-gray LUT).
 *   2. Рисует фон: сетка квадратов 22x22 px, 4 полутона (12 x 8 клеток на 264x176).
 *   3. Каждый шаг двигает чёрный треугольник (10 px) с белой обводкой и обновляет
 *      только "грязный" прямоугольник: пишет окно в RAM 0x24 и 0x26 и запускает update.
 *
 * Экран логически альбомный 264x176, физически панель портретная 176x264 (x - ширина 176).
 * Плоскость A -> RAM 0x24, плоскость B -> RAM 0x26, по 1 биту на пиксель.
 */

#include "eink_demo.h"
#include "DEV_Config.h"
#include "EPD_2in7_V2.h"
#include <string.h>

/* ------------------------- настройки эксперимента ------------------------- */

/* Значение для регистра 0x22 (Display Update Control 2). 0xC7 как в Waveshare 4GRAY.
 * Для экспериментов: 0xCF, 0xFF, 0xF7 и т.д. */
#ifndef EINK_UPDATE_CTRL2
#define EINK_UPDATE_CTRL2   0xC7
#endif

/* 1 - всегда слать весь кадр в RAM (для сравнения), 0 - только окно с изменениями */
#ifndef EINK_SEND_FULL_FRAME
#define EINK_SEND_FULL_FRAME 0
#endif

/* Поворот логического экрана на 180°, если картинка перевёрнута */
#ifndef EINK_ROTATE_180
#define EINK_ROTATE_180     0
#endif

/* Шаг треугольника за одно обновление (px). Скорость ограничена временем update. */
#ifndef EINK_STEP_X
#define EINK_STEP_X         4
#endif
#ifndef EINK_STEP_Y
#define EINK_STEP_Y         3
#endif

/* Кодирование уровней (0 = чёрный, 1 = тёмно-серый, 2 = светло-серый, 3 = белый)
 * в биты RAM 0x24 / 0x26. По документации Waveshare для 4 gray:
 *   белый = (1,1), чёрный = (0,0), серые = (1,0) и (0,1).
 * Если на экране тёмный и светлый серый поменялись местами - поменяйте строки местами. */
static const uint8_t RAM24_BIT[4] = {0, 1, 0, 1};
static const uint8_t RAM26_BIT[4] = {0, 0, 1, 1};

/* ------------------------------- геометрия -------------------------------- */

#define NAT_W       176                     /* ширина панели (портрет) */
#define NAT_H       264
#define ROW_BYTES   (NAT_W / 8)             /* 22 */
#define PLANE_SZ    (ROW_BYTES * NAT_H)     /* 5808 */

#define LW          264                     /* логическая ширина (альбом) */
#define LH          176

#define CELL        22                      /* размер квадрата фона: 264/22 = 12, 176/22 = 8 */
#define TRI_R       10                      /* запас вокруг центра треугольника для dirty rect */
#define BUSY_TIMEOUT_MS 10000

typedef struct { int bx0, bx1, y0, y1; } nrect_t;   /* bx - в байтах (8 px), y - строки */

extern SPI_HandleTypeDef hspi1;

static uint8_t bg_a[PLANE_SZ], bg_b[PLANE_SZ];      /* фон */
static uint8_t fr_a[PLANE_SZ], fr_b[PLANE_SZ];      /* текущий кадр (фон + треугольник) */

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

static void put_logical(uint8_t *a, uint8_t *b, int lx, int ly, uint8_t lvl)
{
    if (lx < 0 || ly < 0 || lx >= LW || ly >= LH) return;
    int nx, ny;
    l2n(lx, ly, &nx, &ny);
    uint32_t idx = (uint32_t)ny * ROW_BYTES + (nx >> 3);
    uint8_t  m   = (uint8_t)(0x80 >> (nx & 7));
    if (RAM24_BIT[lvl]) a[idx] |= m; else a[idx] &= (uint8_t)~m;
    if (RAM26_BIT[lvl]) b[idx] |= m; else b[idx] &= (uint8_t)~m;
}

/* -------------------------------- рисование ------------------------------- */

static void render_background(void)
{
    for (int ly = 0; ly < LH; ly++) {
        for (int lx = 0; lx < LW; lx++) {
            uint8_t lvl = (uint8_t)(((lx / CELL) + (ly / CELL)) & 3);
            put_logical(bg_a, bg_b, lx, ly, lvl);
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
            if (d >= 0.0f)           put_logical(fr_a, fr_b, cx + dx, cy + dy, 0);  /* чёрный */
            else if (d >= -OUTLINE)  put_logical(fr_a, fr_b, cx + dx, cy + dy, 3);  /* белая обводка */
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
    r.bx0 = imin(ax, bx) >> 3;
    r.bx1 = imax(ax, bx) >> 3;
    r.y0  = imin(ay, by);
    r.y1  = imax(ay, by);
    return r;
}

static nrect_t rect_union(nrect_t a, nrect_t b)
{
    nrect_t r;
    r.bx0 = imin(a.bx0, b.bx0); r.bx1 = imax(a.bx1, b.bx1);
    r.y0  = imin(a.y0, b.y0);   r.y1  = imax(a.y1, b.y1);
    return r;
}

static nrect_t rect_around(int cx, int cy)
{
    return rect_logical(cx - TRI_R, cy - TRI_R, cx + TRI_R, cy + TRI_R);
}

static void restore_from_bg(const nrect_t *r)
{
    int n = r->bx1 - r->bx0 + 1;
    for (int y = r->y0; y <= r->y1; y++) {
        uint32_t o = (uint32_t)y * ROW_BYTES + r->bx0;
        memcpy(&fr_a[o], &bg_a[o], (size_t)n);
        memcpy(&fr_b[o], &bg_b[o], (size_t)n);
    }
}

/* ------------------------------ низкий уровень ---------------------------- */

static void epd_cmd(uint8_t c)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(c);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void epd_data1(uint8_t d)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(d);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void epd_wait_busy(void)
{
    uint32_t t0 = HAL_GetTick();
    while (DEV_Digital_Read(EPD_BUSY_PIN)) {            /* BUSY = 1 - занят */
        if (HAL_GetTick() - t0 > BUSY_TIMEOUT_MS) break;
        DEV_Delay_ms(1);
    }
}

static void epd_set_window(const nrect_t *r)
{
    epd_cmd(0x44);                                      /* RAM X start/end, в байтах */
    epd_data1((uint8_t)r->bx0);
    epd_data1((uint8_t)r->bx1);
    epd_cmd(0x45);                                      /* RAM Y start/end */
    epd_data1((uint8_t)(r->y0 & 0xFF));
    epd_data1((uint8_t)((r->y0 >> 8) & 0x01));
    epd_data1((uint8_t)(r->y1 & 0xFF));
    epd_data1((uint8_t)((r->y1 >> 8) & 0x01));
}

static void epd_set_cursor(const nrect_t *r)
{
    epd_cmd(0x4E);
    epd_data1((uint8_t)r->bx0);
    epd_cmd(0x4F);
    epd_data1((uint8_t)(r->y0 & 0xFF));
    epd_data1((uint8_t)((r->y0 >> 8) & 0x01));
}

static void epd_write_plane(uint8_t cmd, const uint8_t *plane, const nrect_t *r)
{
    uint16_t n = (uint16_t)(r->bx1 - r->bx0 + 1);
    epd_set_cursor(r);              /* счётчик адреса надо ставить перед каждой плоскостью */
    epd_cmd(cmd);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    for (int y = r->y0; y <= r->y1; y++) {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)&plane[(uint32_t)y * ROW_BYTES + r->bx0], n, 100);
    }
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

static void push_window(nrect_t r)
{
    if (EINK_SEND_FULL_FRAME) {
        r.bx0 = 0; r.bx1 = ROW_BYTES - 1; r.y0 = 0; r.y1 = NAT_H - 1;
    }
    uint32_t t0 = HAL_GetTick();

    epd_set_window(&r);
    epd_write_plane(0x24, fr_a, &r);
    epd_write_plane(0x26, fr_b, &r);

    epd_cmd(0x22);
    epd_data1(EINK_UPDATE_CTRL2);
    epd_cmd(0x20);
    epd_wait_busy();

    s_last_ms = HAL_GetTick() - t0;
}

/* --------------------------------- API ------------------------------------ */

void eink_demo_init(void)
{
    DEV_Module_Init();
    EPD_2IN7_V2_Init_4GRAY();               /* сброс + загрузка 4-gray LUT (Waveshare) */

    render_background();
    memcpy(fr_a, bg_a, PLANE_SZ);
    memcpy(fr_b, bg_b, PLANE_SZ);

    nrect_t full = { 0, ROW_BYTES - 1, 0, NAT_H - 1 };
    push_window(full);                      /* полная 4-gray отрисовка фона */

    /* первый кадр с треугольником */
    nrect_t r = rect_around(s_x, s_y);
    draw_triangle(s_x, s_y);
    push_window(r);
}

void eink_demo_step(void)
{
    int ox = s_x, oy = s_y;

    s_x += s_dx;
    s_y += s_dy;
    if (s_x < 14 || s_x > LW - 15) { s_dx = -s_dx; s_x += 2 * s_dx; }
    if (s_y < 14 || s_y > LH - 15) { s_dy = -s_dy; s_y += 2 * s_dy; }

    nrect_t r = rect_union(rect_around(ox, oy), rect_around(s_x, s_y));
    restore_from_bg(&r);
    draw_triangle(s_x, s_y);
    push_window(r);
}

uint32_t eink_demo_last_update_ms(void)
{
    return s_last_ms;
}
