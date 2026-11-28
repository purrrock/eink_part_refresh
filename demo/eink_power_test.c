/*
 * eink_power_test.c - замер потребляемого тока мультиметром во всех режимах (STM32U585).
 *
 * Берёт рисование карты, маркёра и трека из eink_demo.c (подключается целиком) и вместо демо-автомата
 * прогоняет последовательность ШАГОВ. Каждый шаг - одна комбинация режимов; на экране вместе с
 * картинкой написано, что проверяется, тот же текст идёт в UART. Измеряйте ток, пока шаг идёт.
 *
 * Группы шагов:
 *   STATIC  неподвижная картинка (карта 4 gray + большая подпись), процессор остановлен/крутится.
 *           Перебор: панель {аналог ВКЛ, аналог ВЫКЛ, deep sleep} x линии {ACTIVE, HIGH, LOW, ANALOG}
 *           x процессор {RUN, SLEEP, STOP2}. Шаг: полное обновление с подписью, затем PWR_DWELL_STATIC_S с.
 *   DYN-4G  маркёр едет по карте, 4 gray, частичные обновления подряд PWR_DWELL_DYN_S секунд.
 *           Перебор: аналог {KEEP - остаётся включённым, CYCLE - выключается после каждого обновления}
 *           x линии в паузах {ACTIVE, HIGH, ANALOG} x процессор в паузах {RUN, SLEEP, STOP2}.
 *   DYN-1B  листание 1-bit меню, частичные обновления подряд. Перебор: линии x процессор.
 *
 * «Линии в паузах»: пока панель рисует волну (0,3..2,5 с), SPI не нужен. ACTIVE - SPI1 включён, SCK
 * стоит в 0 (как сейчас). HIGH - SPI1 выключен, SCK/MOSI/DC = 1. ANALOG - SPI1 выключен, SCK/MOSI/DC
 * в analog (CS и RST остаются выходами: библиотека держит RST низким в импульсе сброса).
 *
 *   HAT-PWR (PWR_GROUPS & 8 и & 16) мини-тесты «что обесточивает HAT»: панель в power off, Stop 2,
 *           режим каждой линии задаёт таблица HAT_VAR (группа 8: одна линия в analog/0, остальные в 1;
 *           группа 16: RST в 0 и разные комбинации остальных). После замера
 *           8 с на осмотр экрана (сохранилась ли картинка), затем время повторного init и полного обновления.
 *
 *   WAKE    (PWR_GROUPS & 32) частичное обновление маркёра после сна HAT и порог окупаемости сна. Каждый шаг:
 *           полное обновление, 3 частичных (маркёр едет), power off, простой PWR_WAKE_IDLE_S секунд, затем
 *           следующий кадр: STAY - HAT не спал (эталон), SLP FULL - после сна init + полное обновление,
 *           SLP PART - после сна init и сразу частичное обновление без полного (проверка, можно ли так).
 *           Сравнивайте заряд от конца простоя до конца обновления (интеграл тока) - см. Docs/POWER_TEST.md.
 *   WAKE1B  (PWR_GROUPS & 64) то же для 1-bit меню: после сна HAT epd4g_bw_restore() (RAM контроллера, без
 *           вспышки) и обычные частичные обновления. Шаги STAY 1B (эталон) и SLP 1B.
 *
 * Параметры (-D... в CMake): PWR_DWELL_STATIC_S, PWR_DWELL_DYN_S, PWR_ANALOG_ON_MAX_S, PWR_GROUPS (маска:
 * 1 STATIC, 2 DYN-4G, 4 DYN-1B, 8 и 16 HAT-PWR, 32 WAKE, 64 WAKE1B), PWR_FIRST_STEP, PWR_LAST_STEP, PWR_TEST_BW_KEEP.
 */
#define EINK_POWER_TEST 1
#include "eink_demo.c"

#include "eink_pwr.h"
#include "eink_sleep.h"
#include "font5x7.h"
#include <stdio.h>

#ifndef PWR_DWELL_STATIC_S
#define PWR_DWELL_STATIC_S   15         /* сколько секунд держать статический режим для замера */
#endif
#ifndef PWR_DWELL_DYN_S
#define PWR_DWELL_DYN_S      30         /* сколько секунд гонять анимацию для замера */
#endif
#ifndef PWR_ANALOG_ON_MAX_S
#define PWR_ANALOG_ON_MAX_S  20         /* аналог панели долго включённым держать нельзя (даташит) */
#endif
#ifndef PWR_GROUPS
#define PWR_GROUPS           7
#endif
#ifndef PWR_FIRST_STEP
#define PWR_FIRST_STEP       1
#endif
#ifndef PWR_LAST_STEP
#define PWR_LAST_STEP        999
#endif
#ifndef PWR_WAKE_IDLE_S
#define PWR_WAKE_IDLE_S      10         /* группа WAKE: длительность простоя между обновлениями */
#endif
#ifndef PWR_TEST_BW_KEEP
#define PWR_TEST_BW_KEEP     0
#endif

volatile unsigned char g_pwr_keep_gray = 1;     /* см. pwr_epd4g_cfg.h */

/* ------------------------------ список шагов ------------------------------- */

enum { GRP_STATIC = 0, GRP_DYN4G, GRP_DYNBW, GRP_HAT, GRP_WAKE, GRP_WAKE1B };
enum { PANEL_ON = 0, PANEL_OFF, PANEL_SLEEP };  /* STATIC: состояние панели */
                                                /* DYN-4G: panel = 1 KEEP, 0 CYCLE */
typedef struct { uint8_t grp, panel, lines, cpu; } step_t;

#define MAX_STEPS   80
static step_t s_steps[MAX_STEPS];
static int    s_n, s_idx;
static int    s_total_all;                      /* число шагов до отбора по FIRST/LAST */

static const uint8_t LINES_STATIC[] = { PWR_LINES_ACTIVE, PWR_LINES_HIGH, PWR_LINES_LOW, PWR_LINES_ANALOG };
static const uint8_t LINES_DYN[]    = { PWR_LINES_ACTIVE, PWR_LINES_HIGH, PWR_LINES_ANALOG };
static const uint8_t CPUS[]         = { PWR_CPU_RUN, PWR_CPU_SLEEP, PWR_CPU_STOP2 };

static const char *const CPU_NAME[]   = { "RUN", "SLEEP", "STOP2" };
static const char *const LINES_NAME[] = { "ACTIVE", "LOW", "HIGH", "ANALOG" };      /* индекс = pwr_lines_t */
static const char *const PANEL_NAME[] = { "ANALOG ON", "ANALOG OFF", "DEEP SLEEP" };

/* Варианты HAT-PWR: режим каждой линии. Порядок: RST, CS, DC, BUSY, SPI (SCK+MOSI).
 * BUSY - вход: L/H оставляют его цифровым входом, A отключает. Первые 7 - группа 8, остальные - группа 16. */
#define L PWR_PIN_LOW
#define H PWR_PIN_HIGH
#define A PWR_PIN_ANALOG
static const struct { const char *name; uint8_t m[5]; } HAT_VAR[] = {
    /* группа 8: одна линия отличается, остальные в 1 */
    { "ALL HIGH",          { H, H, H, H, H } },
    { "RST LOW",           { L, H, H, H, H } },
    { "RST ANALOG",        { A, H, H, H, H } },
    { "CS ANALOG",         { H, A, H, H, H } },
    { "BUSY ANALOG",       { H, H, H, A, H } },
    { "DC ANALOG",         { H, H, A, H, H } },
    { "SCK MOSI ANALOG",   { H, H, H, H, A } },
    /* группа 16: RST в 0 / analog, остальные подобраны так, чтобы не подпитывать HAT через линии */
    { "ALL ANALOG",        { A, A, A, A, A } },
    { "RST0 REST 0",       { L, L, L, A, L } },
    { "RST0 REST ANALOG",  { L, A, A, A, A } },
    { "RST0 CS1 REST AN",  { L, H, A, A, A } },
    { "RST0 DC1 REST AN",  { L, A, H, A, A } },
    { "RST0 SPI1 REST AN", { L, A, A, A, H } },
    { "RST0 BIN REST AN",  { L, A, A, L, A } },
};
#undef L
#undef H
#undef A
#define HAT_N_G8   7
#define HAT_N_ALL  ((int)ARRAY_N(HAT_VAR))

static const char *const WAKE1B_NAME[] = { "STAY 1B", "SLP 1B" };
static const char *const WAKE_NAME[] = { "STAY", "SLP FULL", "SLP PART" };
#define WAKE_N  ((int)ARRAY_N(WAKE_NAME))

static void add_step(int grp, int panel, int lines, int cpu)
{
    s_total_all++;
    if (s_total_all < PWR_FIRST_STEP || s_total_all > PWR_LAST_STEP || s_n >= MAX_STEPS)
        return;
    s_steps[s_n].grp = (uint8_t)grp;
    s_steps[s_n].panel = (uint8_t)panel;
    s_steps[s_n].lines = (uint8_t)lines;
    s_steps[s_n].cpu = (uint8_t)cpu;
    s_n++;
}

static void build_steps(void)
{
    s_n = 0; s_total_all = 0;
#if PWR_GROUPS & 1
    for (int p = 0; p < 3; p++)
        for (int l = 0; l < ARRAY_N(LINES_STATIC); l++)
            for (int c = 0; c < ARRAY_N(CPUS); c++)
                add_step(GRP_STATIC, p, LINES_STATIC[l], CPUS[c]);
#endif
#if PWR_GROUPS & 2
    for (int k = 1; k >= 0; k--)
        for (int l = 0; l < ARRAY_N(LINES_DYN); l++)
            for (int c = 0; c < ARRAY_N(CPUS); c++)
                add_step(GRP_DYN4G, k, LINES_DYN[l], CPUS[c]);
#endif
#if PWR_GROUPS & 4
    for (int l = 0; l < ARRAY_N(LINES_DYN); l++)
        for (int c = 0; c < ARRAY_N(CPUS); c++)
            add_step(GRP_DYNBW, PWR_TEST_BW_KEEP, LINES_DYN[l], CPUS[c]);
#endif
#if PWR_GROUPS & 32
    for (int v = 0; v < WAKE_N; v++)
        add_step(GRP_WAKE, 0, v, PWR_CPU_STOP2);
#endif
#if PWR_GROUPS & 64
    for (int v = 0; v < 2; v++)
        add_step(GRP_WAKE1B, 0, v, PWR_CPU_STOP2);
#endif
#if PWR_GROUPS & 8
    for (int v = 0; v < HAT_N_G8; v++)
        add_step(GRP_HAT, 0, v, PWR_CPU_STOP2);
#endif
#if PWR_GROUPS & 16
    for (int v = HAT_N_G8; v < HAT_N_ALL; v++)
        add_step(GRP_HAT, 0, v, PWR_CPU_STOP2);
#endif
}

static const char *grp_name(int g)
{
    return g == GRP_STATIC ? "STATIC" : (g == GRP_DYN4G ? "DYN 4GRAY" : (g == GRP_DYNBW ? "DYN 1BIT" : (g == GRP_HAT ? "HAT PWR" : (g == GRP_WAKE ? "WAKE" : "WAKE 1BIT"))));
}

static void describe(const step_t *s, char *out, size_t n)
{
    if (s->grp == GRP_WAKE1B)
        snprintf(out, n, "WAKE1B %s idle=%d s", WAKE1B_NAME[s->lines], PWR_WAKE_IDLE_S);
    else if (s->grp == GRP_WAKE)
        snprintf(out, n, "WAKE %s idle=%d s", WAKE_NAME[s->lines], PWR_WAKE_IDLE_S);
    else if (s->grp == GRP_HAT)
        snprintf(out, n, "%s cpu=STOP2 panel=ANALOG OFF others=HIGH, %s", grp_name(s->grp), HAT_VAR[s->lines].name);
    else if (s->grp == GRP_STATIC)
        snprintf(out, n, "%s cpu=%s panel=%s lines=%s", grp_name(s->grp), CPU_NAME[s->cpu],
                 PANEL_NAME[s->panel], LINES_NAME[s->lines]);
    else
        snprintf(out, n, "%s cpu=%s analog=%s lines=%s", grp_name(s->grp), CPU_NAME[s->cpu],
                 s->panel ? "KEEP" : "CYCLE", LINES_NAME[s->lines]);
}

/* --------------------------- текст на экране ------------------------------- */

static uint8_t *t_fb;
static int      t_bw;

static void t_px(int x, int y, int ink)
{
    if (t_bw)
        epd4g_bw_fb_set_rot(t_fb, EINK_ROT, x, y, ink ? 0 : 1);
    else
        epd4g_fb_set_rot(t_fb, EINK_ROT, x, y, ink ? EPD4G_BLACK : EPD4G_WHITE);
}

static void t_rect(int x0, int y0, int x1, int y1, int ink)
{
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            t_px(x, y, ink);
}

static void t_box(int x0, int y0, int x1, int y1)     /* белое поле с чёрной рамкой 2 px */
{
    t_rect(x0, y0, x1, y1, 1);
    t_rect(x0 + 2, y0 + 2, x1 - 2, y1 - 2, 0);
}

static void t_text(int x, int y, const char *s, int scale)
{
    for (; *s; s++) {
        int c = (unsigned char)*s;
        if (c >= 'a' && c <= 'z')
            c -= 32;
        if (c < FONT5X7_FIRST || c > FONT5X7_LAST)
            c = '?';
        const uint8_t *g = font5x7[c - FONT5X7_FIRST];
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 7; row++)
                if ((g[col] >> row) & 1)
                    t_rect(x + col * scale, y + row * scale,
                           x + col * scale + scale - 1, y + row * scale + scale - 1, 1);
        x += 6 * scale;
    }
}

/* Подпись шага; fb - буфер 4 gray (bw = 0) или 1-bit (bw = 1). */
static void draw_label(uint8_t *fb, int bw, int num, const step_t *s, int hold_s)
{
    char l[6][24];

    t_fb = fb; t_bw = bw;
    snprintf(l[0], sizeof l[0], "#%02d/%02d", num, s_n ? s_total_all : 0);
    if (s->grp == GRP_WAKE1B) {
        snprintf(l[1], sizeof l[1], "WAKE 1BIT");
        snprintf(l[2], sizeof l[2], "%s", WAKE1B_NAME[s->lines]);
        snprintf(l[3], sizeof l[3], "IDLE %d S", PWR_WAKE_IDLE_S);
        t_box(0, 8, 71, 83);
        t_text(6, 14, l[0], 1);
        for (int i = 1; i <= 3; i++)
            t_text(6, 14 + i * 10, l[i], 1);
        return;
    }
    if (s->grp == GRP_WAKE) {
        snprintf(l[1], sizeof l[1], "WAKE TEST");
        snprintf(l[2], sizeof l[2], "%s", WAKE_NAME[s->lines]);
        snprintf(l[3], sizeof l[3], "IDLE %d S", PWR_WAKE_IDLE_S);
        t_box(0, 100, 71, 175);
        t_text(6, 106, l[0], 1);
        for (int i = 1; i <= 3; i++)
            t_text(6, 106 + i * 10, l[i], 1);
        return;
    }
    if (s->grp == GRP_HAT) {
        char a[24];
        snprintf(a, sizeof a, "#%02d/%02d HAT PWR", num, s_total_all);
        t_box(10, 36, 253, 139);
        t_text(18, 44, a, 2);
        t_text(18, 62, "CPU:   STOP2", 2);
        t_text(18, 80, "PANEL: ANALOG OFF", 2);
        t_text(18, 98, HAT_VAR[s->lines].name, 2);
        snprintf(l[4], sizeof l[4], "HOLD %d S", hold_s);
        t_text(18, 116, l[4], 2);
        return;
    }
    if (s->grp == GRP_STATIC) {
        char a[24];
        snprintf(a, sizeof a, "#%02d/%02d STATIC", num, s_total_all);
        snprintf(l[1], sizeof l[1], "CPU:   %s", CPU_NAME[s->cpu]);
        snprintf(l[2], sizeof l[2], "PANEL: %s", PANEL_NAME[s->panel]);
        snprintf(l[3], sizeof l[3], "LINES: %s", LINES_NAME[s->lines]);
        snprintf(l[4], sizeof l[4], "HOLD %d S", hold_s);
        t_box(10, 36, 253, 139);
        t_text(18, 44, a, 2);
        for (int i = 1; i <= 4; i++)
            t_text(18, 44 + i * 18, l[i], 2);
        return;
    }
    /* динамика: маленькая подпись (1x) в углу, куда маркёр / меню не доходят */
    snprintf(l[1], sizeof l[1], s->grp == GRP_DYN4G ? "4G MOVE" : "1BIT MENU");
    snprintf(l[2], sizeof l[2], s->panel ? "PWR KEEP" : "PWR CYCLE");
    snprintf(l[3], sizeof l[3], "CPU %s", CPU_NAME[s->cpu]);
    snprintf(l[4], sizeof l[4], "LN %s", LINES_NAME[s->lines]);
    snprintf(l[5], sizeof l[5], "%d S", hold_s);
    const int y0 = (s->grp == GRP_DYN4G) ? 100 : 8;
    t_box(0, y0, 71, y0 + 75);
    for (int i = 0; i < 6; i++)
        t_text(6, y0 + 6 + i * 10, i == 0 ? l[0] : l[i], 1);
}

/* ------------------------------ ожидания ----------------------------------- */

static void hold(pwr_cpu_t cpu, uint32_t ms)
{
    const uint32_t t_end = HAL_GetTick() + ms;

    for (;;) {
        const int32_t left = (int32_t)(t_end - HAL_GetTick());
        if (left <= 0)
            break;
        (void)pwr_wait(cpu, (uint32_t)left, 0);
    }
}

/* Пауза внутри обновления: пока панель рисует, SPI не нужен. */
static void idle_wait(const step_t *s, uint32_t ms, int on_busy)
{
    const int gate = (s->lines != PWR_LINES_ACTIVE);

    if (gate)
        pwr_lines_set((pwr_lines_t)s->lines, PWR_F_KEEP_CTL);
    (void)pwr_wait((pwr_cpu_t)s->cpu, ms, on_busy);
    if (gate)
        pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_KEEP_CTL);
}

/* Ведёт запущенное задание библиотеки до конца, в паузах применяя режимы шага. */
static int run_job(const step_t *s)
{
    while (epd4g_poll() == EPD4G_PENDING) {
        int on_busy = 0;
        const uint32_t ms = epd4g_wait_hint(&on_busy);

        if (ms == 0U || ms == EPD4G_WAIT_FOREVER)
            continue;                           /* идёт обмен по SPI: линии должны быть активны */
        idle_wait(s, ms, on_busy);
    }
    return epd4g_result();
}

/* ------------------------------ STATIC ------------------------------------- */

static int step_static(const step_t *s, int num)
{
    uint32_t hold_s = PWR_DWELL_STATIC_S;
    int rc;

    if (s->panel == PANEL_ON && hold_s > PWR_ANALOG_ON_MAX_S)
        hold_s = PWR_ANALOG_ON_MAX_S;

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    g_pwr_keep_gray = 1;                        /* после обновления аналог остаётся включённым */
    if ((rc = epd4g_init()) != EPD4G_OK)
        return rc;
    memcpy(cur, bg, sizeof cur);
    draw_label(cur, 0, num, s, (int)hold_s);
    if ((rc = epd4g_full(cur, EPD4G_FULL_FAST)) != EPD4G_OK)
        return rc;

    if (s->panel == PANEL_OFF)
        rc = epd4g_power_off();
    else if (s->panel == PANEL_SLEEP)
        rc = epd4g_sleep();
    if (rc != EPD4G_OK)
        return rc;

    pwr_lines_set((pwr_lines_t)s->lines, PWR_F_BUSY);
    DIAG("[HOLD] %lu s, measure now (DC=%d)\r\n", (unsigned long)hold_s,
         (int)(((GPIOB->ODR >> 8) & 1U)));
    pwr_quiet(1);
    hold((pwr_cpu_t)s->cpu, hold_s * 1000U);
    pwr_quiet(0);
    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    return EPD4G_OK;
}

/* ------------------------------ HAT-PWR ------------------------------------ */

static int step_hat(const step_t *s, int num)
{
    const int hold_s = PWR_DWELL_STATIC_S;
    int rc;

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    g_pwr_keep_gray = 1;
    if ((rc = epd4g_init()) != EPD4G_OK)
        return rc;
    memcpy(cur, bg, sizeof cur);
    draw_label(cur, 0, num, s, hold_s);
    if ((rc = epd4g_full(cur, EPD4G_FULL_FAST)) != EPD4G_OK)
        return rc;
    if ((rc = epd4g_power_off()) != EPD4G_OK)       /* аналог выключен ДО любых манипуляций с линиями */
        return rc;

    pwr_lines_set(PWR_LINES_HIGH, 0);
    {
        static const pwr_pin_t PINS[5] = { PWR_PIN_RST, PWR_PIN_CS, PWR_PIN_DC, PWR_PIN_BUSY, PWR_PIN_SPI };
        for (int i = 0; i < 5; i++)
            pwr_pin_set(PINS[i], (pwr_pinmode_t)HAT_VAR[s->lines].m[i]);
    }
    DIAG("[HOLD] %d s, measure now\r\n", hold_s);
    pwr_quiet(1);
    hold(PWR_CPU_STOP2, (uint32_t)hold_s * 1000U);
    pwr_quiet(0);

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    DIAG("[CHECK IMAGE] 8 s: did the picture survive? BUSY=%d\r\n",
         (int)HAL_GPIO_ReadPin(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin));
    hold(PWR_CPU_SLEEP, 8000U);

    uint32_t t0 = HAL_GetTick();
    rc = epd4g_init();
    const uint32_t t_init = HAL_GetTick() - t0;
    DIAG("[RECOVER] init rc=%d, %lu ms\r\n", rc, (unsigned long)t_init);
    if (rc != EPD4G_OK)
        return rc;
    t0 = HAL_GetTick();
    rc = epd4g_full(cur, EPD4G_FULL_FAST);
    DIAG("[RECOVER] full refresh rc=%d, %lu ms\r\n", rc, (unsigned long)(HAL_GetTick() - t0));
    return rc;
}

/* ------------------------------ DYN-4G ------------------------------------- */

static uint8_t lbl[EPD4G_FB_BYTES];             /* карта + подпись: фон под маркёром и треком */

static void route_restart(void)
{
    s_pos = 0.0f;
    memcpy(base, lbl, sizeof base);
    memcpy(cur, lbl, sizeof cur);
    route_at(s_pos, &s_cx, &s_cy, &s_hx, &s_hy);
    draw_marker(cur, s_cx, s_cy, s_hx, s_hy);
}

static int gray_frame_start(void)
{
    const float s_new = s_pos + (float)EINK_STEP_PX;
    uint32_t changed = 0;

    memcpy(prev, cur, sizeof prev);             /* то, что сейчас на панели */
    if (s_new >= rt_cum[ROUTE_N]) {             /* круг пройден: трек стирается полным обновлением */
        route_restart();
        memcpy(prev, cur, sizeof prev);
        return epd4g_start_full(cur, EPD4G_FULL_FAST);
    }
    const int ox = s_cx, oy = s_cy;
    track_draw(base, s_pos, s_new);
    s_pos = s_new;
    route_at(s_pos, &s_cx, &s_cy, &s_hx, &s_hy);
    epd4g_rect_t r = rect_union(rect_around(ox, oy), rect_around(s_cx, s_cy));
    epd4g_fb_copy_rect(cur, base, &r);
    draw_marker(cur, s_cx, s_cy, s_hx, s_hy);
    return epd4g_start_partial_diff(prev, cur, &r, &changed);
}

static void dyn_report(uint32_t t0, int n)
{
    const uint32_t dt = HAL_GetTick() - t0;

    DIAG("[DONE] updates=%d, %lu ms, avg %lu ms per update\r\n", n, (unsigned long)dt,
         n ? (unsigned long)(dt / (uint32_t)n) : 0UL);
}

/* ------------------------------ WAKE --------------------------------------- */

static int step_wake(const step_t *s, int num)
{
    const step_t rj = { GRP_DYN4G, 1, PWR_LINES_ACTIVE, PWR_CPU_STOP2 };   /* режимы для run_job */
    const int variant = s->lines;                   /* 0 STAY, 1 SLP FULL, 2 SLP PART */
    int rc;

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    g_pwr_keep_gray = 1;
    if ((rc = epd4g_init()) != EPD4G_OK)
        return rc;
    memcpy(lbl, bg, sizeof lbl);
    draw_label(lbl, 0, num, s, PWR_WAKE_IDLE_S);
    route_restart();
    if ((rc = epd4g_full(cur, EPD4G_FULL_FAST)) != EPD4G_OK)
        return rc;
    for (int i = 0; i < 3; i++) {                   /* маркёр едет: панель в обычном рабочем состоянии */
        if ((rc = gray_frame_start()) != EPD4G_OK || (rc = run_job(&rj)) != EPD4G_OK)
            return rc;
    }
    if ((rc = epd4g_power_off()) != EPD4G_OK)       /* аналог выключен до любых манипуляций с линиями */
        return rc;

    if (variant != 0 && (rc = pwr_hat_sleep()) != EPD4G_OK)   /* power off + сон HAT */
        return rc;
    DIAG("[IDLE] %d s, HAT %s\r\n", PWR_WAKE_IDLE_S, variant ? "ASLEEP" : "awake (power off only)");
    pwr_quiet(1);
    hold(PWR_CPU_STOP2, (uint32_t)PWR_WAKE_IDLE_S * 1000U);
    pwr_quiet(0);

    /* ---- конец простоя: от этой точки считайте заряд до конца обновления ---- */
    const uint32_t t0 = HAL_GetTick();
    uint32_t t_init = 0;
    if (variant != 0) {
        if ((rc = pwr_hat_wake(NULL, EPD4G_SRC_WHITE)) != EPD4G_OK)   /* линии назад + epd4g_init() */
            return rc;
        t_init = HAL_GetTick() - t0;
    }
    if (variant == 1) {                             /* после сна: полное обновление следующего кадра */
        const float s_new = s_pos + (float)EINK_STEP_PX;
        track_draw(base, s_pos, s_new);
        s_pos = s_new;
        route_at(s_pos, &s_cx, &s_cy, &s_hx, &s_hy);
        memcpy(cur, base, sizeof cur);
        draw_marker(cur, s_cx, s_cy, s_hx, s_hy);
        rc = epd4g_full(cur, EPD4G_FULL_FAST);
    } else {                                        /* STAY и SLP PART: обычное частичное обновление */
        if ((rc = gray_frame_start()) == EPD4G_OK)
            rc = run_job(&rj);
    }
    DIAG("[WAKE] init %lu ms, update rc=%d, total %lu ms from end of idle\r\n", (unsigned long)t_init, rc,
         (unsigned long)(HAL_GetTick() - t0));
    if (rc != EPD4G_OK)
        return rc;
    if (variant != 1 && (rc = gray_frame_start()) == EPD4G_OK)      /* ещё кадр: работает ли штатно дальше */
        rc = run_job(&rj);
    DIAG("[CHECK IMAGE] 8 s: marker moved, background intact? rc=%d\r\n", rc);
    hold(PWR_CPU_SLEEP, 8000U);
    return rc;
}

static int step_dyn4g(const step_t *s, int num)
{
    int rc, n = 0;

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    g_pwr_keep_gray = s->panel;
    if ((rc = epd4g_init()) != EPD4G_OK)
        return rc;
    memcpy(lbl, bg, sizeof lbl);
    draw_label(lbl, 0, num, s, PWR_DWELL_DYN_S);
    route_restart();
    if ((rc = epd4g_full(cur, EPD4G_FULL_FAST)) != EPD4G_OK)
        return rc;

    DIAG("[RUN] %d s, measure now\r\n", PWR_DWELL_DYN_S);
    pwr_quiet(1);                               /* UART молчит на время замера */
    const uint32_t t0 = HAL_GetTick(), t_end = t0 + PWR_DWELL_DYN_S * 1000U;
    while ((int32_t)(t_end - HAL_GetTick()) > 0) {
        if ((rc = gray_frame_start()) != EPD4G_OK)
            break;
        if ((rc = run_job(s)) != EPD4G_OK)
            break;
        n++;
    }
    pwr_quiet(0);
    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    dyn_report(t0, n);
    return rc;
}

/* ------------------------------ DYN-1B ------------------------------------- */

#define PT_MENU_ROWS    6
#define PT_MENU_ROW_H   26
#define PT_MENU_Y0      6
#define PT_MENU_X0      80                      /* слева - подпись */

static uint8_t bwbuf[EPD4G_BW_FB_BYTES];

static epd4g_rect_t menu_row(int i, int selected)
{
    const int lw = epd4g_logical_width(EINK_ROT);
    const int x0 = PT_MENU_X0, x1 = lw - 7;
    const int y0 = PT_MENU_Y0 + i * PT_MENU_ROW_H, y1 = y0 + PT_MENU_ROW_H - 3;
    epd4g_rect_t r = {0, 0, 0, 0};

    t_fb = bwbuf; t_bw = 1;
    t_rect(x0, y0, x1, y1, 1);                                  /* чёрный прямоугольник */
    if (!selected)
        t_rect(x0 + 2, y0 + 2, x1 - 2, y1 - 2, 0);              /* внутри белый: остаётся рамка */
    else
        t_rect(x0 + 8, y0 + 6, x0 + 24, y1 - 6, 0);             /* метка */
    t_rect(x0 + 34, y0 + 8, x0 + 34 + 14 * (i + 2), y0 + 12, selected ? 0 : 1);   /* «текст» */
    epd4g_map_rect(EINK_ROT, x0, y0, x1, y1, &r);
    return r;
}

static int step_dynbw(const step_t *s, int num)
{
    int rc, n = 0, sel = 0;

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    if ((rc = epd4g_bw_init(EPD4G_BW_NORMAL)) != EPD4G_OK)
        return rc;
    epd4g_bw_fb_fill(bwbuf, 1);
    draw_label(bwbuf, 1, num, s, PWR_DWELL_DYN_S);
    for (int i = 0; i < PT_MENU_ROWS; i++)
        (void)menu_row(i, i == sel);
    if ((rc = epd4g_bw_full(bwbuf, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL)) != EPD4G_OK)
        return rc;

    DIAG("[RUN] %d s, measure now\r\n", PWR_DWELL_DYN_S);
    pwr_quiet(1);
    const uint32_t t0 = HAL_GetTick(), t_end = t0 + PWR_DWELL_DYN_S * 1000U;
    while ((int32_t)(t_end - HAL_GetTick()) > 0) {
        epd4g_rect_t a = menu_row(sel, 0);
        sel = (sel + 1) % PT_MENU_ROWS;
        epd4g_rect_t b = menu_row(sel, 1);
        epd4g_rect_t r = rect_union(a, b);
        if ((rc = epd4g_start_bw_partial(bwbuf, EPD4G_SRC_1BPP, &r)) != EPD4G_OK)
            break;
        if ((rc = run_job(s)) != EPD4G_OK)
            break;
        n++;
    }
    pwr_quiet(0);
    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    dyn_report(t0, n);
    return rc;
}

/* ------------------------------ WAKE-1B ------------------------------------ */

/* 1-bit: сон HAT и восстановление RAM контроллера (epd4g_bw_restore) вместо полного обновления. */
static int step_wake1b(const step_t *s, int num)
{
    const step_t rj = { GRP_DYNBW, 0, PWR_LINES_ACTIVE, PWR_CPU_STOP2 };
    const int sleep_hat = s->lines;                 /* 0 STAY 1B, 1 SLP 1B */
    int rc, sel = 0;

    pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    if ((rc = epd4g_bw_init(EPD4G_BW_NORMAL)) != EPD4G_OK)
        return rc;
    epd4g_bw_fb_fill(bwbuf, 1);
    draw_label(bwbuf, 1, num, s, PWR_WAKE_IDLE_S);
    for (int i = 0; i < PT_MENU_ROWS; i++)
        (void)menu_row(i, i == sel);
    if ((rc = epd4g_bw_full(bwbuf, EPD4G_SRC_1BPP, EPD4G_BW_NORMAL)) != EPD4G_OK)
        return rc;
    for (int k = 0; k < 4; k++) {                   /* курсор меню едет: обычное рабочее состояние */
        epd4g_rect_t a = menu_row(sel, 0);
        sel = (sel + 1) % PT_MENU_ROWS;
        epd4g_rect_t b = menu_row(sel, 1);
        epd4g_rect_t r = rect_union(a, b);
        if ((rc = epd4g_start_bw_partial(bwbuf, EPD4G_SRC_1BPP, &r)) != EPD4G_OK || (rc = run_job(&rj)) != EPD4G_OK)
            return rc;
    }
    if (sleep_hat && (rc = pwr_hat_sleep()) != EPD4G_OK)
        return rc;
    DIAG("[IDLE] %d s, HAT %s\r\n", PWR_WAKE_IDLE_S, sleep_hat ? "ASLEEP (1-bit)" : "awake");
    pwr_quiet(1);
    hold(PWR_CPU_STOP2, (uint32_t)PWR_WAKE_IDLE_S * 1000U);
    pwr_quiet(0);

    /* ---- конец простоя: от этой точки считайте заряд до конца обновления ---- */
    uint32_t t0 = HAL_GetTick();
    if (sleep_hat && (rc = pwr_hat_wake(bwbuf, EPD4G_SRC_1BPP)) != EPD4G_OK)    /* restore RAM, без вспышки */
        return rc;
    const uint32_t t_wake = HAL_GetTick() - t0;
    for (int k = 0; k < 2; k++) {
        epd4g_rect_t a = menu_row(sel, 0);
        sel = (sel + 1) % PT_MENU_ROWS;
        epd4g_rect_t b = menu_row(sel, 1);
        epd4g_rect_t r = rect_union(a, b);
        if ((rc = epd4g_start_bw_partial(bwbuf, EPD4G_SRC_1BPP, &r)) != EPD4G_OK || (rc = run_job(&rj)) != EPD4G_OK)
            return rc;
        if (k == 0)
            DIAG("[WAKE] restore %lu ms, first update done %lu ms from end of idle\r\n",
                 (unsigned long)t_wake, (unsigned long)(HAL_GetTick() - t0));
    }
    DIAG("[CHECK IMAGE] 8 s: menu intact, cursor moved, no garbage? rc=%d\r\n", rc);
    hold(PWR_CPU_SLEEP, 8000U);
    return rc;
}

/* --------------------------------- API ------------------------------------- */

int eink_demo_init(void)
{
    char d[80];

    pwr_hw_init();
    epd4g_set_partials_before_full(0, 0);       /* замер чистых частичных обновлений, без автоматических полных */
    route_init();
    render_map(bg);
    build_steps();
    (void)s_last_ms; (void)s_upd_cnt; (void)prev;
    (void)CPUS; (void)LINES_DYN; (void)LINES_STATIC;       /* при выбранной части групп не все таблицы нужны */

    DIAG("\r\n=== POWER TEST: %d steps (static %d s, dynamic %d s) ===\r\n", s_n,
         PWR_DWELL_STATIC_S, PWR_DWELL_DYN_S);
    for (int i = 0; i < s_n; i++) {
        describe(&s_steps[i], d, sizeof d);
        DIAG("  %2d  %s\r\n", PWR_FIRST_STEP + i, d);
    }
    return s_n > 0 ? 0 : -1;
}

static int finish(void)
{
    static int done;

    if (!done) {
        done = 1;
        DIAG("=== POWER TEST DONE ===\r\n");
        pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
        g_pwr_keep_gray = 0;
        if (epd4g_init() == EPD4G_OK) {
            epd4g_fb_fill(cur, EPD4G_WHITE);
            t_fb = cur; t_bw = 0;
            t_box(30, 60, 233, 115);
            t_text(46, 79, "TEST DONE", 3);
            (void)epd4g_full(cur, EPD4G_FULL_FAST);
            (void)epd4g_sleep();
        }
        pwr_lines_set(PWR_LINES_ANALOG, PWR_F_BUSY);
    }
    (void)pwr_wait(PWR_CPU_STOP2, 10000U, 0);
    return 0;
}

int eink_demo_task(void)
{
    char d[80];

    if (s_idx >= s_n)
        return finish();

    const step_t *s = &s_steps[s_idx];
    const int num = PWR_FIRST_STEP + s_idx;
    int rc;

    describe(s, d, sizeof d);
    DIAG("\r\n[STEP %d/%d] %s\r\n", num, s_total_all, d);
    rc = s->grp == GRP_STATIC ? step_static(s, num)
       : s->grp == GRP_DYN4G  ? step_dyn4g(s, num)
       : s->grp == GRP_HAT    ? step_hat(s, num)
       : s->grp == GRP_WAKE   ? step_wake(s, num)
       : s->grp == GRP_WAKE1B ? step_wake1b(s, num) : step_dynbw(s, num);
    if (rc != EPD4G_OK) {
        DIAG("[ERROR] step %d failed rc=%d, panel is reset\r\n", num, rc);
        epd4g_abort();                          /* следующий шаг начнёт с init */
        pwr_quiet(0);
        pwr_lines_set(PWR_LINES_ACTIVE, PWR_F_BUSY);
    }
    s_idx++;
    return 0;
}

uint32_t eink_demo_wait_ms(int *on_busy)
{
    if (on_busy != NULL)
        *on_busy = 0;
    return 0;                                   /* тест сам управляет сном внутри шагов */
}

uint32_t eink_demo_last_update_ms(void)
{
    return 0;
}
