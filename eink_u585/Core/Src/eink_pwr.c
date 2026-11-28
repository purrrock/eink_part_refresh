/*
 * eink_pwr.c - режимы энергосбережения для теста тока (STM32U585). См. eink_pwr.h.
 *
 * Сон Sleep делает eink_sleep() (TIM6, EXTI2), Stop 2 - этот файл: будильник LPTIM1 на LSI
 * (32 кГц / 32 = 1 кГц, т. е. 1 мс на тик) и пробуждение по спаду BUSY (EXTI2, PB2).
 */
#include "eink_pwr.h"

#include "eink_sleep.h"
#include "main.h"
#include "spi.h"
#include "usart.h"

/* Выводы дисплея (SCK/MOSI в CubeMX без меток, их задаёт HAL_SPI_MspInit: PA5, PA7). */
#define SCK_PORT        GPIOA
#define SCK_PIN         GPIO_PIN_5
#define MOSI_PORT       GPIOA
#define MOSI_PIN        GPIO_PIN_7

/* Паузы короче этого срока в режиме Stop 2 проводим в Sleep: вход/выход из Stop 2 того не стоит. */
#ifndef PWR_STOP_MIN_MS
#define PWR_STOP_MIN_MS 3u
#endif
#define LPTIM_MAX_MS    65000u                  /* ARR 16 бит, тик 1 мс */

static uint8_t s_spi_off;                       /* SPI1 остановлен (HAL_SPI_DeInit) */
static uint8_t s_busy_off;                      /* BUSY переведён в analog */

/* ------------------------------ линии дисплея ------------------------------ */

static void pin_out(GPIO_TypeDef *port, uint32_t pin, int level)
{
    GPIO_InitTypeDef g = {0};

    HAL_GPIO_WritePin(port, pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);   /* уровень до включения выхода */
    g.Pin = pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}

static void pin_analog(GPIO_TypeDef *port, uint32_t pin)
{
    GPIO_InitTypeDef g = {0};

    g.Pin = pin;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &g);
}

static void busy_in(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin = EINK_BUSY_Pin;                      /* как в CubeMX (gpio.c) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EINK_BUSY_GPIO_Port, &g);
}

void pwr_lines_set(pwr_lines_t mode, unsigned flags)
{
    const int keep_ctl = (flags & PWR_F_KEEP_CTL) != 0U;

    if (mode == PWR_LINES_ACTIVE) {
        if (s_spi_off) {                        /* вернуть SPI1 и линии, как их настроил CubeMX */
            if (!keep_ctl) {
                pin_out(EINK_CS_GPIO_Port, EINK_CS_Pin, 1);
                pin_out(EINK_RST_GPIO_Port, EINK_RST_Pin, 1);
            }
            pin_out(EINK_DC_GPIO_Port, EINK_DC_Pin, 1);
            MX_SPI1_Init();                     /* тактирование SPI1, SCK/MOSI в AF5 */
            s_spi_off = 0;
        }
        if (s_busy_off) {
            busy_in();
            s_busy_off = 0;
        }
        return;
    }

    if (!s_spi_off) {                           /* выключает SPI1, снимает его тактирование, SCK/MOSI -> analog */
        HAL_SPI_DeInit(&hspi1);
        s_spi_off = 1;
    }

    switch (mode) {
    case PWR_LINES_LOW:
        pin_out(SCK_PORT, SCK_PIN, 0);
        pin_out(MOSI_PORT, MOSI_PIN, 0);
        pin_out(EINK_DC_GPIO_Port, EINK_DC_Pin, 0);
        if (!keep_ctl) {
            pin_out(EINK_CS_GPIO_Port, EINK_CS_Pin, 1);     /* CS = 0 выбрал бы панель, RST = 0 сбросил бы её */
            pin_out(EINK_RST_GPIO_Port, EINK_RST_Pin, 1);
        }
        break;
    case PWR_LINES_HIGH:
        pin_out(SCK_PORT, SCK_PIN, 1);
        pin_out(MOSI_PORT, MOSI_PIN, 1);
        pin_out(EINK_DC_GPIO_Port, EINK_DC_Pin, 1);
        if (!keep_ctl) {
            pin_out(EINK_CS_GPIO_Port, EINK_CS_Pin, 1);
            pin_out(EINK_RST_GPIO_Port, EINK_RST_Pin, 1);
        }
        break;
    case PWR_LINES_ANALOG:
    default:
        pin_analog(SCK_PORT, SCK_PIN);
        pin_analog(MOSI_PORT, MOSI_PIN);
        pin_analog(EINK_DC_GPIO_Port, EINK_DC_Pin);
        if (!keep_ctl) {
            pin_analog(EINK_CS_GPIO_Port, EINK_CS_Pin);
            pin_analog(EINK_RST_GPIO_Port, EINK_RST_Pin);
        }
        if ((flags & PWR_F_BUSY) != 0U) {
            pin_analog(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin);
            s_busy_off = 1;
        }
        break;
    }
}

void pwr_pin_set(pwr_pin_t pin, pwr_pinmode_t mode)
{
    GPIO_TypeDef *port = NULL, *port2 = NULL;
    uint32_t pn = 0, pn2 = 0;

    switch (pin) {
    case PWR_PIN_RST:  port = EINK_RST_GPIO_Port; pn = EINK_RST_Pin; break;
    case PWR_PIN_CS:   port = EINK_CS_GPIO_Port;  pn = EINK_CS_Pin;  break;
    case PWR_PIN_DC:   port = EINK_DC_GPIO_Port;  pn = EINK_DC_Pin;  break;
    case PWR_PIN_SPI:  port = SCK_PORT; pn = SCK_PIN; port2 = MOSI_PORT; pn2 = MOSI_PIN; break;
    case PWR_PIN_BUSY:
    default:
        if (mode == PWR_PIN_ANALOG) {
            pin_analog(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin);
            s_busy_off = 1;
        } else {
            busy_in();
            s_busy_off = 0;
        }
        return;
    }
    if (mode == PWR_PIN_ANALOG) {
        pin_analog(port, pn);
        if (port2 != NULL)
            pin_analog(port2, pn2);
    } else {
        pin_out(port, pn, mode == PWR_PIN_HIGH);
        if (port2 != NULL)
            pin_out(port2, pn2, mode == PWR_PIN_HIGH);
    }
}

/* ---------------------------------- UART ----------------------------------- */

static void uart_flush(UART_HandleTypeDef *h)
{
    const uint32_t t0 = HAL_GetTick();

    if (h->gState == HAL_UART_STATE_RESET)
        return;
    while (!__HAL_UART_GET_FLAG(h, UART_FLAG_TC) && (HAL_GetTick() - t0) < 100U) {
    }
}

void pwr_quiet(int on)
{
    if (on) {
        uart_flush(&huart1);
        uart_flush(&huart2);
        __HAL_RCC_USART1_CLK_DISABLE();
        __HAL_RCC_USART2_CLK_DISABLE();
    } else {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
    }
}

/* ----------------------------- LPTIM1 (будильник) --------------------------- */

/* Обработчик только гасит флаг: просыпается МК, работу делает вызывающий. */
void LPTIM1_IRQHandler(void)
{
    LPTIM1->ICR = LPTIM_ICR_ARRMCF;
}

static void lptim_start(uint32_t ms)
{
    uint32_t guard;

    LPTIM1->CR = 0U;                            /* CFGR и DIER меняются только при выключенном LPTIM */
    for (guard = 0; (LPTIM1->CR & LPTIM_CR_ENABLE) != 0U && guard < 100000U; guard++) {
    }
    LPTIM1->CFGR = (5UL << LPTIM_CFGR_PRESC_Pos);   /* /32: 32 кГц LSI -> 1 кГц */
    LPTIM1->DIER = LPTIM_DIER_ARRMIE;
    LPTIM1->CR = LPTIM_CR_ENABLE;
    LPTIM1->ICR = LPTIM_ICR_ARROKCF | LPTIM_ICR_ARRMCF;
    LPTIM1->ARR = ms - 1U;
    for (guard = 0; (LPTIM1->ISR & LPTIM_ISR_ARROK) == 0U && guard < 100000U; guard++) {
    }
    LPTIM1->ICR = LPTIM_ICR_ARROKCF;
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;             /* непрерывный режим: по совпадению с ARR - флаг и прерывание */
}

/* Останавливает будильник, возвращает прошедшее время в мс. */
static uint32_t lptim_stop(uint32_t ms)
{
    uint32_t el;

    if ((LPTIM1->ISR & LPTIM_ISR_ARRM) != 0U) {
        el = ms;                                /* досчитал до конца */
    } else {
        uint32_t a, b;
        do {                                    /* CNT читается из другого тактового домена: до двух одинаковых */
            a = LPTIM1->CNT;
            b = LPTIM1->CNT;
        } while (a != b);
        el = a + 1U;
        if (el > ms)
            el = ms;
    }
    LPTIM1->CR = 0U;
    LPTIM1->ICR = LPTIM_ICR_ARRMCF | LPTIM_ICR_ARROKCF;
    return el;
}

void pwr_hw_init(void)
{
    uint32_t t0;

    eink_sleep_init();                          /* TIM6 для Sleep (повторный вызов безвреден) */

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();                 /* LSION лежит в RCC_BDCR (домен резервного питания) */
    SET_BIT(RCC->BDCR, RCC_BDCR_LSION);
    t0 = HAL_GetTick();
    while ((RCC->BDCR & RCC_BDCR_LSIRDY) == 0U && (HAL_GetTick() - t0) < 100U) {
    }

    __HAL_RCC_LPTIM1_CLK_ENABLE();
    __HAL_RCC_LPTIM1_CLK_SLEEP_ENABLE();
    __HAL_RCC_LPTIM1_CLKAM_ENABLE();            /* LPTIM1 работает и в Stop 2 */
    MODIFY_REG(RCC->CCIPR3, RCC_CCIPR3_LPTIM1SEL, RCC_CCIPR3_LPTIM1SEL_0);   /* 01 = LSI */
}

/* ---------------------------------- Stop 2 ---------------------------------- */

static uint32_t stop2_wait(uint32_t ms, int on_busy)
{
    uint32_t elapsed;

    if (ms > LPTIM_MAX_MS)
        ms = LPTIM_MAX_MS;

    __disable_irq();                            /* с PRIMASK WFI всё равно просыпается по ожидающему IRQ */

    if (on_busy && !eink_busy_arm()) {          /* BUSY уже низкий: опрос нужен сейчас */
        eink_busy_disarm();
        __enable_irq();
        return 0;
    }

    lptim_start(ms);
    NVIC_ClearPendingIRQ(LPTIM1_IRQn);
    NVIC_EnableIRQ(LPTIM1_IRQn);

    HAL_SuspendTick();
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;         /* ожидающий SysTick разбудил бы МК сразу */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);   /* после Stop - сразу HSI16, как и до него */
    __DSB();
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    elapsed = lptim_stop(ms);
    NVIC_DisableIRQ(LPTIM1_IRQn);
    NVIC_ClearPendingIRQ(LPTIM1_IRQn);
    if (on_busy)
        eink_busy_disarm();

    uwTick += elapsed;                          /* HAL-тик (мс) догоняем по LPTIM1 */
    HAL_ResumeTick();
    __enable_irq();
    return elapsed;
}

/* ---------------------------------- ожидание -------------------------------- */

static uint32_t run_wait(uint32_t ms, int on_busy)
{
    const uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < ms) {
        if (on_busy && HAL_GPIO_ReadPin(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin) == GPIO_PIN_RESET)
            break;
    }
    return HAL_GetTick() - t0;
}

uint32_t pwr_wait(pwr_cpu_t cpu, uint32_t ms, int on_busy)
{
    if (ms == 0U)
        return 0;

    switch (cpu) {
    case PWR_CPU_SLEEP:
        return eink_sleep(ms, on_busy);
    case PWR_CPU_STOP2:
        if (ms < PWR_STOP_MIN_MS)
            return eink_sleep(ms, on_busy);
        return stop2_wait(ms, on_busy);
    case PWR_CPU_RUN:
    default:
        return run_wait(ms, on_busy);
    }
}
