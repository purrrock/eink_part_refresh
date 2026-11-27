/*
 * eink_sleep.c - сон МК по epd4g_wait_hint() (STM32U585). См. eink_sleep.h.
 */
#include "eink_sleep.h"

#include "main.h"

_Static_assert(EINK_BUSY_Pin == GPIO_PIN_2,
               "eink_sleep.c: EINK_BUSY must be on EXTI line 2 (vector name and IRQn below)");

#define TIM_TICK_HZ     10000u                  /* TIM6 считает по 0,1 мс                      */
#define TICKS_PER_MS    (TIM_TICK_HZ / 1000u)
#define MAX_SLEEP_MS    6000u                   /* ARR 16 бит: 65535 / 10 = 6553 мс            */

static uint32_t s_frac;                         /* остаток < 1 мс (в тиках 0,1 мс) между снами */
static uint32_t s_count;
static uint32_t s_total_ms;

/* ------------------------------- прерывания -------------------------------- */

/* Обработчики только гасят флаги: просыпается МК, а работу делает основной цикл. */
void TIM6_IRQHandler(void)
{
    TIM6->SR = 0;
}

void EXTI2_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(EINK_BUSY_Pin);
}

/* ---------------------------------- BUSY ----------------------------------- */

static void busy_pin_input(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* DeInit снимает и линию EXTI: активная линия будила бы МК при каждом спаде BUSY. */
    HAL_GPIO_DeInit(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin);

    gpio.Pin = EINK_BUSY_Pin;                   /* как в CubeMX (gpio.c) */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EINK_BUSY_GPIO_Port, &gpio);
}

/* 1 - линия взведена и BUSY ещё высокий; 0 - BUSY уже низкий (спад упущен), спать нельзя. */
static int busy_wake_arm(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = EINK_BUSY_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EINK_BUSY_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(EINK_BUSY_Pin);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);

    /* уровень читается уже после настройки EXTI: спад до неё не разбудил бы */
    return HAL_GPIO_ReadPin(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin) == GPIO_PIN_SET;
}

static void busy_wake_disarm(void)
{
    NVIC_DisableIRQ(EXTI2_IRQn);
    busy_pin_input();
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(EINK_BUSY_Pin);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

/* ---------------------------------- TIM6 ----------------------------------- */

void eink_sleep_init(void)
{
    uint32_t clk = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR2 & RCC_CFGR2_PPRE1_2) != 0U)  /* делитель APB1 != 1: таймер работает на 2*PCLK1 */
        clk *= 2U;

    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->CR1 = 0;
    TIM6->PSC = (uint16_t)(clk / TIM_TICK_HZ - 1U);
    TIM6->DIER = 0;
    TIM6->SR = 0;
}

static void alarm_start(uint32_t ms)
{
    TIM6->CR1 = TIM_CR1_OPM | TIM_CR1_URS;      /* один проход; UG не поднимает флаг обновления */
    TIM6->ARR = ms * TICKS_PER_MS - 1U;
    TIM6->EGR = TIM_EGR_UG;                     /* загрузить PSC/ARR, CNT = 0 */
    TIM6->SR = 0;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM6_IRQn);
    NVIC_EnableIRQ(TIM6_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

/* Останавливает будильник, возвращает прошедшее время в тиках 0,1 мс. */
static uint32_t alarm_stop(uint32_t ms)
{
    /* в режиме OPM по концу счёта CNT обнуляется, поэтому истечение определяется по флагу */
    const uint32_t ticks = ((TIM6->SR & TIM_SR_UIF) != 0U) ? ms * TICKS_PER_MS : TIM6->CNT;

    TIM6->CR1 = 0;
    TIM6->DIER = 0;
    TIM6->SR = 0;
    NVIC_DisableIRQ(TIM6_IRQn);
    NVIC_ClearPendingIRQ(TIM6_IRQn);
    return ticks;
}

/* ----------------------------------- сон ----------------------------------- */

uint32_t eink_sleep(uint32_t ms, int on_busy)
{
    uint32_t ticks, total, elapsed;

    if (ms == 0U)
        return 0;
    if (ms > MAX_SLEEP_MS)
        ms = MAX_SLEEP_MS;

    __disable_irq();                            /* с PRIMASK WFI всё равно просыпается по ожидающему IRQ */

    if (on_busy && !busy_wake_arm()) {          /* BUSY уже низкий: опрос нужен сейчас */
        busy_wake_disarm();
        __enable_irq();
        return 0;
    }

    alarm_start(ms);
    HAL_SuspendTick();
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;         /* уже ожидающий SysTick разбудил бы МК сразу */
    __DSB();
    __WFI();

    ticks = alarm_stop(ms);
    if (on_busy)
        busy_wake_disarm();

    /* HAL-тик (мс) догоняем по TIM6, остаток 0,1 мс переносим на следующий сон */
    total = s_frac + ticks;
    elapsed = total / TICKS_PER_MS;
    s_frac = total % TICKS_PER_MS;
    uwTick += elapsed;

    HAL_ResumeTick();
    __enable_irq();

    s_count++;
    s_total_ms += elapsed;
    return elapsed;
}

void eink_sleep_stats(uint32_t *count, uint32_t *total_ms)
{
    if (count != NULL)
        *count = s_count;
    if (total_ms != NULL)
        *total_ms = s_total_ms;
}

/* Для eink_pwr.c (Stop 2): те же взвод и снятие пробуждения по спаду BUSY. */
int eink_busy_arm(void)
{
    return busy_wake_arm();
}

void eink_busy_disarm(void)
{
    busy_wake_disarm();
}
