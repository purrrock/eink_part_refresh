/**
 * @file    eink_pwr.h
 * @brief   Низкоуровневые режимы энергосбережения для теста тока (STM32U585 + Waveshare 2.7" HAT).
 *
 * Используется demo/eink_power_test.c. Три независимые «ручки»:
 *
 *   pwr_lines_set()   состояние сигнальных линий дисплея: SPI работает / остановлен и линии
 *                     подтянуты в 0 / в 1 / отключены (analog);
 *   pwr_wait()        чем занят процессор, пока панель рисует или стоит: крутится (RUN), Sleep (WFI)
 *                     или Stop 2 (будильник LPTIM1 от LSI, пробуждение ещё и по спаду BUSY);
 *   pwr_quiet()       остановить USART1/USART2 (отладочный вывод), чтобы они не вносили вклад в ток.
 *
 * Изменений в CubeMX не требуется: LPTIM1, LSI и EXTI2 настраиваются здесь на время сна.
 */
#ifndef EINK_PWR_H
#define EINK_PWR_H

#include <stdint.h>

/** Состояние сигнальных линий дисплея (PA5 SCK, PA7 MOSI, PB7 CS, PB8 DC, PB9 RST, PB2 BUSY). */
typedef enum {
    PWR_LINES_ACTIVE = 0,   /**< SPI1 включён и тактируется, линии как оставила библиотека        */
    PWR_LINES_LOW    = 1,   /**< SPI1 выключен и без тактирования; SCK, MOSI, DC = 0; CS, RST = 1 */
    PWR_LINES_HIGH   = 2,   /**< SPI1 выключен и без тактирования; все пять линий = 1             */
    PWR_LINES_ANALOG = 3    /**< SPI1 выключен и без тактирования; линии в analog (Hi-Z)          */
} pwr_lines_t;

/** Что делает процессор в паузах. */
typedef enum {
    PWR_CPU_RUN   = 0,      /**< крутится в цикле на HSI 16 МГц (эталон «без экономии»)           */
    PWR_CPU_SLEEP = 1,      /**< Sleep (WFI), SysTick остановлен, будят TIM6 и спад BUSY          */
    PWR_CPU_STOP2 = 2       /**< Stop 2, будят LPTIM1 (LSI) и спад BUSY; короткие паузы - Sleep   */
} pwr_cpu_t;

/** Флаги pwr_lines_set(). */
#define PWR_F_BUSY      1u  /**< в режиме ANALOG отключить и BUSY (в остальных режимах - вернуть вход)   */
#define PWR_F_KEEP_CTL  2u  /**< не трогать CS и RST: библиотека держит RST низким в импульсе сброса     */

/** Один раз после MX_xxx_Init(): LSI, LPTIM1, TIM6 (eink_sleep_init). */
void pwr_hw_init(void);

/** Перевести линии в состояние @p mode (см. pwr_lines_t). Нельзя вызывать во время обмена по SPI. */
void pwr_lines_set(pwr_lines_t mode, unsigned flags);

/** 1 - дождаться конца вывода в UART и остановить USART1/USART2; 0 - включить обратно. */
void pwr_quiet(int on);

/**
 * Ждать не дольше @p ms миллисекунд в режиме @p cpu. При on_busy != 0 вернуться раньше, когда
 * BUSY упадёт в 0 (вход BUSY должен быть цифровым). Возвращает, сколько мс прошло: HAL_GetTick()
 * после вызова уже продвинут на это время (в Sleep и Stop 2 - по таймеру, с точностью ~1 мс).
 */
uint32_t pwr_wait(pwr_cpu_t cpu, uint32_t ms, int on_busy);

#endif /* EINK_PWR_H */
