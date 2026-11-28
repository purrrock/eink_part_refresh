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
#include "epd4g_bw.h"

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

/** Отдельные линии (для pwr_pin_set(); PWR_PIN_SPI = SCK и MOSI вместе). */
typedef enum { PWR_PIN_RST = 0, PWR_PIN_CS, PWR_PIN_DC, PWR_PIN_BUSY, PWR_PIN_SPI } pwr_pin_t;
typedef enum { PWR_PIN_LOW = 0, PWR_PIN_HIGH, PWR_PIN_ANALOG } pwr_pinmode_t;

/** Один раз после MX_xxx_Init(): LSI, LPTIM1, TIM6 (eink_sleep_init). */
void pwr_hw_init(void);

/** Перевести линии в состояние @p mode (см. pwr_lines_t). Нельзя вызывать во время обмена по SPI. */
void pwr_lines_set(pwr_lines_t mode, unsigned flags);

/**
 * Переопределить одну линию поверх pwr_lines_set(HIGH/LOW/ANALOG, ...) (SPI1 уже должен быть
 * выключен). BUSY - вход: LOW/HIGH возвращают его цифровым входом, ANALOG отключает.
 */
void pwr_pin_set(pwr_pin_t pin, pwr_pinmode_t mode);

/**
 * Сон HAT (около 0.3 мА вместо ~1.9 мА, измерено). Делает всё по порядку:
 *   1. epd4g_power_off() - аналог панели выключается (с включённым аналогом картинка бледнеет);
 *   2. запоминает режим панели (4 gray или 1-bit);
 *   3. RST = 0, CS/DC/SCK/MOSI = 0, SPI1 выключен и без тактирования, BUSY в analog.
 * Картинка на панели сохраняется, RAM контроллера - нет. Потом можно уходить в Stop 2. Повторный вызов
 * ничего не делает. Возвращает 0 или код ошибки epd4g (при ошибке power off HAT не усыпляется).
 * Из этого состояния панель выводит только pwr_hat_wake(): любая другая работа с epd4g_xxx() до неё
 * бессмысленна.
 */
int pwr_hat_sleep(void);

/**
 * Пробуждение HAT: линии в исходное состояние (RST, CS, DC = 1, SPI1 включён, BUSY - вход), затем
 *   4 gray: epd4g_init() (аппаратный сброс, ~50 мс); img и src не используются (можно NULL);
 *   1-bit:  epd4g_bw_restore(img, src): сброс, init и картинка, которая сейчас на стекле, в обе плоскости RAM
 *           без обновления экрана (~0.1 с, без вспышки). img обязательна и должна совпадать с тем, что
 *           показано на панели (при 1-bit это кадр приложения после последнего завершённого обновления).
 * После этого можно сразу делать обычное частичное обновление. Если HAT не спал, ничего не делает.
 * Возвращает 0 или код ошибки epd4g.
 */
int pwr_hat_wake(const void *img, epd4g_src_t src);

/** 1 - между pwr_hat_sleep() и pwr_hat_wake(). */
int pwr_hat_asleep(void);

/** 1 - дождаться конца вывода в UART и остановить USART1/USART2; 0 - включить обратно. */
void pwr_quiet(int on);

/**
 * Ждать не дольше @p ms миллисекунд в режиме @p cpu. При on_busy != 0 вернуться раньше, когда
 * BUSY упадёт в 0 (вход BUSY должен быть цифровым). Возвращает, сколько мс прошло: HAL_GetTick()
 * после вызова уже продвинут на это время (в Sleep и Stop 2 - по таймеру, с точностью ~1 мс).
 */
uint32_t pwr_wait(pwr_cpu_t cpu, uint32_t ms, int on_busy);

#endif /* EINK_PWR_H */
