#ifndef EINK_DEMO_H
#define EINK_DEMO_H

#include <stdint.h>

/* Инициализация дисплея, полная 4-gray отрисовка фона из квадратов. */
/* Returns 0 on success, -1 on a display/SPI/BUSY error. */
int eink_demo_init(void);

/* Неблокирующий шаг демо: вызывать из while(1) как можно чаще. Возвращает управление за ~1 мс.
 * Когда панель свободна, запускает следующее обновление (кадр 4 gray, либо шаг 1-bit меню). */
/* Returns 0 on success, -1 on a display/SPI/BUSY error. */
int eink_demo_task(void);

/* Для низкого энергопотребления: сколько мс главный цикл может спать до следующего eink_demo_task()
 * (подсказка epd4g_wait_hint()). Вызывать сразу после eink_demo_task().
 *   0     - вызвать eink_demo_task() сейчас (идёт передача данных по SPI, ожидание закончилось);
 *   N > 0 - можно спать до N мс; *on_busy (может быть NULL) = 1, если задача ждёт BUSY панели:
 *           тогда можно проснуться раньше, по спаду BUSY (PB2). Ранний подъём безвреден. */
uint32_t eink_demo_wait_ms(int *on_busy);

/* Длительность последнего обновления (мс) — удобно смотреть в отладчике. */
uint32_t eink_demo_last_update_ms(void);

#endif
