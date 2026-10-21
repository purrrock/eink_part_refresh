#ifndef EINK_DEMO_H
#define EINK_DEMO_H

#include <stdint.h>

/* Инициализация дисплея, полная 4-gray отрисовка фона из квадратов. */
/* Returns 0 on success, -1 on a display/SPI/BUSY error. */
int eink_demo_init(void);

/* Один шаг анимации: сдвиг треугольника + частичное обновление окна. Блокирует до конца BUSY. */
/* Returns 0 on success, -1 on a display/SPI/BUSY error. */
int eink_demo_step(void);

/* Длительность последнего обновления (мс) — удобно смотреть в отладчике. */
uint32_t eink_demo_last_update_ms(void);

#endif
