/**
 * @file    debug_uart.h
 * @brief   Выбор USART отладочной консоли (printf).
 *
 * Номер USART задаётся макросом EINK_DEBUG_USART:
 *   1 - USART1 (по умолчанию), PA9 (TX) / PA10 (RX)
 *   2 - USART2,                PA2 (TX) / PA3  (RX)
 *
 * Значение можно переопределить при сборке:  cmake -DEINK_DEBUG_USART=2 ...
 * (см. CMakeLists.txt) либо #define до подключения этого файла.
 * Выбранный USART должен быть включён в CubeMX (Asynchronous, 115200 8N1).
 */
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "usart.h"

#ifndef EINK_DEBUG_USART
#define EINK_DEBUG_USART 1
#endif

#if EINK_DEBUG_USART == 1
#define DBG_UART_HANDLE   huart1
#define DBG_UART_NAME     "USART1"
#elif EINK_DEBUG_USART == 2
#define DBG_UART_HANDLE   huart2
#define DBG_UART_NAME     "USART2"
#else
#error "EINK_DEBUG_USART должен быть 1 или 2"
#endif

#endif /* DEBUG_UART_H */
