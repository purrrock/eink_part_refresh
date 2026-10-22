/**
 * @file epd4g_port.h
 * @brief Hardware abstraction the epd4g library needs. Implement these 7 functions
 *        for your platform (a ready STM32 HAL port is in port/stm32_hal/).
 *
 * Wiring (Waveshare 2.7" HAT V2): SPI mode 0 (CPOL=0, CPHA=0), MSB first, 8 bit,
 * TX only is enough. CS is a plain GPIO driven by the library. SPI clock <= 10 MHz
 * is safe (the reference project runs at 8 MHz).
 */
#ifndef EPD4G_PORT_H
#define EPD4G_PORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     epd4g_port_gpio_rst(int level);           /**< RST pin: 0 = low, 1 = high         */
void     epd4g_port_gpio_dc(int level);            /**< D/C pin: 0 = command, 1 = data     */
void     epd4g_port_gpio_cs(int level);            /**< CS pin (software chip select)       */
int      epd4g_port_gpio_busy(void);               /**< BUSY pin: 1 = panel busy, 0 = ready */

/** Blocking SPI transmit. Returns 0 on success, non-zero on error (-> EPD4G_ERR_IO). */
int      epd4g_port_spi_write(const uint8_t *data, size_t len);

void     epd4g_port_delay_ms(uint32_t ms);         /**< may yield to an RTOS scheduler      */
uint32_t epd4g_port_tick_ms(void);                 /**< monotonic millisecond counter (wraps ok) */

#ifdef __cplusplus
}
#endif

#endif /* EPD4G_PORT_H */
