/**
 * @file epd4g_port_stm32_hal.c
 * @brief epd4g port for STM32 HAL (any family), software CS, blocking SPI.
 *
 * Defaults match the CubeMX project in this repository: SPI handle `hspi1` and GPIO
 * labels EINK_RST / EINK_DC / EINK_CS / EINK_BUSY (CubeMX generates EINK_xxx_GPIO_Port and
 * EINK_xxx_Pin in main.h). To use other names or pins, override any of the macros below
 * from the build system or from a user config header (see EPD4G_USER_CONFIG).
 *
 * CubeMX settings: SPI Full-Duplex or Transmit-Only Master, 8 bit, MSB first, CPOL low,
 * CPHA 1 Edge, NSS software, baud <= 10 MHz. CS/DC/RST: GPIO output, BUSY: GPIO input.
 */
#include "epd4g_port.h"
#include "epd4g_config.h"

#ifndef EPD4G_HAL_HEADER
#define EPD4G_HAL_HEADER       "main.h"        /* includes the device HAL header and the pin labels */
#endif
#include EPD4G_HAL_HEADER

#ifndef EPD4G_SPI_HANDLE
#define EPD4G_SPI_HANDLE       hspi1
#endif
#ifndef EPD4G_RST_PORT
#define EPD4G_RST_PORT         EINK_RST_GPIO_Port
#define EPD4G_RST_PIN          EINK_RST_Pin
#endif
#ifndef EPD4G_DC_PORT
#define EPD4G_DC_PORT          EINK_DC_GPIO_Port
#define EPD4G_DC_PIN           EINK_DC_Pin
#endif
#ifndef EPD4G_CS_PORT
#define EPD4G_CS_PORT          EINK_CS_GPIO_Port
#define EPD4G_CS_PIN           EINK_CS_Pin
#endif
#ifndef EPD4G_BUSY_PORT
#define EPD4G_BUSY_PORT        EINK_BUSY_GPIO_Port
#define EPD4G_BUSY_PIN         EINK_BUSY_Pin
#endif
#ifndef EPD4G_SPI_TIMEOUT_MS
#define EPD4G_SPI_TIMEOUT_MS   1000U
#endif

extern SPI_HandleTypeDef EPD4G_SPI_HANDLE;

void epd4g_port_gpio_rst(int level)
{
    HAL_GPIO_WritePin(EPD4G_RST_PORT, EPD4G_RST_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void epd4g_port_gpio_dc(int level)
{
    HAL_GPIO_WritePin(EPD4G_DC_PORT, EPD4G_DC_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void epd4g_port_gpio_cs(int level)
{
    HAL_GPIO_WritePin(EPD4G_CS_PORT, EPD4G_CS_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int epd4g_port_gpio_busy(void)
{
    return HAL_GPIO_ReadPin(EPD4G_BUSY_PORT, EPD4G_BUSY_PIN) == GPIO_PIN_SET;
}

int epd4g_port_spi_write(const uint8_t *data, size_t len)
{
    while (len > 0) {
        uint16_t n = (len > 0xFFFFu) ? 0xFFFFu : (uint16_t)len;
        if (HAL_SPI_Transmit(&EPD4G_SPI_HANDLE, (uint8_t *)data, n, EPD4G_SPI_TIMEOUT_MS) != HAL_OK)
            return -1;
        data += n;
        len  -= n;
    }
    return 0;
}

void epd4g_port_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint32_t epd4g_port_tick_ms(void)
{
    return HAL_GetTick();
}
