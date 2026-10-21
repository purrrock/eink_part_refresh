#include "DEV_Config.h"

extern SPI_HandleTypeDef hspi1;

void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
    GPIO_PinState s = Value ? GPIO_PIN_SET : GPIO_PIN_RESET;
    switch (Pin) {
    case EPD_RST_PIN: HAL_GPIO_WritePin(EINK_RST_GPIO_Port, EINK_RST_Pin, s); break;
    case EPD_DC_PIN:  HAL_GPIO_WritePin(EINK_DC_GPIO_Port,  EINK_DC_Pin,  s); break;
    case EPD_CS_PIN:  HAL_GPIO_WritePin(EINK_CS_GPIO_Port,  EINK_CS_Pin,  s); break;
    default: break;
    }
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
    if (Pin == EPD_BUSY_PIN) {
        return (HAL_GPIO_ReadPin(EINK_BUSY_GPIO_Port, EINK_BUSY_Pin) == GPIO_PIN_SET) ? 1 : 0;
    }
    return 0;
}

void DEV_SPI_WriteByte(UBYTE Value)
{
    HAL_SPI_Transmit(&hspi1, &Value, 1, 100);
}

void DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE Len)
{
    while (Len) {
        uint16_t n = (Len > 0xFFFF) ? 0xFFFF : (uint16_t)Len;
        HAL_SPI_Transmit(&hspi1, pData, n, 1000);
        pData += n;
        Len   -= n;
    }
}

void DEV_Delay_ms(UDOUBLE xms)
{
    HAL_Delay(xms);
}

UBYTE DEV_Module_Init(void)
{
    DEV_Digital_Write(EPD_CS_PIN, 1);
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    return 0;
}

void DEV_Module_Exit(void)
{
    DEV_Digital_Write(EPD_CS_PIN, 1);
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_RST_PIN, 0);
}
