/*
 * DEV_Config.h — порт аппаратного слоя библиотеки Waveshare (e-Paper) на STM32 HAL.
 * Пины берутся из меток CubeMX: EINK_CS, EINK_DC, EINK_RST, EINK_BUSY.
 */
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include <stdint.h>
#include "main.h"

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/* Логические номера пинов (их ждёт библиотека Waveshare) */
#define EPD_RST_PIN   0
#define EPD_DC_PIN    1
#define EPD_CS_PIN    2
#define EPD_BUSY_PIN  3
#define EPD_PWR_PIN   4   /* на этой плате питание не коммутируется */

void  DEV_Digital_Write(UWORD Pin, UBYTE Value);
UBYTE DEV_Digital_Read(UWORD Pin);
void  DEV_SPI_WriteByte(UBYTE Value);
void  DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE Len);
void  DEV_Delay_ms(UDOUBLE xms);
UBYTE DEV_Module_Init(void);
void  DEV_Module_Exit(void);

#endif
