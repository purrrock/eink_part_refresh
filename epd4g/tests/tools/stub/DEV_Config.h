/* Minimal stand-in for the Waveshare DEV_Config.h, only for tools/gen_golden.c */
#ifndef DEV_CONFIG_STUB_H
#define DEV_CONFIG_STUB_H
#include <stdint.h>
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t
#define EPD_RST_PIN  1
#define EPD_DC_PIN   2
#define EPD_CS_PIN   3
#define EPD_BUSY_PIN 4
void  DEV_Digital_Write(UWORD pin, UBYTE value);
UBYTE DEV_Digital_Read(UWORD pin);
void  DEV_SPI_WriteByte(UBYTE value);
void  DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE len);
void  DEV_Delay_ms(UDOUBLE ms);
#endif
