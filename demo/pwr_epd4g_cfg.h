/*
 * pwr_epd4g_cfg.h - настройки библиотеки epd4g для теста тока (подключается через
 * -DEPD4G_USER_CONFIG="pwr_epd4g_cfg.h", см. eink_u585/CMakeLists.txt, опция EINK_POWER_TEST).
 *
 * EPD4G_KEEP_POWER становится переменной: тест переключает «аналог панели остаётся включённым /
 * выключается после каждого обновления» без пересборки. Автоотключение аналога библиотекой
 * выключено: питание панели в тесте меняется только явными командами.
 */
#ifndef PWR_EPD4G_CFG_H
#define PWR_EPD4G_CFG_H

extern volatile unsigned char g_pwr_keep_gray;      /* 1 - 0xC4 (аналог включён), 0 - 0xC7 (выключается) */

#define EPD4G_KEEP_POWER        (g_pwr_keep_gray)
#define EPD4G_AUTO_POWEROFF_MS  0

/* 1-bit: по умолчанию штатно (аналог выключается после каждого обновления). Вариант с включённым
 * аналогом в библиотеке помечен «не проверено на железе»: -DPWR_TEST_BW_KEEP=1. */
#if defined(PWR_TEST_BW_KEEP) && PWR_TEST_BW_KEEP
#define EPD4G_BW_KEEP_POWER     1
#define EPD4G_BW_PARTIAL_RESET  0
#endif

#endif
