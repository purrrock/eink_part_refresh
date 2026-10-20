У меня есть дисплей "Waveshare 2.7''e-paper,264x176,2.7inch E-Ink display HAT" V2 supports partial refreshing and fast refreshing, 4 gray levels.
Он подключен к STM32F411CEU6:
| Контакт дисплея | Пин STM32F411CEU6 | Описание и настройка в CubeMX |
|---|---|---|---|
| VCC | 3V3 | Питание |
| GND | GND | Общая земля |
| DIN | PA7 | SPI1_MOSI |
| CLK | PA5 | SPI1_SCK |
| CS | PB7 | GPIO_Output, `EINK_CS` |
| DC | PB8 | GPIO_Output, `EINK_DC` |
| RST | PB9 | GPIO_Output, `EINK_RST` |
| BUSY | PB2 | GPIO_Input, `EINK_BUSY` |

Для E-Ink не используется SPI1_MISO.

Документация https://www.waveshare.com/wiki/2.7inch_e-Paper_HAT_Manual описывает partial refresh только для режима 1-bit. А мне очень нужен partial refresh для режима 2-bit (4 gray levels).