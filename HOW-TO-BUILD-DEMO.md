# Как собрать демо

Демо (`demo/eink_demo.c` + библиотека `epd4g/`) собирается в двух проектах:

| MCU | Каталог проекта | CubeMX-файл |
|---|---|---|
| STM32F411CEU6 | корень репозитория | `eink_part_refresh.ioc` |
| STM32U585CIU6 | `eink_u585/` | `eink_u585/eink_u585.ioc` |

Сабмодуль Waveshare для демо не нужен.

## Что нужно

* VS Code + расширение **STM32Cube** (оно ставит/подтягивает `arm-none-eabi-gcc`, CMake, Ninja, OpenOCD).
* Без VS Code: `arm-none-eabi-gcc`, `cmake` ≥ 3.22, `ninja`.

```sh
git clone https://github.com/purrrock/eink_part_refresh
cd eink_part_refresh
```

## Подключение дисплея (одинаково для обоих MCU)

| HAT | MCU | HAT | MCU |
|---|---|---|---|
| VCC | 3V3 | CS | PB7 |
| GND | GND | DC | PB8 |
| DIN | PA7 (SPI1_MOSI) | RST | PB9 |
| CLK | PA5 (SPI1_SCK) | BUSY | PB2 |

Журнал (`printf`): USART1, PA9 (TX) / PA10 (RX), 115200 8N1.

## STM32F411

**VS Code:** открыть корень репозитория → в панели STM32Cube выбрать проект → пресет `Debug` → *Build* → *Run/Debug*.

**Командная строка:**

```sh
cmake --preset Debug
cmake --build --preset Debug
# результат: build/Debug/eink_part_refresh.elf
```

## STM32U585

**VS Code:** открыть папку `eink_u585/` как отдельный проект (File → Open Folder) → пресет `Debug` → *Build* → *Run/Debug*.

**Командная строка:**

```sh
cd eink_u585
cmake --preset Debug
cmake --build --preset Debug
# результат: eink_u585/build/Debug/eink_u585.elf
```

U585-проект берёт `../demo/eink_demo.c` и `../epd4g`, поэтому собирать его нужно из полного клона репозитория.

## Опции CMake (оба проекта)

Передаются при конфигурации: `cmake --preset Debug -DEINK_DIAG=OFF`.

| Опция | По умолчанию | Смысл |
|---|---|---|
| `EINK_DIAG` | ON | журнал в USART (printf) |
| `EINK_DEBUG_USART` | 1 | `1` = USART1 (PA9/PA10), `2` = USART2 (PA2/PA3) |
| `EINK_PROFILE` | OFF | время по фазам обновления (нужен `EINK_DIAG`) |
| `EINK_SLEEP` | ON | только U585: сон между опросами по `epd4g_wait_hint()` |

После смены опции в VS Code: *CMake: Delete Cache and Reconfigure*.

## Если что-то не собирается

* **Смена опции не применилась** — удалить каталог `build/` и сконфигурировать заново.
* **После перегенерации CubeMX** метки пинов должны остаться `EINK_CS`, `EINK_DC`, `EINK_RST`, `EINK_BUSY`, а `printf` в `main.c` требует `#include <stdio.h>` внутри `USER CODE BEGIN Includes`.
* **Нет вывода в консоль** — проверить, что выбранный USART включён в CubeMX (Asynchronous, 115200) и совпадает с `EINK_DEBUG_USART`.
