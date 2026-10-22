# epd4g - 4-gray partial / fast full refresh for the Waveshare 2.7" e-Paper HAT V2.
#
# Usage from the parent project:
#     add_subdirectory(path/to/epd4g)
#     target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE epd4g)
#
# This is an INTERFACE library: its sources are compiled inside the consuming target, so they
# get the toolchain flags, include paths (HAL, main.h) and definitions of your firmware.
#
# Options (set before add_subdirectory):
#   EPD4G_PORT_STM32_HAL  ON  (default) add the STM32 HAL port; OFF = write your own epd4g_port.h functions
#   EPD4G_PROFILE         OFF  per-phase timing, see epd4g_get_profile()
# Tunables (waveforms, power, timeouts) are compile definitions, see include/epd4g_config.h.
cmake_minimum_required(VERSION 3.22)

option(EPD4G_PORT_STM32_HAL "Build the STM32 HAL port" ON)
option(EPD4G_PROFILE "Record per-phase update timings" OFF)

add_library(epd4g INTERFACE)
target_include_directories(epd4g INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_sources(epd4g INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/epd4g.c
    ${CMAKE_CURRENT_LIST_DIR}/src/epd4g_fb.c
)
if(EPD4G_PORT_STM32_HAL)
    target_sources(epd4g INTERFACE ${CMAKE_CURRENT_LIST_DIR}/port/stm32_hal/epd4g_port_stm32_hal.c)
endif()
if(EPD4G_PROFILE)
    target_compile_definitions(epd4g INTERFACE EPD4G_PROFILE)
endif()
