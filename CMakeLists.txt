cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME eink_part_refresh)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------------------------------------------------------------------------
# Библиотека epd4g (папка epd4g/) - частичное и быстрое полное обновление 4 gray.
# Самодостаточная: Waveshare-библиотека и DEV_Config больше не нужны.
# ---------------------------------------------------------------------------
# Опции демо-прошивки (cmake -DEINK_DIAG=OFF ...):
option(EINK_DIAG    "Печатать журнал демо в USART2 (printf)" ON)
option(EINK_PROFILE "Печатать время по фазам каждого обновления (нужен EINK_DIAG)" OFF)

set(EPD4G_PROFILE ${EINK_PROFILE})          # передаём опцию библиотеке
add_subdirectory(epd4g)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/eink_demo.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    EINK_DIAG=$<BOOL:${EINK_DIAG}>
    EINK_PROFILE=$<BOOL:${EINK_PROFILE}>
    # Настройки библиотеки (все необязательные, значения по умолчанию - в epd4g/include/epd4g_config.h):
    # EPD4G_KEEP_POWER=0
    # EPD4G_FULL_SHAKE_RP=1
    # EPD4G_T_BLACK=45
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    epd4g
)
