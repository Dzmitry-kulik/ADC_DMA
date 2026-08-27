cmake_minimum_required(VERSION 3.22)
project(PMW_CONFIG C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Опции архитектуры ядра
set(CPU_OPTIONS
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
)

# Объявляем исполняемый файл прошивки
add_executable(${PROJECT_NAME}.elf)

# Автоматический поиск исходников библиотеки HAL (подхватит ADC, когда вы его скачаете)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS "Drivers/STM32F4xx_HAL_Driver/Src/*.c")

# Исходные файлы проекта
target_sources(${PROJECT_NAME}.elf PRIVATE
    # Системные файлы и инициализация
    Core/Src/main.cpp
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f4xx.c
    Core/Startup/startup_stm32f411ceux.s

    # Файлы протокола
    Core/Src/protocol/Core/Src/protocol/src/circular_buf.cpp
    Core/Src/protocol/Core/Src/protocol/src/diagnostics.cpp
    Core/Src/protocol/Core/Src/protocol/src/FSM_parser.cpp
    Core/Src/protocol/Core/Src/protocol/src/tx_manager.cpp

    # Исходники HAL
    ${HAL_SOURCES}
)

# Пути к заголовочным файлам (.h, .hpp)
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    Core/Inc
    Core/Src/protocol/Core/Src/protocol/inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

# Макросы препроцессора
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    STM32F411xE
    USE_HAL_DRIVER
)

# Флаги компилятора
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    ${CPU_OPTIONS}
    -O0
    -g3
    -Wno-unused-parameter
    -fdata-sections
    -ffunction-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

# Флаги линковщика
target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${CPU_OPTIONS}
    -specs=nano.specs
    -u _printf_float    # Включает поддержку %f в printf/snprintf для float
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F411CEUX_FLASH.ld
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segments
)