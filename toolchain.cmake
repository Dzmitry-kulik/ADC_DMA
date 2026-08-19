set(CMAKE_SYSTEM_NAME Generic)      # Для bare-metal (без ОС). Или Linux, Darwin, Windows
set(CMAKE_SYSTEM_PROCESSOR arm)

# 2. Префиксы/пути к инструментам
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# 3. Указываем явные пути к компиляторам
set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)

# 4. Настройка поиска библиотек и заголовков (чтобы CMake не искал хостовые системные библиотеки)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) # Программы ищем на хосте
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)  # Библиотеки ищем только в sysroot целевой системы
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)  # Заголовки ищем только в sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

add_compile_options(
    --specs=nosys.specs
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Опции линковки
add_link_options(
    --specs=nosys.specs
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)