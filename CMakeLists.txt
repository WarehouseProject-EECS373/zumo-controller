cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/modules/stm32-cmake/cmake/stm32_gcc.cmake)

project(zumo-controller C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)


add_compile_options(-Wall -Wextra -Wpedantic -Werror)


add_definitions(-DSTM32F401xE)

stm32_fetch_cmsis(F4)
stm32_fetch_hal(F4)

find_package(CMSIS COMPONENTS STM32F401RE REQUIRED)
find_package(HAL COMPONENTS STM32F4 REQUIRED)

set(CMAKE_ASM_FLAGS -c)

set(PROJECT_SOURCES
    src/app_defs.h
    
    src/main.c
    src/watchdog.c
    src/os_port_arm_m4.c
    src/rmk_hal_clock_cfg.c
    
    src/subsys/drive_subsystem.c
    src/subsys/input_ctl_subsystem.c
    src/subsys/reflectance_array_subsystem.c
)

set(STM_SOURCES
    src/stm/startup_stm32f401xe.s
    src/stm/system_stm32f4xx.c
    src/stm32f4xx_hal_conf.h
    src/stm/stm32f401xe.h
    src/stm/system_stm32f4xx.h
)
    
add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${STM_SOURCES}
)


add_subdirectory(modules/rmkernel)

target_link_libraries(${PROJECT_NAME} PRIVATE
    rmkernel
)



target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    HAL::STM32::F4::RCC
    HAL::STM32::F4::CORTEX
    HAL::STM32::F4::GPIO
    HAL::STM32::F4::TIM
    HAL::STM32::F4::TIMEx
    CMSIS::STM32::F4
    STM32::NoSys)

stm32_add_linker_script(${PROJECT_NAME} PRIVATE F401RE.ld)

stm32_generate_binary_file(${PROJECT_NAME})
stm32_generate_hex_file(${PROJECT_NAME})
stm32_print_size_of_target(${PROJECT_NAME})
