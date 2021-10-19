cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/modules/stm32-cmake/cmake/stm32_gcc.cmake)

project(zumo-controller C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

stm32_fetch_cmsis(L4)
stm32_fetch_hal(L4)

find_package(CMSIS COMPONENTS STM32L4R5ZI REQUIRED)
find_package(HAL COMPONENTS STM32L4 REQUIRED)

set(CMAKE_ASM_FLAGS -c)

set(PROJECT_SOURCES
    os/src/os_mem.c
    os/src/os_msg.c
    os/src/os_util.c
    os/src/os.c

    src/startup_stm32l4r5xx.s
    src/system_stm32l4xx.c

    src/app_defs.h

    src/main.c
    src/pwm_ao.c
    src/gpio_ao.c
    src/os_port_arm_m4.c
    src/rmk_hal_clock_cfg.c
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES}
    src/stm32l4xx_hal_conf.h
    src/stm32l4r5xx.h
    src/system_stm32l4xx.h
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/os/inc)

target_link_libraries(${PROJECT_NAME} PRIVATE
    HAL::STM32::L4::RCC
    HAL::STM32::L4::CORTEX
    HAL::STM32::L4::GPIO
    HAL::STM32::L4::TIM
    HAL::STM32::L4::TIMEx
    HAL::STM32::L4::PWR
    HAL::STM32::L4::PWREx
    CMSIS::STM32::L4
    STM32::NoSys)

add_definitions(-DSTM32L4R5xx)

stm32_add_linker_script(${PROJECT_NAME} PRIVATE L4R5ZI.ld)

# Do not use HAL template, not sure if this has been fixed
set(CMSIS_TEMPLATE "${CMAKE_SOURCE_DIR}/debug/_deps/stm32-cmsis-l4-src/Source/Templates/system_stm32l4xx.c")
set(CMSIS_TEMPLATE_PATH "${CMAKE_SOURCE_DIR}/debug/_deps/stm32-cmsis-l4-src/Source/Templates/")
file(REMOVE ${CMSIS_TEMPLATE})
file(COPY "${CMAKE_SOURCE_DIR}/src/system_stm32l4xx.c"
    DESTINATION ${CMSIS_TEMPLATE_PATH}
)

stm32_generate_binary_file(${PROJECT_NAME})
stm32_generate_hex_file(${PROJECT_NAME})
stm32_print_size_of_target(${PROJECT_NAME})