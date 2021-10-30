cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/modules/stm32-cmake/cmake/stm32_gcc.cmake)

project(zumo-controller C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

add_definitions(-DSTM32L4R5xx)

stm32_fetch_cmsis(L4)
stm32_fetch_hal(L4)

find_package(CMSIS COMPONENTS STM32L4R5ZI REQUIRED)
find_package(HAL COMPONENTS STM32L4 REQUIRED)

set(CMAKE_ASM_FLAGS -c)

set(OS_SOURCES
    os/src/os_mem.c
    os/src/os_msg.c
    os/src/os_util.c
    os/src/os.c
)

set(PROJECT_SOURCES
    src/app_defs.h
    
    src/main.c
    src/gpio_ao.c
    src/os_port_arm_m4.c
    src/rmk_hal_clock_cfg.c
    
    src/subsys/drive_subsystem.c
    src/subsys/input_ctl_subsystem.c
    src/subsys/reflectance_array_subsystem.c
)

set(STM_SOURCES
    src/stm/startup_stm32l4r5xx.s
    src/stm/system_stm32l4xx.c
    src/stm32l4xx_hal_conf.h
    src/stm/stm32l4r5xx.h
    src/stm/system_stm32l4xx.h
)
    
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES} ${OS_SOURCES} ${STM_SOURCES})
    
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

stm32_add_linker_script(${PROJECT_NAME} PRIVATE L4R5ZI.ld)

stm32_generate_binary_file(${PROJECT_NAME})
stm32_generate_hex_file(${PROJECT_NAME})
stm32_print_size_of_target(${PROJECT_NAME})