# Realtime Micro Kernel - Development and Testing

***NOTE: current eecs 373 project has an updated version of this code with some major bug fixes, will move fixes here after project ends***

Development environment for a toy real-time, event driven, run-to-completion operating system. Not super useful yet though it's great for creating nasty problems to debug.

## Features

- Active Objects
  - Message queues
  - Variable sized, custom messages
- Periodic and single timed events
- Memory pools

## Supported Platforms

Tested and developed on STM32 platforms using `stm32-cmake`

- ARM Cortex-M3 (developed on STM32L4R5ZI Nucleo)

## Basic Usage

TODO

## Notes

### STM32 HAL source and headers

Some source and headers are included in the `src` directory. `stm32-cmake` is configured to not use the templated startup and board specific device so we must provide these ourselves. Mainly needed to get the clocks configured correctly.

### UART

This was painful to figure out.

- `VddIO2` must be enabled for LPUART to work. Enable `PWR` clock beforehand.

### Clocks, Timing

This was a pain.

- `SysTick_Handler` runs at lower interrupt priority for `rmkernel`. However, STM32 HAL expects to hook into the 1ms tick. Therefore, we need an alternative to `SysTick_Handler` for the STM32 HAL. The solution was to sacrifice `TIM2` to the HAL and configure it as a 1ms clock to drive the millisecond-precision OSTime and HAL time. Can hook onto the `__weak`ly defined `HAL_IncTick`, `HAL_InitTick`, and `HAL_GetTick` to make this happen.
