#ifndef RMK_HAL_CLOCK_CFG_H
#define RMK_HAL_CLOCK_CFG_H

#include <stdint.h>
#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

__attribute__((__interrupt__)) extern void TIM2_IRQHandler();

extern void HAL_IncTick(void);

extern HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

extern void Clock_Init();

#endif