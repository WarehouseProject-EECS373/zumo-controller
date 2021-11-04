#ifndef RMK_HAL_CLOCK_CFG_H
#define RMK_HAL_CLOCK_CFG_H

#include <stdint.h>
#include <stm/stm32f4xx.h>
#include <stm32f4xx_hal.h>

/**
 * @brief ISR for handling millisecond clock interrupt. Not kernel aware
 *
 */
__attribute__((__interrupt__)) extern void TIM2_IRQHandler();

/**
 * @brief Implementation of __weak HAL_IncTick
 *
 * @return uint32_t current time in milliseconds
 */
extern void HAL_IncTick(void);

/**
 * @brief Get current system time
 *
 * @return uint32_t
 */
extern uint32_t HAL_GetTick(void);

/**
 * @brief Create 1ms timer to replace SysTick for use with HAL and general system time
 *
 * @param TickPriority
 * @return HAL_StatusTypeDef
 */
extern HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

/**
 * @brief Initialize all clocks
 *
 */
extern void Clock_Init();

#endif
