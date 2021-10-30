#include <stdint.h>
#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include "os.h"

#include "app_defs.h"

OS_t os;

static TIM_HandleTypeDef hhal_tim;

__attribute__((__interrupt__)) extern void TIM2_IRQHandler()
{
    if(__HAL_TIM_GET_FLAG(&hhal_tim, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&hhal_tim, TIM_IT_UPDATE);
        HAL_IncTick();
    }
}

/**
 * @brief Increment system time
 *
 */
extern void HAL_IncTick(void)
{
    os.time++;
}

/**
 * @brief Implementation of __weak
 *
 * @return uint32_t current time in milliseconds
 */
extern uint32_t HAL_GetTick(void)
{
    return os.time;
}

/**
 * @brief Create 1ms timer to replace SysTick for use with HAL and general system time
 *
 * @param TickPriority
 * @return HAL_StatusTypeDef
 */
extern HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    os.time = 0;
    __TIM2_CLK_ENABLE();

    hhal_tim.Instance = TIM2;
    hhal_tim.Channel = TIM_CHANNEL_1;
    hhal_tim.Init.Prescaler = 999;
    hhal_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hhal_tim.Init.Period = 15;
    hhal_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hhal_tim.Init.RepetitionCounter = 0;

    HAL_TIM_Base_Init(&hhal_tim);

    // HAL tick timebase interrupt priority and enable
    HAL_NVIC_SetPriority(TIM2_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_TIM_Base_Start_IT(&hhal_tim);

    return HAL_OK;
}

extern void Clock_Init()
{
    // setup system clock using High speed internal oscillator
    RCC->CR |= (RCC_CR_HSION);
    while(!(RCC->CR & RCC_CR_HSIRDY))
    {
    }

    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= (RCC_CFGR_SW_HSI);

    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    SystemCoreClock = HSI_VALUE;

    //
    // enable peripheral clocks
    //
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_RCC_LPUART1_CLK_ENABLE();

    __HAL_RCC_DAC1_CLK_ENABLE();

    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    // set systick frequency (1ms)
    SysTick_Config(SystemCoreClock / 1000);
}