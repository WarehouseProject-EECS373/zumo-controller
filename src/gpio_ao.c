#include "gpio_ao.h"

#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"

#define USER_BUTTON_DEBOUNCE_TIME 100

__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void)
{
    static bool time_set = false;
    static uint32_t last_time = 0;

    OS_ISR_ENTER(&os);

    // if triggered by GPIO_13
    if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13))
    {
        // check debounce
        if (!time_set || USER_BUTTON_DEBOUNCE_TIME < (OSGetTime() - last_time))
        {
        }

        // must clear interrupt
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    }

    // always exit to tail chain scheduler
    OS_ISR_EXIT(&os);
}

extern void GPIO_OutputInit()
{
    GPIO_InitTypeDef gpio_cfg;

    gpio_cfg.Pin = GPIO_PIN_7;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &gpio_cfg);

    gpio_cfg.Pin = GPIO_PIN_3;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &gpio_cfg);
}

extern void GPIO_UserPushButtonInit()
{
    GPIO_InitTypeDef gpio_cfg;
    gpio_cfg.Pin = GPIO_PIN_13;
    gpio_cfg.Mode = GPIO_MODE_IT_FALLING;
    gpio_cfg.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOC, &gpio_cfg);
}

extern void GPIO_Init()
{
    GPIO_OutputInit();
    GPIO_UserPushButtonInit();

    // set blue user push button interrupt priority
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}