#include "input_ctl_subsystem.h"

#include <stm32l4xx_hal.h>

#include <os.h>
#include <os_msg.h>

#include "app_defs.h"
#include "stm/stm32l4xx.h"

#define USER_BUTTON_DEBOUNCE_TIME 100

__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void)
{
    static bool time_set = false;
    static uint32_t last_time = 0;

    OS_ISR_ENTER(&os);

    // if triggered by GPIO_13
    if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13))
    {
        // check debounce
        if(!time_set || USER_BUTTON_DEBOUNCE_TIME < (OSGetTime() - last_time))
        {
            Message_t drive_toggle_msg = {.id = DRIVE_TOGGLE_MSG_ID, .msg_size = sizeof(Message_t)};
            MsgQueuePut(&drive_ss_ao, &drive_toggle_msg);
        }

        // must clear interrupt
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    }

    // always exit to tail chain scheduler
    OS_ISR_EXIT(&os);
}

extern void InputHandler(Message_t* msg)
{
}

extern void ITCTL_Init()
{
    GPIO_InitTypeDef gpio_cfg;
    gpio_cfg.Pin = GPIO_PIN_13;
    gpio_cfg.Mode = GPIO_MODE_IT_FALLING;
    gpio_cfg.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOC, &gpio_cfg);

    // set blue user push button interrupt priority
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}