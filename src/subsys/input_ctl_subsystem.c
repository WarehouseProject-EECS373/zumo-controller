#include "input_ctl_subsystem.h"

#include <stm32f4xx_hal.h>
#include <stm/stm32f4xx.h>

#include <os.h>
#include <os_msg.h>

#include "app_defs.h"

// only allow button to be pressed once every 100ms
#define USER_BUTTON_DEBOUNCE_TIME 100

#define USER_BUTTON_PORT GPIOC
#define USER_BUTTON_PIN  GPIO_PIN_13

__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void)
{
    // button debouncing
    static bool     time_set = false;
    static uint32_t last_time = 0;

    OS_ISR_ENTER(&os);

    // if triggered by GPIO_13
    if (__HAL_GPIO_EXTI_GET_FLAG(USER_BUTTON_PIN) != RESET)
    {
        // check debounce
        if (!time_set || USER_BUTTON_DEBOUNCE_TIME < (OSGetTime() - last_time))
        {
            // toggle drive state (enabled and disabled)
            Message_t drive_toggle_msg = {.id = DRIVE_TOGGLE_MSG_ID, .msg_size = sizeof(Message_t)};
            MsgQueuePut(&drive_ss_ao, &drive_toggle_msg);
        }

        // must clear interrupt
        __HAL_GPIO_EXTI_CLEAR_IT(USER_BUTTON_PIN);
    }

    // always exit to tail chain scheduler
    OS_ISR_EXIT(&os);
}

extern void InputEventHandler(Message_t* msg)
{
    UNUSED(msg);
}

extern void ITCTL_Init()
{
    // initialize blue user button interrupt
    GPIO_InitTypeDef gpio_cfg;
    gpio_cfg.Pin = USER_BUTTON_PIN;
    gpio_cfg.Mode = GPIO_MODE_IT_FALLING;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(USER_BUTTON_PORT, &gpio_cfg);

    // set blue user push button interrupt priority
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
