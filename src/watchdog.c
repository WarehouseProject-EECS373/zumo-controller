#include "watchdog.h"

#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include <os.h>

#include "app_defs.h"

extern void WatchdogEventHandler(Message_t* msg)
{
    if (HEARTBEAT_MSG_ID == msg->id)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

extern void Watchdog_Init()
{
    GPIO_InitTypeDef gpio_cfg;

    gpio_cfg.Pin = GPIO_PIN_5;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &gpio_cfg);
}
