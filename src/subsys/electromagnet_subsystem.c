#include "electromagnet_subsystem.h"

#include <stm32f4xx_hal.h>
#include "stm/stm32f4xx.h"

#include "app_defs.h"

#include <os.h>

#define EMAG_PORT GPIOB
#define EMAG_PIN  GPIO_PIN_5

extern void EMag_Init()
{
    GPIO_InitTypeDef gpio_cfg;

    gpio_cfg.Pin = EMAG_PIN;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(EMAG_PORT, &gpio_cfg);
    HAL_GPIO_WritePin(EMAG_PORT, EMAG_PIN, GPIO_PIN_RESET);
}

extern void ElectromagnetEventHandler(Message_t* msg)
{
    if (ELECTROMAGNET_DISABLE == msg->id)
    {
        HAL_GPIO_WritePin(EMAG_PORT, EMAG_PIN, GPIO_PIN_RESET);
    }
    else if (ELECTROMAGNET_ENABLE == msg->id)
    {
        HAL_GPIO_WritePin(EMAG_PORT, EMAG_PIN, 1);
    }
}
