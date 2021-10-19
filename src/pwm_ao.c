#include "pwm_ao.h"

#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"

#define PRESCALER 99 // 16e6MHz / (999 + 1) = 160KHz
#define PERIOD 100
#define DC_PERIOD_OFFSET 1
#define PWM_FREQ (SystemCoreClock / ((PRESCALER) + 1))
#define DUTY_PULSE(x) ((uint32_t)((x) * (PERIOD)) + DC_PERIOD_OFFSET)

static TIM_HandleTypeDef htim3;
// static TIM_HandleTypeDef htim4;

extern void PWM_Init()
{
    //
    //  TIM3
    //

    GPIO_InitTypeDef gpio_cfg = {0};

    gpio_cfg.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio_cfg.Mode = GPIO_MODE_AF_PP;
    gpio_cfg.Alternate = GPIO_AF2_TIM3;
    gpio_cfg.Pull = GPIO_NOPULL;
    gpio_cfg.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(GPIOB, &gpio_cfg);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = PRESCALER;
    htim3.Channel = TIM_CHANNEL_1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PERIOD;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef tim3_oc = {0};

    tim3_oc.OCMode = TIM_OCMODE_PWM1;
    tim3_oc.Pulse = 0;
    tim3_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim3_oc.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim3, &tim3_oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim3_oc, TIM_CHANNEL_2);
}

extern void PWM_Start()
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DUTY_PULSE(0.5));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, DUTY_PULSE(0.25));

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);
}

extern void DualPwmEventHandler(Message_t *msg)
{
}