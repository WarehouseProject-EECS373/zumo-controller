#include "drive_subsystem.h"

#include <stm32l4xx_hal.h>

#include "stm/stm32l4xx.h"
#include "app_defs.h"

#include <os.h>

#define PRESCALER 99 // 16e6MHz / (999 + 1) = 160KHz
#define PERIOD 100
#define PWM_FREQ (SystemCoreClock / ((PRESCALER) + 1))
#define DUTY_PULSE(x) ((uint32_t)((x) * (PERIOD)))

#define DRIVE_STATE_DISABLED 0
#define DRIVE_STATE_ENABLED 1

#define LEFT_MOTOR_ORIENTATION 1
#define RIGHT_MOTOR_ORIENTATION 1

#define MAX_DRIVE_PERCENT 100.0
#define MIN_DRIVE_PERCENT -100.0

#define BASE_VEL 0.0
#define POSITION_kP 0.0
#define POSITION_kI 0.0
#define POSITION_kD 0.0

static float position_setpoint = 0.0;
static float position_actual = 0.0;
static float position_previous_error = 0.0;
static float position_last_time = 0.0;
static float position_i_acc = 0.0;

static uint32_t state = DRIVE_STATE_DISABLED;

static TIM_HandleTypeDef htim3;

void HandleDriveLineSetpoint(DriveControlMessage_t *msg);
void HandleTimedActivity(Message_t *msg);
float BoundDrivePercent(float percent_output);

float BoundDrivePercent(float output)
{
    if (output > MAX_DRIVE_PERCENT)
    {
        return MAX_DRIVE_PERCENT;
    }
    else if (output < MIN_DRIVE_PERCENT)
    {
        return MIN_DRIVE_PERCENT;
    }
    else
    {
        return output;
    }
}

void HandleDriveLineSetpoint(DriveControlMessage_t *msg)
{
    position_actual = msg->actual;
}

void HandleTimedActivity(Message_t *msg)
{
    float current_time = (float)OSGetTime();

    float position_error = position_setpoint - position_actual;
    float p = position_error;
    position_i_acc += position_error * (current_time - position_last_time);
    float d = (position_error - position_previous_error) / (current_time - position_last_time);

    float motor_speed = POSITION_kP * p + POSITION_kI * position_i_acc + POSITION_kI * d;

    float left_output = BASE_VEL + motor_speed;
    float right_output = BASE_VEL - motor_speed;

    left_output = BoundDrivePercent(left_output);
    right_output = BoundDrivePercent(right_output);

    Drive_SetOutputPercent(left_output, right_output);

    position_last_time = current_time;
}

extern void DriveEventHandler(Message_t *msg)
{
    if (msg->id == DRIVE_ENABLE_MSG_ID)
    {
        state = DRIVE_STATE_ENABLED;
    }
    else if (msg->id == DRIVE_DISABLE_MSG_ID)
    {
        state = DRIVE_STATE_DISABLED;
    }
    else if (msg->id == DRIVE_CTL_IN_MSG_ID)
    {
        HandleDriveLineSetpoint((DriveControlMessage_t *)msg);
    }
    else if (msg->id == DRIVE_TIMED_ACTIVITY_MSG_ID)
    {
        HandleTimedActivity(msg);
    }
}

extern void Drive_Init()
{
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

extern void Drive_SetOutputPercent(float left_percent_output, float right_percent_output)
{
    if (state == DRIVE_STATE_DISABLED)
    {
        return;
    }

    // use output percent to set direction, not sure which pins to use

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DUTY_PULSE(left_percent_output));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, DUTY_PULSE(right_percent_output));

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);
}