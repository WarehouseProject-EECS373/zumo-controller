#include "drive_subsystem.h"

#include <os.h>
#include <stm32l4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32l4xx.h"

#define MOTOR_PWM_TIMER   TIM4
#define LEFT_PWM_CHANNEL  TIM_CHANNEL_1
#define RIGHT_PWM_CHANNEL TIM_CHANNEL_2

#define PWM_RIGHT_OUTPUT_PIN  GPIO_PIN_4
#define PWM_LEFT_OUTPUT_PIN   GPIO_PIN_5
#define PWM_LEFT_OUTPUT_PORT  GPIOB
#define PWM_RIGHT_OUTPUT_PORT GPIOB
#define PWM_LEFT_AF           GPIO_AF2_TIM3
#define PWM_RIGHT_AF          GPIO_AF2_TIM3

#define LEFT_DIR_PORT  GPIOA
#define RIGHT_DIR_PORT GPIOA
#define LEFT_DIR_PIN   GPIO_PIN_3
#define RIGHT_DIR_PIN  GPIO_PIN_4

#define PRESCALER     99 // 16e6MHz / (999 + 1) = 160KHz
#define PERIOD        100
#define PWM_FREQ      (SystemCoreClock / ((PRESCALER) + 1))
#define DUTY_PULSE(x) ((uint32_t)((x) * (PERIOD)))

#define DRIVE_STATE_DISABLED 0
#define DRIVE_STATE_ENABLED  1

#define LEFT_MOTOR_ORIENTATION  1
#define RIGHT_MOTOR_ORIENTATION 1

#define MAX_DRIVE_PERCENT 100.0
#define MIN_DRIVE_PERCENT -100.0

#define BASE_VEL    0.0
#define POSITION_kP 0.0
#define POSITION_kI 0.0
#define POSITION_kD 0.0

static float position_setpoint = 0.0;
static float position_actual = 0.0;
static float position_previous_error = 0.0;
static float position_last_time = 0.0;
static float position_i_acc = 0.0;

static uint32_t state = DRIVE_STATE_DISABLED;

static TIM_HandleTypeDef motor_pwm;

void HandleDriveLineSetpoint(DriveControlMessage_t* msg);
void HandleTimedActivity(Message_t* msg);
float BoundDrivePercent(float percent_output);
void ClearPIDState();

float BoundDrivePercent(float output)
{
    if(output > MAX_DRIVE_PERCENT)
    {
        return MAX_DRIVE_PERCENT;
    }
    else if(output < MIN_DRIVE_PERCENT)
    {
        return MIN_DRIVE_PERCENT;
    }
    else
    {
        return output;
    }
}

void HandleDriveLineSetpoint(DriveControlMessage_t* msg)
{
    position_actual = msg->actual;
}

void HandleTimedActivity(Message_t* msg)
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

void ClearPIDState()
{
    position_setpoint = 0.0;
    position_actual = 0.0;
    position_previous_error = 0.0;
    position_last_time = 0.0;
    position_i_acc = 0.0;
}

extern void DriveEventHandler(Message_t* msg)
{
    if(msg->id == DRIVE_ENABLE_MSG_ID)
    {
        ClearPIDState();
        state = DRIVE_STATE_ENABLED;
    }
    else if(msg->id == DRIVE_DISABLE_MSG_ID)
    {
        ClearPIDState();
        state = DRIVE_STATE_DISABLED;
    }
    else if(msg->id == DRIVE_CTL_IN_MSG_ID)
    {
        HandleDriveLineSetpoint((DriveControlMessage_t*)msg);
    }
    else if(msg->id == DRIVE_TIMED_ACTIVITY_MSG_ID)
    {
        HandleTimedActivity(msg);
    }
}

extern void Drive_Init()
{
    GPIO_InitTypeDef pwm_gpio_cfg = {0};
    GPIO_InitTypeDef dir_gpio_cfg = {0};

    pwm_gpio_cfg.Pin = PWM_LEFT_OUTPUT_PIN;
    pwm_gpio_cfg.Mode = GPIO_MODE_AF_PP;
    pwm_gpio_cfg.Alternate = PWM_LEFT_AF;
    pwm_gpio_cfg.Pull = GPIO_NOPULL;
    pwm_gpio_cfg.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(PWM_LEFT_OUTPUT_PORT, &pwm_gpio_cfg);

    pwm_gpio_cfg.Pin = PWM_RIGHT_OUTPUT_PIN;
    pwm_gpio_cfg.Mode = GPIO_MODE_AF_PP;
    pwm_gpio_cfg.Alternate = PWM_RIGHT_AF;
    pwm_gpio_cfg.Pull = GPIO_NOPULL;
    pwm_gpio_cfg.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(PWM_RIGHT_OUTPUT_PORT, &pwm_gpio_cfg);

    dir_gpio_cfg.Pin = LEFT_DIR_PIN;
    dir_gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    dir_gpio_cfg.Pull = GPIO_NOPULL;
    dir_gpio_cfg.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(LEFT_DIR_PORT, &dir_gpio_cfg);

    dir_gpio_cfg.Pin = RIGHT_DIR_PIN;
    dir_gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    dir_gpio_cfg.Pull = GPIO_NOPULL;
    dir_gpio_cfg.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(RIGHT_DIR_PORT, &dir_gpio_cfg);

    motor_pwm.Instance = MOTOR_PWM_TIMER;
    motor_pwm.Init.Prescaler = PRESCALER;
    motor_pwm.Channel = LEFT_PWM_CHANNEL | RIGHT_PWM_CHANNEL;
    motor_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
    motor_pwm.Init.Period = PERIOD;
    motor_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    motor_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&motor_pwm);

    TIM_OC_InitTypeDef tim3_oc = {0};

    tim3_oc.OCMode = TIM_OCMODE_PWM1;
    tim3_oc.Pulse = 0;
    tim3_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim3_oc.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&motor_pwm, &tim3_oc, LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_ConfigChannel(&motor_pwm, &tim3_oc, RIGHT_PWM_CHANNEL);
}

extern void Drive_SetOutputPercent(float left_percent_output, float right_percent_output)
{
    if(state == DRIVE_STATE_DISABLED)
    {
        return;
    }

    // use output percent to set direction, not sure which pins to use

    __HAL_TIM_SET_COMPARE(&motor_pwm, TIM_CHANNEL_1, DUTY_PULSE(left_percent_output));
    __HAL_TIM_SET_COMPARE(&motor_pwm, TIM_CHANNEL_2, DUTY_PULSE(right_percent_output));

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
    HAL_TIM_PWM_Start(&motor_pwm, LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&motor_pwm, RIGHT_PWM_CHANNEL);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);
}