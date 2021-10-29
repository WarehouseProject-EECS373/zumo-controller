#include "drive_subsystem.h"

#include <os.h>
#include <stm32l4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32l4xx.h"

// pin configurations for drive

#define MOTOR_PWM_TIMER   TIM1
#define RIGHT_PWM_CHANNEL TIM_CHANNEL_1
#define LEFT_PWM_CHANNEL  TIM_CHANNEL_2

#define PWM_RIGHT_OUTPUT_PIN  GPIO_PIN_9
#define PWM_RIGHT_OUTPUT_PORT GPIOE
#define PWM_RIGHT_AF          GPIO_AF1_TIM1

#define PWM_LEFT_OUTPUT_PIN  GPIO_PIN_11
#define PWM_LEFT_OUTPUT_PORT GPIOE
#define PWM_LEFT_AF          GPIO_AF1_TIM1

#define RIGHT_DIR_PORT GPIOF
#define RIGHT_DIR_PIN  GPIO_PIN_12

#define LEFT_DIR_PORT GPIOF
#define LEFT_DIR_PIN  GPIO_PIN_13

// timer configuration
#define PRESCALER     1 // 16e6MHz / (1 + 1) = 8MHz
#define PERIOD        400 // 160 MHz / 400 = 20KHz
#define PWM_FREQ      (SystemCoreClock / ((PRESCALER) + 1))
#define DUTY_PULSE(x) ((uint32_t)((x) * (PERIOD)))

// drive subsystem states
#define DRIVE_STATE_DISABLED 0
#define DRIVE_STATE_ENABLED  1

// drive modes for PID controller
#define DRIVE_MODE_DRIVE  0
#define DRIVE_MODE_ROTATE 1

// change to fix hardware orientation (e.g. motor installed backwards)
#define LEFT_MOTOR_ORIENTATION  1
#define RIGHT_MOTOR_ORIENTATION 1

#define DEADBAND          0.0
#define MAX_DRIVE_PERCENT 1.0
#define MIN_DRIVE_PERCENT -1.0

// position control PID constants
#define kP 0.0
#define kI 0.0
#define kD 0.0

static float base_output_percent = 0.0;

// position control
static float setpoint = 0.0;
static float actual = 0.0;
static float previous_error = 0.0;
static float last_time = 0.0;
static float i_accumulator = 0.0;

static uint32_t state = DRIVE_STATE_ENABLED;
static uint32_t drive_mode = DRIVE_MODE_DRIVE;

static TIM_HandleTypeDef motor_pwm;

void HandleDriveLineSetpoint(DriveControlMessage_t* msg);
void HandleTimedActivity(Message_t* msg);
void HandleSetpointChange(DriveSetpointMessage_t* msg);

void SetOutoutPercent(float left_percent_output, float right_percent_output);

float ApplyDriveDeadband(float value);
float BoundDrivePercent(float percent_output);

void ClearPIDState();

void ConfigureGPIO();
void ConfigureTimer();

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

float ApplyDriveDeadband(float value)
{
    if(value < DEADBAND && value > -1 * DEADBAND)
    {
        return 0.0;
    }
    else
    {
        return value;
    }
}

void HandleTimedActivity(Message_t* msg)
{
    float current_time = (float)OSGetTime();

    float error = setpoint - actual;
    float p = error;
    i_accumulator += error * (current_time - last_time);
    float d = (error - previous_error) / (current_time - last_time);

    float motor_speed = kP * p + kI * i_accumulator + kI * d;

    float left_output = base_output_percent + motor_speed;
    float right_output = base_output_percent - motor_speed;

    left_output = BoundDrivePercent(left_output);
    right_output = BoundDrivePercent(right_output);

    SetOutoutPercent(left_output, right_output);

    last_time = current_time;
}

void ClearPIDState()
{
    setpoint = 0.0;
    actual = 0.0;
    previous_error = 0.0;
    last_time = 0.0;
    i_accumulator = 0.0;
}

void HandleSetpointChange(DriveSetpointMessage_t* msg)
{
    if(msg->drive_mode != drive_mode)
    {
        ClearPIDState();
        drive_mode = msg->drive_mode;
    }

    setpoint = msg->setpoint;
}

void SetDriveState(uint32_t new_state)
{
    ClearPIDState();
    state = new_state;
}

extern void DriveEventHandler(Message_t* msg)
{
    if(msg->id == DRIVE_ENABLE_MSG_ID)
    {
        SetDriveState(DRIVE_STATE_ENABLED);
    }
    else if(msg->id == DRIVE_DISABLE_MSG_ID)
    {
        SetDriveState(DRIVE_STATE_DISABLED);
    }
    else if(msg->id == DRIVE_CTL_IN_MSG_ID)
    {
        actual = ((DriveControlMessage_t*)msg)->actual;
    }
    else if(msg->id == DRIVE_TIMED_ACTIVITY_MSG_ID)
    {
        HandleTimedActivity(msg);
    }
    else if(msg->id == DRIVE_BASE_VELOCITY_MSG_ID)
    {
        base_output_percent = ((DriveBaseVelocityMessage_t*)msg)->base_velocity;
    }
    else if(msg->id == DRIVE_SETPOINT_MSG_ID)
    {
        HandleSetpointChange((DriveSetpointMessage_t*)msg);
    }
}

void ConfigureGPIO()
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
}

void ConfigureTimer()
{
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

extern void Drive_Init()
{
    ConfigureGPIO();
    ConfigureTimer();
    SetOutoutPercent(-0.8, 0.8);
}

void SetOutoutPercent(float left_percent_output, float right_percent_output)
{
    // don't set if drive isn't enabled
    if(state == DRIVE_STATE_DISABLED)
    {
        return;
    }

    // fix for incorrect hardware configuration
    left_percent_output *= (float)LEFT_MOTOR_ORIENTATION;
    right_percent_output *= (float)RIGHT_MOTOR_ORIENTATION;

    // apply deadband
    left_percent_output = ApplyDriveDeadband(left_percent_output);
    right_percent_output = ApplyDriveDeadband(right_percent_output);

    // set motor directions
    if(left_percent_output < 0.0)
    {
        left_percent_output *= -1;
        HAL_GPIO_WritePin(LEFT_DIR_PORT, LEFT_DIR_PIN, 1);
    }
    else
    {
        HAL_GPIO_WritePin(LEFT_DIR_PORT, LEFT_DIR_PIN, GPIO_PIN_RESET);
    }

    if(right_percent_output < 0.0)
    {
        right_percent_output *= -1;
        HAL_GPIO_WritePin(RIGHT_DIR_PORT, RIGHT_DIR_PIN, 1);
    }
    else
    {
        HAL_GPIO_WritePin(RIGHT_DIR_PORT, RIGHT_DIR_PIN, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&motor_pwm, LEFT_PWM_CHANNEL, DUTY_PULSE(left_percent_output));
    __HAL_TIM_SET_COMPARE(&motor_pwm, RIGHT_PWM_CHANNEL, DUTY_PULSE(right_percent_output));

    HAL_TIM_PWM_Start(&motor_pwm, LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&motor_pwm, RIGHT_PWM_CHANNEL);
}