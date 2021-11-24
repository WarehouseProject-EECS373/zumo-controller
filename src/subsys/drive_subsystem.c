#include "drive_subsystem.h"

#include <os.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32f4xx.h"

#include "trace.h"

// pin configurations for drive

#define MOTOR_PWM_TIMER_RIGHT TIM3
#define MOTOR_PWM_TIMER_LEFT  TIM4
#define RIGHT_PWM_CHANNEL     TIM_CHANNEL_2
#define LEFT_PWM_CHANNEL      TIM_CHANNEL_1

#define PWM_RIGHT_OUTPUT_PIN  GPIO_PIN_7
#define PWM_RIGHT_OUTPUT_PORT GPIOC
#define PWM_RIGHT_AF          GPIO_AF2_TIM3

#define PWM_LEFT_OUTPUT_PIN  GPIO_PIN_6
#define PWM_LEFT_OUTPUT_PORT GPIOB
#define PWM_LEFT_AF          GPIO_AF2_TIM4

#define RIGHT_DIR_PORT GPIOA
#define RIGHT_DIR_PIN  GPIO_PIN_8

#define LEFT_DIR_PORT GPIOA
#define LEFT_DIR_PIN  GPIO_PIN_9

// timer clock configuration
#define PRESCALER     1 // 16e6MHz / (1 + 1) = 8MHz
#define PERIOD        400 // 160 MHz / 400 = 20KHz
#define PWM_FREQ      (SystemCoreClock / ((PRESCALER) + 1))
#define DUTY_PULSE(x) ((uint32_t)((x) * (PERIOD))) // [0.0, 1.0] mapped to [0, PERIOD]

// drive subsystem states
#define DRIVE_STATE_DISABLED 0
#define DRIVE_STATE_ENABLED  1

// drive modes
#define DRIVE_MODE_DRIVE        0x0
#define DRIVE_MODE_ROTATE       0x1
#define DRIVE_MODE_CALIBRATE    0x2

// change to fix hardware orientation (e.g. motor installed backwards)
#define LEFT_MOTOR_ORIENTATION  1
#define RIGHT_MOTOR_ORIENTATION 1

// minimum and maximum output percents
#define MAX_DRIVE_PERCENT 1.0
#define MIN_DRIVE_PERCENT -1.0


// if we ever get small output percent that's "close enough" to 0.0 but PID
// doesn't drive to exactly 0.0
static float deadband = 0.0;
static uint32_t drive_control_loop_period = 10;

// position control PID constants
static float kP = 0.0;
static float kI = 0.0;
static float kD = 0.0;

// "target" speed when driving straight,
// PID will add/subtract from this for right/left motor to turn
static float base_output_percent = 0.0;

// position control
static float setpoint = 0.0;
static float actual = 0.0;
static float previous_error = 0.0;
static float last_time = 0.0;
static float i_accumulator = 0.0;

// drive subsystem state
static uint32_t state = DRIVE_STATE_DISABLED;
static uint32_t drive_mode = DRIVE_MODE_OPEN_LOOP;

// PWM timer
static TIM_HandleTypeDef motor_pwm_right;
static TIM_HandleTypeDef motor_pwm_left;

static TimedEventSimple_t drive_control_loop_periodic_event;
static Message_t drive_control_loop_periodic_msg = {.id = DRIVE_PERIODIC_EVENT_MSG_ID, .msg_size = sizeof(Message_t)};

static TimedEventSimple_t current_timed_event_loopback;
static TimedEventSimple_t current_timed_event_response;
static Message_t current_timed_event_msg;


static void HandleStartTimedTurn(DriveTimedTurn_t* msg);
static void HandleTimedActivity(Message_t* msg);
static void HandleSetpointChange(DriveSetpointMessage_t* msg);

static void SetOutoutPercent(float left_percent_output, float right_percent_output);
static void SetDriveState(uint32_t new_state);
static void ToggleDriveState();

static void RampTestIteration();

static float ApplyDriveDeadband(float value);
static float BoundDrivePercent(float percent_output);

static void ClearPIDState();

static void ConfigureGPIO();
static void ConfigureTimer();

static void PropertyHandler(PropertyGetSetMessage_t *msg);

static void PropertyHandler(PropertyGetSetMessage_t *msg)
{
    if (DRIVE_DEADBAND_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, deadband, float);
    }
    else if (DRIVE_CTL_LOOP_PERIOD_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, drive_control_loop_period, uint32_t);
    }
    else if (DRIVE_kP_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, kP, float);
    }
    else if (DRIVE_kI_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, kI, float);
    }
    else if (DRIVE_kD_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, kD, float);
    }
    else if (DRIVE_BASE_OUTPUT_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, base_output_percent, float);
    }
    else if (DRIVE_STATE_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, state, uint32_t);
    }
    else if (DRIVE_SETPOINT_ID == msg->p_id)
    {
        GET_SET_PROPERTY(msg, state, float);
    }
}

/**
 * @brief bounds desired output percent between -1.0 and 1.0
 *
 * @param output
 * @return float
 */
static float BoundDrivePercent(float output)
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

/**
 * @brief Like a bandstop filter. If we give the drive very small outut
 *  percent that's "close enough" to 0.0, we may want to just stop
 *
 * @param value
 * @return float
 */
static float ApplyDriveDeadband(float value)
{
    // TODO: make generic to accept and target, not just 0.0
    if(value < deadband && value > -1 * deadband)
    {
        return 0.0;
    }
    else
    {
        return value;
    }
}

/**
 * @brief Main PID control loop, should happen every n milliseconds
 *
 * @param msg
 */
static void HandleTimedActivity(Message_t* msg)
{
    UNUSED(msg);

    float current_time = (float)OSGetTime();

    // position error (how far away from line are we and in what direction)
    float error = setpoint - actual;

    // calculate P, I, D
    float p = error;
    i_accumulator += error * (current_time - last_time);
    float d = (error - previous_error) / (current_time - last_time);

    // sum scaled P, I, D
    float motor_speed = kP * p + kI * i_accumulator + kD * d;

    // calculate left and right percent output
    float left_output = base_output_percent + motor_speed;
    float right_output = base_output_percent - motor_speed;

    SetOutoutPercent(left_output, right_output);

    // save last time for next calculation
    last_time = current_time;

#ifdef DRIVE_CTL_TRACE_ENABLED
    ControlLoopTrace(left_output, right_output, error, actual);
#endif
}

/**
 * @brief Test all combinations of motor speeds and directions
 *
 */
static void RampTestIteration()
{
    static float left = MIN_DRIVE_PERCENT;
    static float right = MIN_DRIVE_PERCENT;
    static float iter_step = 0.1;
    static bool done = false;

    // do nothing if done with test
    if(done)
    {
        SetOutoutPercent(0.0, 0.0);
        return;
    }

    SetOutoutPercent(left, right);

    // step left until left is at max
    left += iter_step;

    if(left > MAX_DRIVE_PERCENT)
    {
        right += iter_step;
        left = MIN_DRIVE_PERCENT;
    }

    // done with test once right is max
    if(right > MAX_DRIVE_PERCENT)
    {
        left = 0.0;
        right = 0.0;
        done = true;
    }
}

/**
 * @brief reset PID controller
 *
 */
static void ClearPIDState()
{
    setpoint = 0.0;
    actual = 0.0;
    previous_error = 0.0;
    last_time = 0.0; // FIXME: need to get better initial time
    i_accumulator = 0.0;
}

/**
 * @brief change PID setpoint
 *
 * @param msg
 */
static void HandleSetpointChange(DriveSetpointMessage_t* msg)
{
    setpoint = msg->setpoint;
    drive_mode = DRIVE_MODE_CLOSED_LOOP;

    if (!drive_control_loop_periodic_event.active)
    {
        TimedEventSimpleCreate(&drive_control_loop_periodic_event, &drive_ss_ao, &drive_control_loop_periodic_msg, drive_control_loop_period, TIMED_EVENT_PERIODIC_TYPE);
        SchedulerAddTimedEvent(&drive_control_loop_periodic_event);
    }

#ifdef DRIVE_CTL_TRACE_ENABLED
    ControlLoopTraceInit(setpoint);
#endif
}

/**
 * @brief Set the set drive state
 *
 * @param new_state enabled or disabled
 */
static void SetDriveState(uint32_t new_state)
{
    if(DRIVE_STATE_DISABLED == new_state)
    {
        SetOutoutPercent(0.0, 0.0);
    }

    ClearPIDState();
    state = new_state;
}

/**
 * @brief Toggle drive state
 *
 */
static void ToggleDriveState()
{
    if(state == DRIVE_STATE_DISABLED)
    {
        SetDriveState(DRIVE_STATE_ENABLED);
    }
    else
    {
        SetDriveState(DRIVE_STATE_DISABLED);
    }
}

static void HandleStartTimedTurn(DriveTimedTurn_t* msg)
{
    // for timed turn, create two single shot timed events
    // send to 
    // - requesting AO
    // - drive event handler (to stop turn)

    DriveTimedTurn_t *ttmsg = (DriveTimedTurn_t*)msg;

    current_timed_event_msg.id = DRIVE_TIMED_TURN_DONE_MSG_ID;
    current_timed_event_msg.msg_size = sizeof(Message_t);

    // create timed events
    TimedEventSimpleCreate(&current_timed_event_loopback, &drive_ss_ao, &current_timed_event_msg, ttmsg->time, TIMED_EVENT_SINGLE_TYPE);
    TimedEventSimpleCreate(&current_timed_event_response, ttmsg->response, &current_timed_event_msg, ttmsg->time, TIMED_EVENT_SINGLE_TYPE);

        // schedule timed events
    SchedulerAddTimedEvent(&current_timed_event_loopback);
    SchedulerAddTimedEvent(&current_timed_event_response);
        
    // start drive
    if (DRIVE_TURN_DIR_LEFT == ttmsg->direction)
    {
        // TODO: make turn speed configurable
        SetOutoutPercent(-0.1, 0.1);
    }
    else
    {
        // TODO: make turn speed configurable
        SetOutoutPercent(0.1, -0.1);
    }
}

extern void DriveEventHandler(Message_t* msg)
{
    if (DRIVE_TOGGLE_MSG_ID == msg->id)
    {
        ToggleDriveState();
    }
    else if (DRIVE_ENABLE_MSG_ID == msg->id)
    {
        SetDriveState(DRIVE_STATE_ENABLED);
    }
    else if (DRIVE_DISABLE_MSG_ID == msg->id)
    {
        SetDriveState(DRIVE_STATE_DISABLED);
    }
    else if (DRIVE_CTL_IN_MSG_ID == msg->id)
    {
        actual = ((DriveControlMessage_t*)msg)->actual;
    }
    else if (DRIVE_TIMED_ACTIVITY_MSG_ID == msg->id || DRIVE_PERIODIC_EVENT_MSG_ID == msg->id)
    {
        HandleTimedActivity(msg);
    }
    else if (DRIVE_BASE_VELOCITY_MSG_ID == msg->id)
    {
        base_output_percent = ((DriveBaseVelocityMessage_t*)msg)->base_velocity;
    }
    else if (DRIVE_SETPOINT_MSG_ID == msg->id)
    {
        HandleSetpointChange((DriveSetpointMessage_t*)msg);
    }
    else if (DRIVE_RAMP_TEST_ITERATION_MSG_ID == msg->id)
    {
        RampTestIteration();
    }
    else if (DRIVE_TIMED_TURN_MSG_ID == msg->id)
    {
        HandleStartTimedTurn((DriveTimedTurn_t*)msg);
    }
    else if (DRIVE_TIMED_TURN_DONE_MSG_ID == msg->id)
    {
        // stop drive system when timed event done
        SetOutoutPercent(0.0, 0.0);
    }
    else if (DRIVE_OPEN_LOOP_MSG_ID == msg->id)
    {
        DriveOpenLoopControlMessage_t *cmsg = (DriveOpenLoopControlMessage_t*)msg;

        drive_mode = DRIVE_MODE_OPEN_LOOP;
        TimedEventDisable(&drive_control_loop_periodic_event);
        SetOutoutPercent(cmsg->percent_left, cmsg->percent_right);
    }
    else if (GET_PROPERTY_MSG_ID == msg->id || SET_PROPERTY_MSG_ID == msg->id)
    {
        PropertyHandler((PropertyGetSetMessage_t*)msg);
    }
}

/**
 * @brief Configure all GPIOs used by the drive subsystem
 *
 */
static void ConfigureGPIO()
{
    GPIO_InitTypeDef pwm_gpio_cfg = {0};
    GPIO_InitTypeDef dir_gpio_cfg = {0};

    // PWM output
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

    // direction output
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

/**
 * @brief Setup PWM timer
 *
 */
static void ConfigureTimer()
{
    motor_pwm_right.Instance = MOTOR_PWM_TIMER_RIGHT;
    motor_pwm_right.Init.Prescaler = PRESCALER;
    motor_pwm_right.Channel = RIGHT_PWM_CHANNEL;
    motor_pwm_right.Init.CounterMode = TIM_COUNTERMODE_UP;
    motor_pwm_right.Init.Period = PERIOD;
    motor_pwm_right.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    motor_pwm_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&motor_pwm_right);

    motor_pwm_left.Instance = MOTOR_PWM_TIMER_LEFT;
    motor_pwm_left.Init.Prescaler = PRESCALER;
    motor_pwm_left.Channel = LEFT_PWM_CHANNEL;
    motor_pwm_left.Init.Period = PERIOD;
    motor_pwm_left.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    motor_pwm_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&motor_pwm_left);

    TIM_OC_InitTypeDef tim_oc = {0};

    tim_oc.OCMode = TIM_OCMODE_PWM1;
    tim_oc.Pulse = 0;
    tim_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&motor_pwm_left, &tim_oc, LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_ConfigChannel(&motor_pwm_right, &tim_oc, RIGHT_PWM_CHANNEL);
}

extern void Drive_Init()
{
    drive_control_loop_periodic_event.active = false;
    ConfigureGPIO();
    ConfigureTimer();

    SetOutoutPercent(0.0, 0.0);
}

/**
 * @brief Set the drive output percent
 *
 * @param left_percent_output
 * @param right_percent_output
 */
static void SetOutoutPercent(float left_percent_output, float right_percent_output)
{
    // don't set if drive isn't enabled
    if(state == DRIVE_STATE_DISABLED)
    {
        return;
    }

    // fix for incorrect hardware configuration
    left_percent_output *= (float)LEFT_MOTOR_ORIENTATION;
    right_percent_output *= (float)RIGHT_MOTOR_ORIENTATION;

    // apply bounding and deadband
    left_percent_output = BoundDrivePercent(left_percent_output);
    right_percent_output = BoundDrivePercent(right_percent_output);

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

    // set CCR and restart timer
    __HAL_TIM_SET_COMPARE(&motor_pwm_left, LEFT_PWM_CHANNEL, DUTY_PULSE(left_percent_output));
    __HAL_TIM_SET_COMPARE(&motor_pwm_right, RIGHT_PWM_CHANNEL, DUTY_PULSE(right_percent_output));

    HAL_TIM_PWM_Start(&motor_pwm_left, LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&motor_pwm_right, RIGHT_PWM_CHANNEL);
}

