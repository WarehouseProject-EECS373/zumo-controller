#include "reflectance_array_subsystem.h"

#include <os.h>
#include <stm32f4xx_hal.h>
#include <string.h>

#include "app_defs.h"
#include "stm/stm32f4xx.h"

#include "trace.h"

// states for periodic event
#define STATE_DISABLED       0x0
#define STATE_CALIBRATE      0x1
#define STATE_FUNCTIONAL     0x2
#define STATE_LINE_FOLLOWING 0x3

// calibration states
#define STATE_CALIBRATE_LEFT_TURN  0x0
#define STATE_CALIBRATE_RIGHT_TURN 0x1
#define STATE_CALIBRATE_RETURN     0x2

// duration of turn of calibration
// should not really be timed but we don't have encoders
// can go to gyro/magnetometer eventually if have time
#define CALIBRATION_TIMED_TURN_DURATION 2000

#define NOISE_THRESHOLD                      0
#define ON_LINE_THRESHOLD                    300 // are sensors over line? FIXME: need to determine experimentally
#define ABOVE_LINE(sensor)                   ((sensor) > ON_LINE_THRESHOLD)
#define ABOVE_NOISE_THRESH(val)              ((val) > NOISE_THRESHOLD)
#define REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD 5
#define MAX_READING                          4000
#define SENSOR_CHARGE_DELAY                  50
#define MIDPOINT                             2500

#define NOT_IN_INTERSECTION 0
#define IN_INTERSECTION     1

#define DRIVE_CTL_DEBOUNCE_COUNTS 2

static TimedEventSimple_t sensor_read_periodic_event;
static Message_t          sensor_read_periodic_msg = {.id = REFARR_PERIODIC_EVENT_MSG_ID,
                                                      .msg_size = sizeof(Message_t)};
static TIM_HandleTypeDef  htim11; // output compare no output
static TIM_HandleTypeDef  htim4; // input capture
static TIM_HandleTypeDef  htim5; // input capture
static GPIO_InitTypeDef   gpioc = {0};

UART_HandleTypeDef huart2;

static uint16_t       sensor_values[6];
static uint16_t       read_buffer[6];
static const uint16_t default_vals[6] = {MAX_READING, MAX_READING, MAX_READING,
                                         MAX_READING, MAX_READING, MAX_READING};

static uint16_t max_sensor_readings[6] = {3313, 2621, 1934, 1707, 1721, 1764};
static uint16_t min_sensor_readings[6] = {71, 65, 55, 68, 106, 148};

static uint32_t last_val = 2500;

static uint8_t glob_count = 0;

static bool drive_control_loop_enabled = true;
static bool left_intersection_enabled = true;
static bool right_intersection_enabled = true;

// current states
static uint32_t state = STATE_DISABLED;
static uint32_t calibration_state = STATE_CALIBRATE_LEFT_TURN;

static uint16_t intersection_count_target = 0;
static uint16_t intersection_count_current = 0;
static uint16_t intersection_debounce_count = DRIVE_CTL_DEBOUNCE_COUNTS;
static uint16_t intersection_debounce_count_current = 0;
static uint16_t intersection_exit_debounce_count_current = 0;
static uint32_t previous_line_state = NOT_IN_INTERSECTION;
static uint32_t current_line_state = NOT_IN_INTERSECTION;

static ActiveObject_t* line_follow_done_response = NULL;

static void HandlePeriodicEvent();
static void HandleStartCalibration();
static void HandleTimedTurnDoneMsg();
static void StartLineFollow(LineFollowMessage_t* msg);
static void StopLineFollow();
static void HandleSensorRead();

__attribute__((__interrupt__)) extern void TIM1_TRG_COM_TIM11_IRQHandler()
{
    OS_ISR_ENTER();
    HAL_TIM_IRQHandler(&htim11);
    OS_ISR_EXIT();
}

__attribute__((__interrupt__)) extern void TIM4_IRQHandler()
{
    OS_ISR_ENTER();
    HAL_TIM_IRQHandler(&htim4);
    OS_ISR_EXIT();
}

__attribute__((__interrupt__)) extern void TIM5_IRQHandler()
{
    OS_ISR_ENTER();
    HAL_TIM_IRQHandler(&htim5);
    OS_ISR_EXIT();
}

static void HandlePeriodicEvent()
{
    // glob_count = 0;
    //  initialize read buffer
    memcpy(read_buffer, default_vals, sizeof(read_buffer));

    // Set GPIOs as output
    gpioc.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &gpioc);

    // write GPIOs
    GPIOC->BSRR = gpioc.Pin;

    // start sensor read sequence
    TIM11->CNT = 0;
    TIM11->CCR1 = SENSOR_CHARGE_DELAY;
    __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_CC1);
    TIM11->CR1 |= 1;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        TIM11->CR1 &= ~((uint32_t)1);
        if (TIM11->CCR1 == SENSOR_CHARGE_DELAY)
        {
            if (glob_count > 0)
            {
                TIM4->CR1 &= ~((uint32_t)1);
                TIM5->CR1 &= ~((uint32_t)1);
                TIM4->CNT = 0;
                TIM5->CNT = 0;
                glob_count = 0;
            }
            TIM11->CNT = 0;
            TIM11->CCR1 = MAX_READING;
            gpioc.Mode = GPIO_MODE_INPUT;
            HAL_GPIO_Init(GPIOC, &gpioc);
            HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
            HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
            HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
            HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
            HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
            HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
            __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_CC1);
            TIM4->CR1 |= 1;
            TIM5->CR1 |= 1;
            TIM11->CR1 |= 1;
        }
        else
        {
            if (glob_count < 6)
            {
                TIM4->CR1 &= ~((uint32_t)1);
                TIM5->CR1 &= ~((uint32_t)1);
                glob_count = 0;
                TIM4->CNT = 0;
                TIM5->CNT = 0;

                memcpy(sensor_values, read_buffer, sizeof(sensor_values));
                Message_t pr_msg = {.id = REFARR_PROCESS_READING_MSG_ID,
                                    .msg_size = sizeof(Message_t)};
                MsgQueuePut(&refarr_ss_ao, &pr_msg);
            }
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM4)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            read_buffer[0] = htim->Instance->CCR1;
            ++glob_count;
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            read_buffer[1] = htim->Instance->CCR2;
            ++glob_count;
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            read_buffer[2] = htim->Instance->CCR3;
            ++glob_count;
        }
        else
        {
            read_buffer[3] = htim->Instance->CCR4;
            ++glob_count;
        }
    }
    else if (htim->Instance == TIM5)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            read_buffer[4] = htim->Instance->CCR1;
            ++glob_count;
        }
        else
        {
            read_buffer[5] = htim->Instance->CCR2;
            ++glob_count;
        }
    }
    if (glob_count >= 6)
    {
        glob_count = 0;
        TIM4->CR1 &= ~((uint32_t)1);
        TIM5->CR1 &= ~((uint32_t)1);
        TIM4->CNT = 0;
        TIM5->CNT = 0;
        glob_count = 0;
        memcpy(sensor_values, read_buffer, sizeof(sensor_values));
        Message_t pr_msg = {.id = REFARR_PROCESS_READING_MSG_ID, .msg_size = sizeof(Message_t)};
        MsgQueuePut(&refarr_ss_ao, &pr_msg);
    }
}

static void HandleStartCalibration()
{
    state = STATE_CALIBRATE;

    TimedEventSimpleCreate(&sensor_read_periodic_event, &refarr_ss_ao, &sensor_read_periodic_msg,
                           REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&sensor_read_periodic_event);
    // start calibration process with timed turn left
    DriveTimedTurn_t ttmsg;
    ttmsg.base.id = DRIVE_TIMED_TURN_MSG_ID;
    ttmsg.base.msg_size = sizeof(DriveTimedTurn_t);
    ttmsg.direction = DRIVE_TURN_DIR_LEFT;
    ttmsg.time = CALIBRATION_TIMED_TURN_DURATION;
    ttmsg.response = &refarr_ss_ao;

    MsgQueuePut(&drive_ss_ao, &ttmsg);

    calibration_state = STATE_CALIBRATE_LEFT_TURN;
}

static void HandleTimedTurnDoneMsg()
{
    if (STATE_CALIBRATE_LEFT_TURN == calibration_state)
    {
        // second calibration step, right rotation
        DriveTimedTurn_t ttmsg;
        ttmsg.base.id = DRIVE_TIMED_TURN_MSG_ID;
        ttmsg.base.msg_size = sizeof(DriveTimedTurn_t);
        ttmsg.direction = DRIVE_TURN_DIR_RIGHT;
        ttmsg.response = &refarr_ss_ao;
        ttmsg.time = 2 * CALIBRATION_TIMED_TURN_DURATION;

        MsgQueuePut(&drive_ss_ao, &ttmsg);
        calibration_state = STATE_CALIBRATE_RIGHT_TURN;
    }
    else if (STATE_CALIBRATE_RIGHT_TURN == calibration_state)
    {
        // third calibration step, left rotation back to start
        DriveTimedTurn_t ttmsg;
        ttmsg.base.id = DRIVE_TIMED_TURN_MSG_ID;
        ttmsg.base.msg_size = sizeof(DriveTimedTurn_t);
        ttmsg.direction = DRIVE_TURN_DIR_LEFT;
        ttmsg.response = &refarr_ss_ao;
        ttmsg.time = CALIBRATION_TIMED_TURN_DURATION;

        MsgQueuePut(&drive_ss_ao, &ttmsg);
        calibration_state = STATE_CALIBRATE_RETURN;
    }
    else if (STATE_CALIBRATE_RETURN == calibration_state)
    {
        uint8_t i, recalibrate = 0;

        // if bad calibration: recalibrate
        for (; i < 6; ++i)
        {
            if (((int)max_sensor_readings[i] - (int)min_sensor_readings[i]) < NOISE_THRESHOLD)
            {
                recalibrate = 1;
            }
        }

        if (recalibrate)
        {
            Message_t calibrate_msg = {.id = REFARR_CALIBRATE_MSG_ID,
                                       .msg_size = sizeof(Message_t)};
            MsgQueuePut(&refarr_ss_ao, &calibrate_msg);
        }

        else
        {
            Message_t cd_msg = {.id = SM_CALIBRATE_DONE, .msg_size = sizeof(Message_t)};
            MsgQueuePut(&state_ctl_ao, &cd_msg);
            state = STATE_FUNCTIONAL;
        }
    }
}

static void HandleSensorRead()
{
    if (STATE_LINE_FOLLOWING == state)
    {
        uint32_t sum = 0;
        uint32_t avg = 0;

        for (uint8_t i = 0; i < 6; i++)
        {
            // calibrate to val between 0 and 1000
            int32_t calmin = min_sensor_readings[i];
            int32_t calmax = max_sensor_readings[i];
            int32_t interval = calmax - calmin;
            int32_t reading = (int32_t)sensor_values[i] - calmin;
            reading *= 1000.0 / (float)interval;

            // adjust max and min values continuously

            if (reading < 0)
            {
                reading = 0;
                // min_sensor_readings[i] = sensor_values[i];
            }
            else if (reading > 1000)
            {
                reading = 1000;
                // max_sensor_readings[i] = sensor_values[i];
            }

            sensor_values[i] = reading;

            if (ABOVE_NOISE_THRESH(reading))
            {
                avg += reading * i * 1000;
                sum += reading;
            }
        }

        bool possibly_at_intersection =
            (left_intersection_enabled && ABOVE_LINE(sensor_values[0])) ||
            (right_intersection_enabled && ABOVE_LINE(sensor_values[5]));

        if (possibly_at_intersection && current_line_state == NOT_IN_INTERSECTION)
        {
            ++intersection_debounce_count_current;
            if (intersection_debounce_count_current >= intersection_debounce_count)
            {
                current_line_state = IN_INTERSECTION;
                intersection_debounce_count_current = 0;
            }
        }
        else if (!possibly_at_intersection && current_line_state == IN_INTERSECTION &&
                 previous_line_state == IN_INTERSECTION)
        {
            ++intersection_exit_debounce_count_current;
            if (intersection_exit_debounce_count_current >= intersection_debounce_count)
            {
                current_line_state = NOT_IN_INTERSECTION;
                intersection_exit_debounce_count_current = 0;
            }
        }
        else
        {
            intersection_exit_debounce_count_current = 0;
            intersection_debounce_count_current = 0;
        }

        if (drive_control_loop_enabled)
        {
            DriveControlMessage_t dcmmsg;
            dcmmsg.base.id = DRIVE_CTL_IN_MSG_ID;
            dcmmsg.base.msg_size = sizeof(DriveControlMessage_t);

            if (!possibly_at_intersection)
            {
                last_val = (float)avg / (float)sum;
            }
            else
            {
                last_val = 2500;
            }

            dcmmsg.actual = last_val;
            MsgQueuePut(&drive_ss_ao, &dcmmsg);
        }

        if (current_line_state == NOT_IN_INTERSECTION && previous_line_state == IN_INTERSECTION)
        {
            ++intersection_count_current;
        }

        if (intersection_count_current >= intersection_count_target)
        {
            Message_t ich_msg = {.id = REFARR_INTERSECTION_COUNT_HIT,
                                 .msg_size = sizeof(Message_t)};
            MsgQueuePut(line_follow_done_response, &ich_msg);

            StopLineFollow();
        }
#ifdef LINE_FOLLOW_TRACE_ENABLED
        LineFollowTrace(sensor_values);
#endif
        previous_line_state = current_line_state;
    }

    // stop taking repeated calibration measurements of sensors
    else if (STATE_CALIBRATE == state)
    {
        for (uint8_t i = 0; i < 6; ++i)
        {
            uint16_t reading = sensor_values[i];
            if (reading > max_sensor_readings[i])
            {
                max_sensor_readings[i] = reading;
            }
            if (reading < min_sensor_readings[i])
            {
                min_sensor_readings[i] = reading;
            }
        }
    }
}

static void StartLineFollow(LineFollowMessage_t* msg)
{
    state = STATE_LINE_FOLLOWING;

    intersection_count_current = 0;
    intersection_exit_debounce_count_current = 0;
    intersection_debounce_count_current = 0;
    intersection_count_target = msg->intersection_count;
    line_follow_done_response = msg->response;

    current_line_state = NOT_IN_INTERSECTION;
    previous_line_state = NOT_IN_INTERSECTION;

    left_intersection_enabled = msg->mode & REFARR_LEFT_SENSOR_ENABLE;
    right_intersection_enabled = msg->mode & REFARR_RIGHT_SENSOR_ENABLE;
    drive_control_loop_enabled = msg->mode & REFARR_DRIVE_CTL_ENABLE;

    intersection_debounce_count = DRIVE_CTL_DEBOUNCE_COUNTS;

    if (drive_control_loop_enabled)
    {
        // load "middle of the line" setpoint into drive subsystem
        // drive control loop will use this setpoint when calculating
        // error using the periodic actual position update from REFARR
        DriveSetpointMessage_t sp_msg;
        sp_msg.base.id = DRIVE_SETPOINT_MSG_ID;
        sp_msg.base.msg_size = sizeof(DriveSetpointMessage_t);
        sp_msg.setpoint = 2500; // middle of line

        DriveBaseVelocityMessage_t bv_msg;
        bv_msg.base.id = DRIVE_BASE_VELOCITY_MSG_ID;
        bv_msg.base.msg_size = sizeof(DriveBaseVelocityMessage_t);
        bv_msg.base_velocity = msg->base_speed;

        MsgQueuePut(&drive_ss_ao, &bv_msg);
        MsgQueuePut(&drive_ss_ao, &sp_msg);
    }

    TimedEventSimpleCreate(&sensor_read_periodic_event, &refarr_ss_ao, &sensor_read_periodic_msg,
                           REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&sensor_read_periodic_event);
}

static void StopLineFollow()
{
    state = STATE_FUNCTIONAL;

    TimedEventDisable(&sensor_read_periodic_event);
}
extern void ReflectanceArrayEventHandler(Message_t* msg)
{
    if (REFARR_CALIBRATE_MSG_ID == msg->id)
    {
        HandleStartCalibration();
    }
    else if (DRIVE_TIMED_TURN_DONE_MSG_ID == msg->id)
    {
        HandleTimedTurnDoneMsg();
    }
    else if (REFARR_START_LINE_FOLLOW_MSG_ID == msg->id)
    {
        StartLineFollow((LineFollowMessage_t*)msg);
    }
    else if (REFARR_STOP_LINE_FOLLOW_MSG_ID == msg->id)
    {
        StopLineFollow();
    }
    else if (REFARR_PERIODIC_EVENT_MSG_ID == msg->id)
    {
        HandlePeriodicEvent();
    }
    else if (REFARR_PROCESS_READING_MSG_ID == msg->id)
    {
        HandleSensorRead();
    }
}

extern void REFARR_Init()
{
    // init gpios that charge/discharge capacitor
    gpioc.Pull = GPIO_NOPULL;
    gpioc.Speed = GPIO_SPEED_FREQ_LOW;
    gpioc.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // timer 11: used to start read sequence
    // enable timer 11 interrupts
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

    // configure timer 4 gpios for input capture; used to measure t(cap. discharge)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // enable timer 4 interrupts
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    // configure timer 5 gpios for input capture; used to measure t(cap. discharge)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // enable timer 5 interrupts
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC = {0};
    TIM_IC_InitTypeDef      sConfigIC = {0};

    // init timer 11: compare timer
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 15;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 65535;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim11);
    HAL_TIM_OC_Init(&htim11);
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1);
    TIM11->CCR1 = SENSOR_CHARGE_DELAY;

    // init timer 4: capture timer
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 15;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
    HAL_TIM_IC_Init(&htim4);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);

    // int timer 5: capture timer
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 15;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4294967295;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim5);

    HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);
    HAL_TIM_IC_Init(&htim5);
    HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2);
}
