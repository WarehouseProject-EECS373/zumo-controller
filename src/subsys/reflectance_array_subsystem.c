#include "reflectance_array_subsystem.h"

#include <os.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32f4xx.h"

#include "trace.h"

// states for periodic event
#define STATE_DISABLED       0x0
#define STATE_CALIBRATE      0x1
#define STATE_FUNCTIONAL     0x2
#define STATE_LINE_FOLLOWING 0x3

// calibration states
#define STATE_CALIBRATE_LEFT_TURN   0x0
#define STATE_CALIBRATE_RIGHT_TURN  0x1
#define STATE_CALIBRATE_RETURN      0x2

// duration of turn of calibration
// should not really be timed but we don't have encoders
// can go to gyro/magnetometer eventually if have time
#define CALIBRATION_TIMED_TURN_DURATION 500


#define REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD  10

static TimedEventSimple_t line_follow_periodic_event;
static  Message_t line_follow_periodic_msg = {.id = REFARR_PERIODIC_EVENT_MSG_ID, .msg_size = sizeof(Message_t)}; 


static uint16_t sensor_values[6];

// current states
static uint32_t state = STATE_DISABLED;
static uint32_t calibration_state = STATE_CALIBRATE_LEFT_TURN;

static uint16_t intersection_count_target = 0;
static uint16_t intersection_count_current = 0;
static ActiveObject_t* line_follow_done_response = NULL;

static void HandlePeriodicEvent();
static void HandleStartCalibration();
static void HandleTimedTurnDoneMsg();
static void StartLineFollow(LineFollowMessage_t *msg);
static void StopLineFollow();

static void HandleStartCalibration()
{
    state = STATE_CALIBRATE;
        
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
        // TODO: stop taking repeated calibration measurements of sensors

        Message_t cd_msg = {.id = SM_CALIBRATE_DONE, .msg_size = sizeof(Message_t)};
        MsgQueuePut(&state_ctl_ao, &cd_msg);

        state = STATE_FUNCTIONAL;
    }
}

static void HandlePeriodicEvent()
{
    if (STATE_LINE_FOLLOWING == state)
    {
        if (intersection_count_current > intersection_count_target)
        {
            Message_t ich_msg = {.id = REFARR_INTERSECTION_COUNT_HIT, .msg_size = sizeof(Message_t)};
            MsgQueuePut(line_follow_done_response, &ich_msg);

            StopLineFollow();

        }

#ifdef LINE_FOLLOW_TRACE_ENABLED
        LineFollowTrace(sensor_values);
#endif

        // TODO: take measurements
        // TODO: get new actual position and send to drive
        // TODO: monitor intersection count and send message to state controller
    }
}

static void StartLineFollow(LineFollowMessage_t *msg)
{
    state = STATE_LINE_FOLLOWING;

    intersection_count_current = 0;
    intersection_count_target = msg->intersection_count;
    line_follow_done_response = msg->response;


    // load "middle of the line" setpoint into drive subsystem
    // drive control loop will use this setpoint when calculating
    // error using the periodic actual position update from REFARR
    DriveSetpointMessage_t sp_msg;
    sp_msg.base.id = DRIVE_SETPOINT_MSG_ID;
    sp_msg.base.msg_size = sizeof(DriveSetpointMessage_t);
    sp_msg.setpoint = 0.0; // FIXME: what is middle of robot?

    DriveBaseVelocityMessage_t bv_msg;
    bv_msg.base.id = DRIVE_BASE_VELOCITY_MSG_ID;
    bv_msg.base.msg_size = sizeof(DriveBaseVelocityMessage_t);
    bv_msg.base_velocity = msg->base_speed;

    MsgQueuePut(&drive_ss_ao, &sp_msg);
    MsgQueuePut(&drive_ss_ao, &bv_msg);


    TimedEventSimpleCreate(&line_follow_periodic_event, &refarr_ss_ao, &line_follow_periodic_msg, REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&line_follow_periodic_event);
}

static void StopLineFollow()
{
    state = STATE_FUNCTIONAL;

    DriveOpenLoopControlMessage_t olctl_msg;
    olctl_msg.base.id = DRIVE_OPEN_LOOP_MSG_ID;
    olctl_msg.base.msg_size = sizeof(DriveOpenLoopControlMessage_t);
    olctl_msg.percent_left = 0.0;
    olctl_msg.percent_right = 0.0;

    MsgQueuePut(&drive_ss_ao, &olctl_msg);
    TimedEventDisable(&line_follow_periodic_event);
}

extern void REFARR_Init()
{
}


extern void ReflectanceArrayEventHandler(Message_t* msg)
{
    if (msg->id == REFARR_CALIBRATE_MSG_ID)
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
}
