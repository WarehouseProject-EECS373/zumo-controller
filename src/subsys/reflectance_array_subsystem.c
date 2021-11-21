#include "reflectance_array_subsystem.h"

#include <os.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32f4xx.h"

#define STATE_DISABLED       0x0
#define STATE_CALIBRATE      0x1
#define STATE_FUNCTIONAL     0x2

// calibration states
#define STATE_CALIBRATE_LEFT_TURN   0x0
#define STATE_CALIBRATE_RIGHT_TURN  0x1
#define STATE_CALIBRATE_RETURN      0x2

// duration of turn of calibration
// should not really be timed but we don't have encoders
// can go to gyro/magnetometer eventually if have time
#define CALIBRATION_TIMED_TURN_DURATION 500

// current states
static uint32_t state = STATE_DISABLED;
static uint32_t calibration_state = STATE_CALIBRATE_LEFT_TURN;

static void HandlePeriodicEvent();
static void HandleTimedTurnDoneMsg();

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
        state = STATE_FUNCTIONAL;
    }
}

static void HandlePeriodicEvent()
{
    // 
}

extern void REFARR_Init()
{
}


extern void ReflectanceArrayEventHandler(Message_t* msg)
{
    if (msg->id == REFARR_CALIBRATE_MSG_ID)
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

        // TODO: start a periodic event to take repeated measurements while drive is turning
    }
    else if (DRIVE_TIMED_TURN_DONE_MSG_ID == msg->id)
    {
        HandleTimedTurnDoneMsg();
    }
    else if (REFARR_PERIODIC_EVENT_MSG_ID == msg->id)
    {
        HandlePeriodicEvent();       
    }
}
