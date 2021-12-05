#include "state_controller.h"

#include <os.h>

#include "app_defs.h"

#include "state_machine.h"

#include "cmd/delay_cmd.h"
#include "cmd/drive_open_loop_cmd.h"
#include "cmd/lf_cmd.h"
#include "cmd/turn180_cmd.h"
#include "cmd/turn_cmd.h"
#include "cmd/bay_dropoff_cmd.h"

#define IDLE_STATE          0x1
#define DISPATCHED_STATE    0x2

#define TURN_SPEED_FW  0.55
#define TURN_SPEED_REV -0.6
#define MAX_BAY_COUNT  3

static uint32_t state = IDLE_STATE;

static StateMachine_t state_machine;

static LineFollowCommand_t leave_idle_lf_cmd;
static TurnCommand_t       aisle_1_turn_cmd;
static BayDropoffCommand_t bay_dropoff_cmd;
static TurnCommand_t       aisle_1_leave_cmd;

static void HandleDisptach(DispatchMessage_t* msg);

static void HandleDisptach(DispatchMessage_t* msg)
{
    uint8_t aisle_id = msg->aisle_id;
    uint8_t bay_id = msg->bay_id;

    // bay id
    //  odd for right turn in aisle 1
    //  even for left turn in aisle 2
    // so to get actual index (intersection count) divide by 2 add 1
    uint8_t bay_index = (bay_id / 2) + 1;

    uint8_t lf_mode =
        REFARR_DRIVE_CTL_ENABLE | REFARR_LEFT_SENSOR_ENABLE | REFARR_RIGHT_SENSOR_ENABLE;

    // turn into aisle 1 if necessary
    if (1 == aisle_id)
    {
        // stop at first intersection
       uint8_t leave_idle_intersection_count = 1;

        // line follow until first intersection
        LineFollowCommandInit(&leave_idle_lf_cmd, lf_mode, leave_idle_intersection_count,
                              LINE_FOLLOW_MAX_BASE_VELOCITY, 0, (Command_t*)&aisle_1_turn_cmd);

        // turn left at first intersection, drive until bay index
        TurnCommandInit(&aisle_1_turn_cmd, TURN_DIR_LEFT, bay_index, TURN_TYPE_FROM_TOP,
                        TURN_SPEED_FW, TURN_SPEED_REV, (Command_t*)&bay_dropoff_cmd);

        // dropoff, drive until last intersection in aisle (remaining bays + 1 for exit
        // intersection)
        BayDropoffCommandInit(&bay_dropoff_cmd, bay_id,  2 - bay_index + 1,
                              (Command_t*)&aisle_1_leave_cmd);

        // start turn, then drive until idle position
        TurnCommandInit(&aisle_1_leave_cmd, TURN_DIR_LEFT, 1, TURN_TYPE_FROM_BASE, TURN_SPEED_FW,
                        TURN_SPEED_REV, NULL);
    }
    else
    {
        // +1 for aisle 1 intersection, can drive straight through first intersection
        uint8_t leave_idle_intersection_count = 1 + bay_index;

        // start line following, arrive at bay
        LineFollowCommandInit(&leave_idle_lf_cmd, lf_mode, leave_idle_intersection_count,
                              LINE_FOLLOW_MAX_BASE_VELOCITY, 0, (Command_t*)&bay_dropoff_cmd);

        // dropoff and exit, drive until idle (remaining bays in aisle + 1 for aisle 1 exit + 1 for
        // idle position)
        BayDropoffCommandInit(&bay_dropoff_cmd, bay_id, 4 - bay_index + 2, NULL);
    }

    StateMachineInit(&state_machine, (Command_t*)&leave_idle_lf_cmd);
    StateMachineStart(&state_machine, NULL);
}

extern void StateController_Init()
{
}

extern void StateControllerEventHandler(Message_t* msg)
{
    if (IDLE_STATE == state && SM_DISPATCH_FROM_IDLE_MSG_ID == msg->id)
    {
        state = DISPATCHED_STATE;
        HandleDisptach((DispatchMessage_t*)msg);
    }
    else
    {
        // if message comes to state machine and it's not a dispatch, send it to state machine
        // SM will handle message using current command
        if (StateMachineStep(&state_machine, msg, NULL))
        {
            state = IDLE_STATE;
            DriveOpenLoopControlMessage_t stop_msg;
            stop_msg.base.id = DRIVE_OPEN_LOOP_MSG_ID;
            stop_msg.base.msg_size = sizeof(DriveOpenLoopControlMessage_t);
            stop_msg.percent_left = 0.0;
            stop_msg.percent_right = 0.0;

            MsgQueuePut(&drive_ss_ao, &stop_msg);
        }
    }
}
