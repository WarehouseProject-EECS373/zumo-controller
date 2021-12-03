#include "test_subsystem.h"

#include <os.h>
#include <os_msg.h>

#include "app_defs.h"


#define TEST_STATE_INIT             0x0
#define TEST_STATE_LF_DRIVE_CTL     0x1
#define TEST_STATE_LF_TURN          0x2

#define TEST_STATE_LF_TURN_DRIVE_UP     0x10
#define TEST_STATE_LF_TURN_TURN         0x11
#define TEST_STATE_LF_TURN_DRIVE_OUT    0x12

static DriveOpenLoopControlMessage_t cached_drive_control_msg;
static float cached_base_velocity;
static uint8_t cached_mode;
static bool drive_cache_current = false;
static bool refarr_cache_current = false;

static uint32_t test_state = TEST_STATE_INIT;
static uint32_t test_sub_state = TEST_STATE_INIT;

static void StopDrive();

static void StopDrive()
{
    // stop drive
    DriveOpenLoopControlMessage_t olctl_msg;
    olctl_msg.base.id = DRIVE_OPEN_LOOP_MSG_ID;
    olctl_msg.base.msg_size = sizeof(DriveOpenLoopControlMessage_t);
    olctl_msg.percent_left = 0.0;
    olctl_msg.percent_right = 0.0;

    MsgQueuePut(&drive_ss_ao, &olctl_msg);
}

extern void TestSystemEventHandler(Message_t *msg)
{
    if (TEST_STATE_INIT == test_state)
    {
        if (DRIVE_OPEN_LOOP_MSG_ID == msg->id)
        {
            os_memcpy(&cached_drive_control_msg, msg, sizeof(DriveOpenLoopControlMessage_t));
            drive_cache_current = true;
        }
        else if (REFARR_START_LINE_FOLLOW_MSG_ID == msg->id)
        {
            LineFollowMessage_t *lf_msg = (LineFollowMessage_t *)msg;
            lf_msg->response = &test_ss_ao;

            if (lf_msg->mode & REFARR_DRIVE_CTL_ENABLE)
            {
                test_state = TEST_STATE_LF_DRIVE_CTL;
            }
            else
            {
                test_state = TEST_STATE_LF_TURN;
                test_sub_state = TEST_STATE_LF_TURN_DRIVE_UP;

                cached_base_velocity = lf_msg->base_speed;
                cached_mode = lf_msg->mode;
                
                refarr_cache_current = true;

                lf_msg->intersection_count = 1;
                lf_msg->mode = REFARR_DRIVE_CTL_ENABLE | REFARR_LEFT_SENSOR_ENABLE | REFARR_RIGHT_SENSOR_ENABLE;
            }

            MsgQueuePut(&refarr_ss_ao, lf_msg);
        }
    }
    else if (TEST_STATE_LF_DRIVE_CTL == test_state)
    {
        if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
        {
            // stop drive
            StopDrive();
        }
    }
    else if (TEST_STATE_LF_TURN == test_state)
    {
        if (TEST_STATE_LF_TURN_DRIVE_UP == test_sub_state)
        {
            if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
            {
                LineFollowMessage_t lfmsg;
                lfmsg.base.id = REFARR_START_LINE_FOLLOW_MSG_ID;
                lfmsg.base.msg_size = sizeof(LineFollowMessage_t);
                lfmsg.intersection_count = 1;
                lfmsg.response = &test_ss_ao;
                lfmsg.mode = cached_mode;

                MsgQueuePut(&drive_ss_ao, &cached_drive_control_msg);
                MsgQueuePut(&refarr_ss_ao, &lfmsg);

                test_sub_state = TEST_STATE_LF_TURN_TURN;
                drive_cache_current = false;

            }
        }
        else if (TEST_STATE_LF_TURN_TURN == test_sub_state)
        {
            if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
            {
                test_sub_state = TEST_STATE_LF_TURN_DRIVE_OUT;

                LineFollowMessage_t lfmsg;

                lfmsg.base.id = REFARR_START_LINE_FOLLOW_MSG_ID;
                lfmsg.base.msg_size = sizeof(LineFollowMessage_t);
                lfmsg.intersection_count = 1;
                lfmsg.response = &test_ss_ao;
                lfmsg.base_speed = cached_base_velocity;
                lfmsg.mode = REFARR_DRIVE_CTL_ENABLE | REFARR_LEFT_SENSOR_ENABLE | REFARR_RIGHT_SENSOR_ENABLE;

                refarr_cache_current = false;

                MsgQueuePut(&refarr_ss_ao, &lfmsg);
            }
        }
        else if (TEST_STATE_LF_TURN_DRIVE_OUT == test_sub_state)
        {
            if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
            {
                StopDrive();
                test_state = TEST_STATE_INIT;
                test_sub_state = TEST_STATE_INIT;
            }
        }
    }

}


extern void TestSystemInit()
{
}
