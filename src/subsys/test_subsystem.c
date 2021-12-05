#include "test_subsystem.h"

#include <os.h>
#include <os_msg.h>

#include "app_defs.h"

#define TEST_STATE_CONFIGURE_LF_DRIVE_CTL   0x90
#define TEST_STATE_CONFIGURE_LF_TURN        0x91
#define TEST_STATE_CONFIGURE_180T           0x92

#define TEST_STATE_INIT             0x0
#define TEST_STATE_LF_DRIVE_CTL     0x1
#define TEST_STATE_LF_TURN          0x2
#define TEST_STATE_180_TURN         0x3

#define TEST_STATE_LF_TURN_DRIVE_UP     0x10
#define TEST_STATE_LF_TURN_TURN         0x11
#define TEST_STATE_LF_TURN_DRIVE_OUT    0x12

#define TEST_STATE_180T_TURN            0x22
#define TEST_STATE_180T_DRIVE_OUT       0x23

#define LF_TURN_DELAY 10

static TimedEventSimple_t post_turn_lf_delay_event;
static LineFollowMessage_t post_turn_lf_delay_msg;

static TimedEventSimple_t turn180_delay_event;
static LineFollowMessage_t turn180_delay_msg;

static TimedEventSimple_t turn180_drive_delay_event;
static LineFollowMessage_t turn180_drive_delay_msg;

static DriveOpenLoopControlMessage_t cached_drive_control_msg;

static float cached_base_velocity;
static uint8_t cached_mode;

static uint32_t test_state = TEST_STATE_INIT;
static uint32_t test_sub_state = TEST_STATE_INIT;

static void ConfigureLFTurn(Message_t *msg);
static void Configure180T(Message_t *msg);
static void ConfigureLF(Message_t *msg);


static void HandleConfigure(Message_t *msg);
static void HandleTestStart(Message_t *msg);
static void HandleTurn(Message_t *msg);
static void Handle180Turn(Message_t *msg);

static void StartDelayedLF(uint32_t delay);
static void StopDrive();

static void StartLF(LineFollowMessage_t *lf_msg);

static void HandleTestStart(Message_t *msg)
{
    if (TEST_LF_MSG_ID == msg->id)
    {
        test_state = TEST_STATE_CONFIGURE_LF_DRIVE_CTL;
    }
    else if (TEST_TURN_MSG_ID == msg->id)
    {
        test_state = TEST_STATE_CONFIGURE_LF_TURN;
    }
    else if (TEST_180_MSG_ID == msg->id)
    {
        test_state = TEST_STATE_CONFIGURE_180T;
    }
}

static void Configure180T(Message_t *msg)
{
    static uint8_t msg_count = 0;

    if (0 == msg_count && DRIVE_OPEN_LOOP_MSG_ID == msg->id)
    {
        ++msg_count;
        os_memcpy(&turn180_drive_delay_msg, msg, sizeof(DriveOpenLoopControlMessage_t));
    }
    else if (1 == msg_count && REFARR_START_LINE_FOLLOW_MSG_ID == msg->id)
    {
        ++msg_count;
        os_memcpy(&turn180_delay_msg, msg, sizeof(LineFollowMessage_t));
        cached_base_velocity = ((LineFollowMessage_t*)msg)->base_speed;
    }
    else if (2 == msg_count && DRIVE_OPEN_LOOP_MSG_ID == msg->id)
    {
        ++msg_count;
        os_memcpy(&cached_drive_control_msg, msg, sizeof(DriveOpenLoopControlMessage_t));
    }
    else if (3 == msg_count && DRIVE_OPEN_LOOP_MSG_ID == msg->id)
    {
        TimedEventSimpleCreate(&turn180_drive_delay_event, &drive_ss_ao, &turn180_drive_delay_msg, 400, TIMED_EVENT_SINGLE_TYPE);
        TimedEventSimpleCreate(&turn180_delay_event, &refarr_ss_ao, &turn180_delay_msg, 400, TIMED_EVENT_SINGLE_TYPE);
        SchedulerAddTimedEvent(&turn180_drive_delay_event);
        SchedulerAddTimedEvent(&turn180_delay_event);
        MsgQueuePut(&drive_ss_ao, msg);
        test_state = TEST_STATE_180_TURN;
        test_sub_state = TEST_STATE_180T_TURN;
        msg_count = 0;
    }
}

static void ConfigureLF(Message_t *msg)
{
    test_state = TEST_STATE_LF_DRIVE_CTL;
    LineFollowMessage_t *lf_msg = (LineFollowMessage_t *)msg;
        
    StartLF(lf_msg);
}

static void ConfigureLFTurn(Message_t *msg)
{
    if (DRIVE_OPEN_LOOP_MSG_ID == msg->id)
    {
        os_memcpy(&cached_drive_control_msg, msg, sizeof(DriveOpenLoopControlMessage_t));
    }
    else if (REFARR_START_LINE_FOLLOW_MSG_ID == msg->id)
    {
        LineFollowMessage_t *lf_msg = (LineFollowMessage_t *)msg;
        test_state = TEST_STATE_LF_TURN;
        test_sub_state = TEST_STATE_LF_TURN_DRIVE_UP;
        cached_base_velocity = lf_msg->base_speed;
        lf_msg->intersection_count = 1;
        
        StartLF(lf_msg);
    }
}

static void StartLF(LineFollowMessage_t *lf_msg)
{
    lf_msg->response = &test_ss_ao;
    cached_mode = lf_msg->mode;
    lf_msg->mode = REFARR_DRIVE_CTL_ENABLE | REFARR_LEFT_SENSOR_ENABLE | REFARR_RIGHT_SENSOR_ENABLE;

    MsgQueuePut(&refarr_ss_ao, lf_msg);
}

static void HandleConfigure(Message_t *msg)
{
    if (TEST_STATE_CONFIGURE_180T == test_state)
    {
        // send open loop msg to cache
        // send LF message to start
        // send open loop msg to execute
        Configure180T(msg);
    }
    else if (TEST_STATE_CONFIGURE_LF_DRIVE_CTL == test_state)
    {
        // 1. send LF message tos start
        ConfigureLF(msg);
    }
    else if (TEST_STATE_CONFIGURE_LF_TURN == test_state)
    {
        // 1. send open loop msg to cache
        // 2. send LF message to start test
        ConfigureLFTurn(msg);
    }
}

static void HandleTurn(Message_t *msg)
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

        }
    }
    else if (TEST_STATE_LF_TURN_TURN == test_sub_state)
    {
        if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
        {
            StartDelayedLF(LF_TURN_DELAY);
            test_sub_state = TEST_STATE_LF_TURN_DRIVE_OUT;
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

static void Handle180Turn(Message_t *msg)
{
    if (TEST_STATE_180T_TURN == test_sub_state)
    {
        if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
        {
            StartDelayedLF(LF_TURN_DELAY);
            test_sub_state = TEST_STATE_180T_DRIVE_OUT;
        }
    }
    else if (TEST_STATE_180T_DRIVE_OUT == test_sub_state)
    {
        if (REFARR_INTERSECTION_COUNT_HIT == msg->id)
        {
            test_state = TEST_STATE_INIT;
            test_sub_state = TEST_STATE_INIT;
            StopDrive();
        }
    }
}

static void StartDelayedLF(uint32_t delay)
{
    // keep driving since StopLF stops drive
    // this should stop when StartLF happens after a delay
    MsgQueuePut(&drive_ss_ao, &cached_drive_control_msg);

    post_turn_lf_delay_msg.base.id = REFARR_START_LINE_FOLLOW_MSG_ID;
    post_turn_lf_delay_msg.base.msg_size = sizeof(LineFollowMessage_t);
    post_turn_lf_delay_msg.intersection_count = 1;
    post_turn_lf_delay_msg.response = &test_ss_ao;
    post_turn_lf_delay_msg.base_speed = cached_base_velocity;
    post_turn_lf_delay_msg.mode = REFARR_DRIVE_CTL_ENABLE | REFARR_LEFT_SENSOR_ENABLE | REFARR_RIGHT_SENSOR_ENABLE;

    TimedEventSimpleCreate(&post_turn_lf_delay_event, &refarr_ss_ao, &post_turn_lf_delay_msg, delay, TIMED_EVENT_SINGLE_TYPE);
    SchedulerAddTimedEvent(&post_turn_lf_delay_event);
}

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
        HandleTestStart(msg);
    }
    else if (TEST_STATE_CONFIGURE_180T == test_state || TEST_STATE_CONFIGURE_LF_DRIVE_CTL == test_state
                || TEST_STATE_CONFIGURE_LF_TURN == test_state)
    {
        HandleConfigure(msg);
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
        HandleTurn(msg);
    }
    else if (TEST_STATE_180_TURN == test_state)
    {
        Handle180Turn(msg);
    }
}


extern void TestSystemInit()
{
}
