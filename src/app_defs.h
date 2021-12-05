#pragma once

#include <os.h>

// share os with all
extern OS_t os;

//*****************************************************************/
// Global constants
//*****************************************************************/

#define LINE_FOLLOW_MAX_BASE_VELOCITY 0.40

//*****************************************************************/
// Custom Messages
//*****************************************************************/

typedef struct DriveControlMessage_s
{
    Message_t base;
    float     actual;
} DriveControlMessage_t;

typedef struct DriveSetpointMessage_s
{
    Message_t base;
    float     setpoint;
} DriveSetpointMessage_t;

typedef struct DriveTimedTurn_s
{
    Message_t       base;
    ActiveObject_t* response;
    uint16_t        time;
    uint8_t         direction;
} DriveTimedTurn_t;

typedef struct DriveBaseVelocityMessage_s
{
    Message_t base;
    float     base_velocity;
} DriveBaseVelocityMessage_t;

typedef struct DriveOpenLoopControlMessage_s
{
    Message_t base;
    float     percent_left;
    float     percent_right;
} DriveOpenLoopControlMessage_t;

typedef struct UartSmallPacketMessage_s
{
    Message_t base;
    uint8_t   length;
    uint8_t   payload[8];
} UartSmallPacketMessage_t;

typedef struct UartLargePacketMessage_s
{
    Message_t base;
    uint16_t  length;
    uint16_t  mem_key;
} UartLargePacketMessage_t;

typedef struct DispatchMessage_s
{
    Message_t base;
    uint8_t   bay_id;
    uint8_t   aisle_id;
} DispatchMessage_t;

typedef struct LineFollowMessage_s
{
    Message_t       base;
    ActiveObject_t* response;
    float           base_speed;
    uint8_t         intersection_count;
    uint8_t
        mode; // 4 most sig bits: drive ctl loop enable, lowest 4 are left turn, regular, right turn
} LineFollowMessage_t;

typedef struct PropertyGetSetMessage_s
{
    Message_t base;
    uint16_t  p_id;
    uint8_t   value[4];
} PropertyGetSetMessage_t;

//*****************************************************************/
// MESSAGE IDs
//*****************************************************************/

// system
#define DATA_MSG_ID        0x0
#define SENSOR_READ_MSG_ID 0x1
#define HEARTBEAT_MSG_ID   0x2

#define DRIVE_MODE_OPEN_LOOP   0x0
#define DRIVE_MODE_CLOSED_LOOP 0x1

// drive subsystem
#define DRIVE_CTL_IN_MSG_ID              0x10
#define DRIVE_DISABLE_MSG_ID             0x11
#define DRIVE_ENABLE_MSG_ID              0x12
#define DRIVE_TIMED_ACTIVITY_MSG_ID      0x13
#define DRIVE_BASE_VELOCITY_MSG_ID       0x14
#define DRIVE_SETPOINT_MSG_ID            0x15
#define DRIVE_TOGGLE_MSG_ID              0x16
#define DRIVE_RAMP_TEST_ITERATION_MSG_ID 0x17
#define DRIVE_TIMED_TURN_MSG_ID          0x18
#define DRIVE_TIMED_TURN_DONE_MSG_ID     0x19
#define DRIVE_OPEN_LOOP_MSG_ID           0x1A
#define DRIVE_PERIODIC_EVENT_MSG_ID      0x1B

#define DRIVE_TURN_DIR_LEFT  0
#define DRIVE_TURN_DIR_RIGHT 1

#define REFARR_LEFT_SENSOR_ENABLE  0x1
#define REFARR_RIGHT_SENSOR_ENABLE 0x2
#define REFARR_DRIVE_CTL_ENABLE    0x10

// reflectance array //ask about priorities
#define REFARR_CALIBRATE_MSG_ID         0x30
#define REFARR_ON_MSG_ID                0x32
#define REFARR_OFF_MSG_ID               0x33
#define REFARR_START_READ_MSG_ID        0x34
#define REFARR_STOP_READ_MSG_ID         0x35
#define REFARR_PERIODIC_EVENT_MSG_ID    0x36
#define REFARR_START_LINE_FOLLOW_MSG_ID 0x37
#define REFARR_STOP_LINE_FOLLOW_MSG_ID  0x38
#define REFARR_INTERSECTION_COUNT_HIT   0x39
#define REFARR_PROCESS_READING_MSG_ID   0x3A

#define PUSH_BUTTON_PRESSED_MSG_ID 0x61

#define TEST_LF_MSG_ID   0x80
#define TEST_TURN_MSG_ID 0x81
#define TEST_180_MSG_ID  0x82

// comms subsystem
#define UART_SMALL_PACKET_MSG_ID    0x81
#define UART_LARGE_PACKET_MSG_ID    0x82
#define OS_DEBUG_MSG_ID             0x83
#define DRIVE_CTL_TRACE_MSG_ID      0x84
#define DRIVE_CTL_TRACE_INIT_MSG_ID 0x85
#define LINE_FOLLOW_TRACE_MSG_ID    0x86

// main state machine
#define SM_PERIODIC_EVENT_MSG_ID 0x100

#define SM_DISPATCH_FROM_IDLE_MSG_ID 0x110
#define SM_CALIBRATE_DONE            0x120

//*****************************************************************/
// Property Management
//*****************************************************************/

#define MSG_P_GET_ID          0x10
#define MSG_P_GET_RESPONSE_ID 0x11

#define MSG_P_SET_ID 0x20

#define GET_PROPERTY_MSG_ID 0x220
#define SET_PROPERTY_MSG_ID 0x221

#define GET_PROPERTY(var, vartype)                                                                 \
    do                                                                                             \
    {                                                                                              \
        UartSmallPacketMessage_t msg;                                                              \
        msg.base.id = UART_SMALL_PACKET_MSG_ID;                                                    \
        msg.base.msg_size = sizeof(UartSmallPacketMessage_t);                                      \
        msg.payload[0] = MSG_P_GET_RESPONSE_ID;                                                    \
        msg.length = 5;                                                                            \
        vartype* start = (vartype*)(msg.payload + 1);                                              \
        *start = var;                                                                              \
        MsgQueuePut(&comms_ss_ao, &msg);                                                           \
    } while (false);

#define SET_PROPERTY(msg, var, vartype)                                                            \
    do                                                                                             \
    {                                                                                              \
        var = *((vartype*)(msg->value));                                                           \
    } while (false);

#define GET_SET_PROPERTY(msg, var, vartype)                                                        \
    do                                                                                             \
    {                                                                                              \
        if (GET_PROPERTY_MSG_ID == msg->base.id)                                                   \
        {                                                                                          \
            GET_PROPERTY(var, vartype);                                                            \
        }                                                                                          \
        else if (SET_PROPERTY_MSG_ID == msg->base.id)                                              \
        {                                                                                          \
            SET_PROPERTY(msg, var, vartype);                                                       \
        }                                                                                          \
    } while (false);

#define DRIVE_PROPERTY_MIN_ID 0x0
#define DRIVE_PROPERTY_MAX_ID 0xA

#define DRIVE_DEADBAND_ID        0x0
#define DRIVE_CTL_LOOP_PERIOD_ID 0x1
#define DRIVE_kP_ID              0x2
#define DRIVE_kI_ID              0x3
#define DRIVE_kD_ID              0x4
#define DRIVE_BASE_OUTPUT_ID     0x5
#define DRIVE_STATE_ID           0x6
#define DRIVE_SETPOINT_ID        0x7
#define DRIVE_ACTUAL_ID          0x8
#define DRIVE_I_ZONE_ID          0x9

//*****************************************************************/
// Commands
//*****************************************************************/

#define COMMAND_ON_END_INSTANT      0x0
#define COMMAND_ON_END_WAIT_FOR_END 0x1

// turn configuration
#define TURN_TYPE_FROM_BASE 0x0
#define TURN_TYPE_FROM_TOP  0x1
#define TURN_TYPE_180_LEFT  0x0
#define TURN_TYPE_180_RIGHT 0x1
#define TURN_DIR_LEFT       0x0
#define TURN_DIR_RIGHT      0x1

#define TIMED_EVENT_DONE_MSG_ID 0x999

typedef struct Command_s Command_t;

struct Command_s
{
    void (*on_Start)(Command_t* cmd, void* instance_data);
    bool (*on_Message)(Command_t* cmd, Message_t* msg, void* instance_data);
    void (*on_End)(Command_t* cmd, void* instance_data);
    uint32_t   end_behavior;
    Command_t* next;
};

//*****************************************************************/
// Active Object Extern Declarations and Configuration
//*****************************************************************/

// message queue sizes
#define HEARTBEAT_QUEUE_SIZE     1
#define DRIVE_SS_QUEUE_SIZE      16
#define REFARR_SS_QUEUE_SIZE     16
#define INPUT_CTL_SS_QUEUE_SIZE  4
#define TEST_OBJ_QUEUE_SIZE      8
#define COMMS_QUEUE_SIZE         8
#define STATE_MACHINE_QUEUE_SIZE 16
#define TEST_AO_QUEUE_SIZE       8

#define WATCHDOG_AO_ID  0x0
#define DRIVE_AO_ID     0x1
#define INPUT_CTL_AO_ID 0x2
#define COMMS_AO_ID     0x3
#define STATE_AO_ID     0x4
#define REFARR_AO_ID    0x5
#define TEST_AO_ID      0x6

ACTIVE_OBJECT_EXTERN(watchdog_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(input_ctl_ss_ao, INPUT_CTL_SS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(comms_ss_ao, COMMS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(state_ctl_ao, STATE_MACHINE_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(refarr_ss_ao, REFARR_SS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(test_ss_ao, TEST_AO_QUEUE_SIZE)
