#ifndef APP_DEFS_H
#define APP_DEFS_H

#include <os.h>

// share os with all
extern OS_t os;

//*****************************************************************/
// Custom Messages
//*****************************************************************/

typedef struct DriveControlMessage_s
{
    Message_t base;
    float actual;
} DriveControlMessage_t;

typedef struct DriveSetpointMessage_s
{
    Message_t base;
    float setpoint;
    uint32_t drive_mode;
} DriveSetpointMessage_t;

typedef struct DriveBaseVelocityMessage_s
{
    Message_t base;
    float base_velocity;
} DriveBaseVelocityMessage_t;

typedef struct UartSmallPacketMessage_s
{
    Message_t base;
    uint8_t length;
    uint8_t payload[8];
} UartSmallPacketMessage_t;

typedef struct UartLargePacketMessage_s
{
    Message_t base;
    uint16_t length;
    uint16_t mem_key;
} UartLargePacketMessage_t;

typedef struct DispatchMessage_s
{
    Message_t base;
    uint8_t bay_id;
    uint8_t aisle_id;
} DispatchMessage_t;

//*****************************************************************/
// MESSAGE IDs
//*****************************************************************/

// system
#define DATA_MSG_ID        0x0
#define SENSOR_READ_MSG_ID 0x1
#define HEARTBEAT_MSG_ID   0x2

// drive subsystem
#define DRIVE_CTL_IN_MSG_ID              0x10
#define DRIVE_DISABLE_MSG_ID             0x11
#define DRIVE_ENABLE_MSG_ID              0x12
#define DRIVE_TIMED_ACTIVITY_MSG_ID      0x13
#define DRIVE_BASE_VELOCITY_MSG_ID       0x14
#define DRIVE_SETPOINT_MSG_ID            0x15
#define DRIVE_TOGGLE_MSG_ID              0x16
#define DRIVE_RAMP_TEST_ITERATION_MSG_ID 0x17

// reflectance array
#define REFARR_CALIBRATE_MSG_ID  0x30
#define REFARR_ON_MSG_ID         0x31
#define REFARR_OFF_MSG_ID        0x32
#define REFARR_START_READ_MSG_ID 0x33
#define REFARR_STOP_READ_MSG_ID  0x34

#define PUSH_BUTTON_PRESSED_MSG_ID 0x61

// comms subsystem
#define UART_SMALL_PACKET_MSG_ID 0x81
#define UART_LARGE_PACKET_MSG_ID 0x82


// main state machine
#define SM_PERIODIC_EVENT_MSG_ID        0x100

#define SM_DISPATCH_FROM_IDLE_MSG_ID    0x110

//*****************************************************************/
// Active Object Extern Declarations and Configuration
//*****************************************************************/

// message queue sizes
#define HEARTBEAT_QUEUE_SIZE     1
#define DRIVE_SS_QUEUE_SIZE      8
#define REFARR_SS_QUEUE_SIZE     8
#define INPUT_CTL_SS_QUEUE_SIZE  4
#define TEST_OBJ_QUEUE_SIZE      8
#define COMMS_QUEUE_SIZE         8
#define STATE_MACHINE_QUEUE_SIZE 16

ACTIVE_OBJECT_EXTERN(watchdog_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(input_ctl_ss_ao, INPUT_CTL_SS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(comms_ss_ao, COMMS_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(state_ctl_ao, STATE_MACHINE_QUEUE_SIZE)

#endif
