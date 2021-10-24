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

//*****************************************************************/
// MESSAGE IDs
//*****************************************************************/

#define DATA_MSG_ID        0x0
#define SENSOR_READ_MSG_ID 0x1
#define HEARTBEAT_MSG_ID   0x2

#define DRIVE_CTL_IN_MSG_ID         0x10
#define DRIVE_DISABLE_MSG_ID        0x11
#define DRIVE_ENABLE_MSG_ID         0x12
#define DRIVE_TIMED_ACTIVITY_MSG_ID 0x13
#define DRIVE_BASE_VELOCITY_MSG_ID  0x14
#define DRIVE_SETPOINT_MSG_ID       0x14

#define PUSH_BUTTON_PRESSED_MSG_ID 0x21

//*****************************************************************/
// Active Object Extern Declarations and Configuration
//*****************************************************************/

#define HEARTBEAT_QUEUE_SIZE 1
#define DRIVE_SS_QUEUE_SIZE  16

ACTIVE_OBJECT_EXTERN(heartbeat_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)

#endif