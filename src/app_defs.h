#ifndef APP_DEFS_H
#define APP_DEFS_H

#include <os.h>

// share os with all
extern OS_t os;

//*****************************************************************/
// Custom Messages
//*****************************************************************/

typedef struct WheelControlMessage_s
{
    Message_t base;
    uint16_t velocity_1;
    uint16_t setpoint_2;
} ControlMessage_t;

//*****************************************************************/
// MESSAGE IDs
//*****************************************************************/

#define PUSH_BUTTON_PRESSED_MSG_ID 0x1
#define DATA_MSG_ID 0x3
#define HEARTBEAT_MSG_ID 0x5

//*****************************************************************/
// Active Object Extern Declarations and Configuration
//*****************************************************************/

#define HEARTBEAT_QUEUE_SIZE 2
#define PWM_QUEUE_SIZE 32

ACTIVE_OBJECT_EXTERN(heartbeat_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(pwm_ao, PWM_QUEUE_SIZE)

#endif