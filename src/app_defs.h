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
#define CREATE_DATA_MSG_ID 0x2
#define DATA_MSG_ID 0x3
#define DAC_OUTPUT_DATA_MSG_ID 0x4
#define HEARTBEAT_MSG_ID 0x5
#define LOG_MSG_ID 0x6

//*****************************************************************/
// Active Object Extern Declarations and Configuration
//*****************************************************************/

#define PRODUCER_QUEUE_SIZE 32
#define AVERAGER_QUEUE_SIZE 16
#define INTEGRATOR_QUEUE_SIZE 16
#define DIFFERENTIATOR_QUEUE_SIZE 16
#define DAC_QUEUE_SIZE 16
#define HEARTBEAT_QUEUE_SIZE 2
#define UART_QUEUE_SIZE 32
#define PWM_QUEUE_SIZE 32

ACTIVE_OBJECT_EXTERN(data_producer_ao, PRODUCER_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(averager_ao, AVERAGER_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(integrator_ao, INTEGRATOR_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(differentiator_ao, DIFFERENTIATOR_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(dac1_ao, DAC_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(dac2_ao, DAC_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(heartbeat_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(uart_ao, UART_QUEUE_SIZE)
ACTIVE_OBJECT_EXTERN(pwm_ao, PWM_QUEUE_SIZE)

#endif