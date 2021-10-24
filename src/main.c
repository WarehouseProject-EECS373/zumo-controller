
#include <stdint.h>
#include <stm/stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"
#include "rmk_hal_clock_cfg.h"

#include "gpio_ao.h"
#include "subsys/drive_subsystem.h"
#include "subsys/input_ctl_subsystem.h"
#include "subsys/reflectance_array_subsystem.h"

//*****************************************************************/
// Periodic Timing Definitions
//*****************************************************************/

#define HEARTBEAT_PERIOD               500
#define IR_SENSOR_READ_PERIOD          3
#define DRIVE_SS_TIMED_ACTIVITY_PERIOD 5

//*****************************************************************/
// Active Object Declarations and Configuration
//*****************************************************************/

ACTIVE_OBJECT_DECL(heartbeat_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(refarr_ss_ao, REFARR_SS_QUEUE_SIZE)

//*****************************************************************/
// Application and local declarations
//*****************************************************************/

extern OS_t os;

//*****************************************************************/
// Timed Events and their messages
//*****************************************************************/

static TimedEventSimple_t hb_event;
static Message_t hb_msg = {.id = HEARTBEAT_MSG_ID, .msg_size = sizeof(Message_t)};

static TimedEventSimple_t ir_sensor_read_event;
static Message_t ir_sensor_read_msg = {.id = SENSOR_READ_MSG_ID, .msg_size = sizeof(Message_t)};

static TimedEventSimple_t drive_ss_ctl_loop_event;
static Message_t drive_ss_ctl_loop_msg = {.id = DRIVE_TIMED_ACTIVITY_MSG_ID,
                                          .msg_size = sizeof(Message_t)};

/**
 * @brief Simple LED heartbeat so we know everything is ok. Tie this into a watchdog eventually.
 *
 * @param msg
 */
void HeartbeatHandler(Message_t* msg)
{
    if(HEARTBEAT_MSG_ID == msg->id)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }
}

void OnKernelInit()
{
    HAL_NVIC_SetPriority(SysTick_IRQn, OS_BASEPRI, 0);
    HAL_NVIC_EnableIRQ(SysTick_IRQn);

    // set PendSV to lowest priority
    HAL_NVIC_SetPriority(PendSV_IRQn, 0xFF, 0);
    HAL_NVIC_EnableIRQ(PendSV_IRQn);
}

int main()
{
    // disable interrupts while setting everything up
    DISABLE_INTERRUPTS();

    HAL_Init();

    // initialize all clocks required
    // - system clock
    // - peripheral clocks
    Clock_Init();

    // peripherals init
    GPIO_Init();

    // initialize subsystems
    Drive_Init();
    ITCTL_Init();
    REFARR_Init();

    // start subsystems

    // initialize all active objects
    AO_INIT(heartbeat_ao, 6, HeartbeatHandler, HEARTBEAT_QUEUE_SIZE)
    AO_INIT(drive_ss_ao, 2, DriveEventHandler, DRIVE_SS_QUEUE_SIZE)
    AO_INIT(refarr_ss_ao, 3, ReflectanceArrayEventHandler, REFARR_SS_QUEUE_SIZE)

    // create and schedule timed events.
    TimedEventSimpleCreate(&hb_event, &heartbeat_ao, &hb_msg, HEARTBEAT_PERIOD,
                           TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&hb_event);

    TimedEventSimpleCreate(&ir_sensor_read_event, &refarr_ss_ao, &ir_sensor_read_msg,
                           IR_SENSOR_READ_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&ir_sensor_read_event);

    TimedEventSimpleCreate(&drive_ss_ctl_loop_event, &drive_ss_ao, &drive_ss_ctl_loop_msg,
                           DRIVE_SS_TIMED_ACTIVITY_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&drive_ss_ctl_loop_event);

    // initialze kernel
    OSCallbacksCfg_t os_callback_cfg = {
        .on_Idle = NULL, .on_Init = OnKernelInit, .on_SysTick = NULL};
    KernelInit(&os, &os_callback_cfg);

    // enable interrupts again
    ENABLE_INTERRUPTS();

    // run scheduler
    SchedulerRun(&os);

    return 0;
}