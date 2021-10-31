
#include <stdint.h>
#include <stm/stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"
#include "rmk_hal_clock_cfg.h"

#include "subsys/drive_subsystem.h"
#include "subsys/input_ctl_subsystem.h"
#include "subsys/reflectance_array_subsystem.h"
#include "watchdog.h"

//*****************************************************************/
// Periodic Timing Definitions
//*****************************************************************/

#define HEARTBEAT_PERIOD               500
#define IR_SENSOR_READ_PERIOD          3
#define DRIVE_SS_TIMED_ACTIVITY_PERIOD 5
#define DRIVE_SS_RAMP_TEST_PERIOD      100

//*****************************************************************/
// Active Object Declarations and Configuration
//*****************************************************************/

ACTIVE_OBJECT_DECL(watchdog_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(input_ctl_ss_ao, INPUT_CTL_SS_QUEUE_SIZE)

//*****************************************************************/
// Application and local declarations
//*****************************************************************/

extern OS_t os;

//*****************************************************************/
// Timed Events and their messages
//*****************************************************************/

static TimedEventSimple_t hb_event;
static Message_t hb_msg = {.id = HEARTBEAT_MSG_ID, .msg_size = sizeof(Message_t)};

static TimedEventSimple_t drive_ramp_test_event;
static Message_t drive_ramp_test_msg = {.id = DRIVE_RAMP_TEST_ITERATION_MSG_ID,
                                        .msg_size = sizeof(Message_t)};

void TestObjHandler(Message_t* msg)
{
    // test obj handler, does nothing for now
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
    Watchdog_Init();

    // initialize subsystems
    Drive_Init();
    ITCTL_Init();

    // start subsystems

    // initialize all active objects
    AO_INIT(watchdog_ao, 6, WatchdogEventHandler, HEARTBEAT_QUEUE_SIZE)
    AO_INIT(drive_ss_ao, 2, DriveEventHandler, DRIVE_SS_QUEUE_SIZE)
    AO_INIT(input_ctl_ss_ao, 3, InputEventHandler, INPUT_CTL_SS_QUEUE_SIZE)

    // create and schedule timed events.
    TimedEventSimpleCreate(&hb_event, &watchdog_ao, &hb_msg, HEARTBEAT_PERIOD,
                           TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&hb_event);

    TimedEventSimpleCreate(&drive_ramp_test_event, &drive_ss_ao, &drive_ramp_test_msg,
                           DRIVE_SS_RAMP_TEST_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&drive_ramp_test_event);

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