
#include <stdint.h>
#include <string.h>

#include <stm/stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include <os.h>

#include "app_defs.h"
#include "rmk_hal_clock_cfg.h"

#include "subsys/drive_subsystem.h"
#include "subsys/input_ctl_subsystem.h"
#include "subsys/reflectance_array_subsystem.h"
#include "subsys/comms_subsystem.h"
#include "subsys/test_subsystem.h"
#include "subsys/electromagnet_subsystem.h"

#include "trace.h"

#include "state_controller.h"

#include "watchdog.h"

//*****************************************************************/
// Periodic Timing Definitions
//*****************************************************************/

#define HEARTBEAT_PERIOD 500

//*****************************************************************/
// Active Object Declarations and Configuration
//*****************************************************************/

ACTIVE_OBJECT_DECL(watchdog_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(input_ctl_ss_ao, INPUT_CTL_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(comms_ss_ao, COMMS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(state_ctl_ao, STATE_MACHINE_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(refarr_ss_ao, REFARR_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(test_ss_ao, TEST_AO_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(electromagnet_ss_ao, EMAG_SS_QUEUE_SIZE)

//*****************************************************************/
// Application and local declarations
//*****************************************************************/

extern OS_t os;

//*****************************************************************/
// Timed Events and their messages
//*****************************************************************/

static TimedEventSimple_t hb_event;
static Message_t          hb_msg = {.id = HEARTBEAT_MSG_ID, .msg_size = sizeof(Message_t)};

static void TimedEventSetup();

static void TimedEventSetup()
{
    // create and schedule timed events.
    TimedEventSimpleCreate(&hb_event, &watchdog_ao, &hb_msg, HEARTBEAT_PERIOD,
                           TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&hb_event);
}

void OnKernelInit()
{
    HAL_NVIC_SetPriority(SysTick_IRQn, OS_BASEPRI, 0);
    HAL_NVIC_EnableIRQ(SysTick_IRQn);

    // set PendSV to lowest priority
    HAL_NVIC_SetPriority(PendSV_IRQn, 0xFF, 0);
    HAL_NVIC_EnableIRQ(PendSV_IRQn);
}

void CheckMessageSizes()
{
    STATIC_ASSERT(sizeof(DriveControlMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(DriveSetpointMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(DriveTimedTurn_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(DriveBaseVelocityMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(DriveOpenLoopControlMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(UartSmallPacketMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(UartLargePacketMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(DispatchMessage_t) <= OS_MESSAGE_MAX_SIZE);
    STATIC_ASSERT(sizeof(LineFollowMessage_t) <= OS_MESSAGE_MAX_SIZE);
}

int main()
{
    // disable interrupts while setting everything up
    DISABLE_INTERRUPTS();
    CheckMessageSizes();

    HAL_Init();

    // initialize all clocks required
    // - system clock
    // - peripheral clocks
    Clock_Init();

    // peripherals init
    Watchdog_Init();

    // initialize subsystems
    Drive_Init();
    REFARR_Init();
    ITCTL_Init();
    Comms_Init();
    EMag_Init();

    TestSystemInit();

    StateController_Init();

    // start subsystems

    // initialize all active objects
    AO_INIT(watchdog_ao, 6, WatchdogEventHandler, HEARTBEAT_QUEUE_SIZE, WATCHDOG_AO_ID);
    AO_INIT(drive_ss_ao, 2, DriveEventHandler, DRIVE_SS_QUEUE_SIZE, DRIVE_AO_ID);
    AO_INIT(input_ctl_ss_ao, 3, InputEventHandler, INPUT_CTL_SS_QUEUE_SIZE, INPUT_CTL_AO_ID);
    AO_INIT(comms_ss_ao, 4, CommsEventHandler, COMMS_QUEUE_SIZE, COMMS_AO_ID);
    AO_INIT(state_ctl_ao, 0, StateControllerEventHandler, STATE_MACHINE_QUEUE_SIZE, STATE_AO_ID);
    AO_INIT(refarr_ss_ao, 1, ReflectanceArrayEventHandler, REFARR_SS_QUEUE_SIZE, REFARR_AO_ID);
    AO_INIT(test_ss_ao, 1, TestSystemEventHandler, TEST_AO_QUEUE_SIZE, TEST_AO_ID);
    AO_INIT(electromagnet_ss_ao, 3, ElectromagnetEventHandler, EMAG_SS_QUEUE_SIZE, EMAG_AO_ID);

    TimedEventSetup();

    // initialze kernel
    OSCallbacksCfg_t os_callback_cfg = {
        .on_Idle = NULL, .on_Init = OnKernelInit, .on_SysTick = NULL, .on_DebugPrint = NULL};

#ifdef OS_TRACE_ENABLED
    os_callback_cfg.on_DebugPrint = DebugPrint;
#endif

    KernelInit(&os, &os_callback_cfg);

    // enable interrupts again
    ENABLE_INTERRUPTS();

    // run scheduler
    SchedulerRun(&os);

    return 0;
}
