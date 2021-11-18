
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

#include "watchdog.h"

//*****************************************************************/
// Periodic Timing Definitions
//*****************************************************************/

#define HEARTBEAT_PERIOD               500
#define IR_SENSOR_READ_PERIOD          3
#define DRIVE_SS_TIMED_ACTIVITY_PERIOD 5
#define DRIVE_SS_RAMP_TEST_PERIOD      100
#define COMMS_TEST_PERIOD              225

//*****************************************************************/
// Active Object Declarations and Configuration
//*****************************************************************/

ACTIVE_OBJECT_DECL(watchdog_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(drive_ss_ao, DRIVE_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(input_ctl_ss_ao, INPUT_CTL_SS_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(comms_ss_ao, COMMS_QUEUE_SIZE)

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

static TimedEventSimple_t test_short_uart_msg;
static UartSmallPacketMessage_t uart_short_msg;

static TimedEventSimple_t test_long_uart_msg;
static UartLargePacketMessage_t uart_large_msg;

static const char* test_str = "hello this test\r\n";


static void TimedEventSetup();

static void TimedEventSetup()
{
    // create and schedule timed events.
    TimedEventSimpleCreate(&hb_event, &watchdog_ao, &hb_msg, HEARTBEAT_PERIOD,
                           TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&hb_event);

    TimedEventSimpleCreate(&drive_ramp_test_event, &drive_ss_ao, &drive_ramp_test_msg,
                           DRIVE_SS_RAMP_TEST_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&drive_ramp_test_event);


    uart_short_msg.base.id = UART_SMALL_PACKET_MSG_ID;
    uart_short_msg.base.msg_size = sizeof(UartSmallPacketMessage_t);
    uart_short_msg.length = 8;
    uint8_t buffer[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    os_memcpy(uart_short_msg.payload, buffer, 8);

    TimedEventSimpleCreate(&test_short_uart_msg, &comms_ss_ao, &uart_short_msg, COMMS_TEST_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&test_short_uart_msg);

    uint16_t key;
    OSStatus_t status;
    uint8_t *block = OSMemoryBlockNew(&key, MEMORY_BLOCK_32, &status);
    
    os_memcpy(block, test_str, strlen(test_str));
    
    uart_large_msg.base.id = UART_LARGE_PACKET_MSG_ID;
    uart_large_msg.base.msg_size = sizeof(UartLargePacketMessage_t);
    uart_large_msg.length = strlen(test_str);
    uart_large_msg.mem_key = key;

    TimedEventSimpleCreate(&test_long_uart_msg, &comms_ss_ao, &uart_large_msg, COMMS_TEST_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&test_long_uart_msg);
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
    Comms_Init();

    // start subsystems

    // initialize all active objects
    AO_INIT(watchdog_ao, 6, WatchdogEventHandler, HEARTBEAT_QUEUE_SIZE)
    AO_INIT(drive_ss_ao, 2, DriveEventHandler, DRIVE_SS_QUEUE_SIZE)
    AO_INIT(input_ctl_ss_ao, 3, InputEventHandler, INPUT_CTL_SS_QUEUE_SIZE)
    AO_INIT(comms_ss_ao, 4, CommsEventHandler, COMMS_QUEUE_SIZE);

    TimedEventSetup();

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
