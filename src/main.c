
#include <stdint.h>
#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"
#include "rmk_hal_clock_cfg.h"

#include "gpio_ao.h"
#include "pwm_ao.h"

#define HEARTBEAT_PERIOD 500

//*****************************************************************/
// Active Object Declarations and Configuration
//*****************************************************************/

ACTIVE_OBJECT_DECL(heartbeat_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(pwm_ao, PWM_QUEUE_SIZE)

//*****************************************************************/
// Application and local declarations
//*****************************************************************/

extern OS_t os;

static TimedEventSimple_t hb_event;
static Message_t hb_msg = {.id = HEARTBEAT_MSG_ID, .msg_size = sizeof(Message_t)};

void HeartbeatHandler(Message_t *msg)
{
    if (HEARTBEAT_MSG_ID == msg->id)
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

    // initialize peripherals
    GPIO_Init();
    PWM_Init();

    // start peripherals
    PWM_Start();

    // initialize all active objects
    AO_INIT(heartbeat_ao, 6, HeartbeatHandler, HEARTBEAT_QUEUE_SIZE)
    AO_INIT(pwm_ao, 2, DualPwmEventHandler, PWM_QUEUE_SIZE)

    // create and schedule timed events.
    TimedEventSimpleCreate(&hb_event, &heartbeat_ao, &hb_msg, HEARTBEAT_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&hb_event);

    // initialze kernel
    OSCallbacksCfg_t os_callback_cfg = {.on_Idle = NULL, .on_Init = OnKernelInit, .on_SysTick = NULL};
    KernelInit(&os, &os_callback_cfg);

    // enable interrupts again
    ENABLE_INTERRUPTS();

    // run scheduler
    SchedulerRun(&os);

    return 0;
}