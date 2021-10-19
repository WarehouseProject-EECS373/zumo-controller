
#include <stdint.h>
#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"
#include "rmk_hal_clock_cfg.h"

#include "gpio_ao.h"
#include "math_ao.h"
#include "dac_ao.h"
#include "uart_ao.h"
#include "pwm_ao.h"

#define HEARTBEAT_PERIOD 500
#define DATA_PERIOD 10

//*****************************************************************/
// Active Object Declarations and Configuration
//*****************************************************************/

ACTIVE_OBJECT_DECL(data_producer_ao, PRODUCER_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(averager_ao, AVERAGER_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(integrator_ao, INTEGRATOR_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(differentiator_ao, DIFFERENTIATOR_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(dac1_ao, DAC_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(dac2_ao, DAC_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(heartbeat_ao, HEARTBEAT_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(uart_ao, UART_QUEUE_SIZE)
ACTIVE_OBJECT_DECL(pwm_ao, PWM_QUEUE_SIZE)

//*****************************************************************/
// Application and local declarations
//*****************************************************************/

extern OS_t os;

static TimedEventSimple_t create_data_event;
static Message_t create_data_msg = {.id = CREATE_DATA_MSG_ID, .msg_size = sizeof(Message_t)};

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
    DAC_Init();
    GPIO_Init();
    UART_Init();
    PWM_Init();

    // start peripherals
    DAC_Start();
    PWM_Start();

    // initialize all active objects
    AO_INIT(data_producer_ao, 1, DataProducerEventHandler, PRODUCER_QUEUE_SIZE)
    AO_INIT(averager_ao, 3, AveragerEventHandler, AVERAGER_QUEUE_SIZE)
    AO_INIT(integrator_ao, 3, IntegratorEventHandler, INTEGRATOR_QUEUE_SIZE)
    AO_INIT(differentiator_ao, 3, DifferentiatorEventHandler, DIFFERENTIATOR_QUEUE_SIZE)
    AO_INIT(dac1_ao, 5, DAC1EventHandler, DAC_QUEUE_SIZE)
    AO_INIT(dac2_ao, 5, DAC2EventHandler, DAC_QUEUE_SIZE)
    AO_INIT(heartbeat_ao, 6, HeartbeatHandler, HEARTBEAT_QUEUE_SIZE)
    AO_INIT(uart_ao, 6, UARTEventHandler, UART_QUEUE_SIZE)
    AO_INIT(pwm_ao, 2, DualPwmEventHandler, PWM_QUEUE_SIZE)

    // create and schedule timed events.
    TimedEventSimpleCreate(&create_data_event, &data_producer_ao, &create_data_msg, DATA_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    // SchedulerAddTimedEvent(&create_data_event);

    TimedEventSimpleCreate(&hb_event, &heartbeat_ao, &hb_msg, HEARTBEAT_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&hb_event);

    OSCallbacksCfg_t os_callback_cfg = {.on_Idle = NULL, .on_Init = OnKernelInit, .on_SysTick = NULL};

    // initialze kernel
    KernelInit(&os, &os_callback_cfg);

    // enable interrupts again
    ENABLE_INTERRUPTS();

    // run scheduler
    SchedulerRun(&os);

    return 0;
}