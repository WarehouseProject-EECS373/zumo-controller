#include "comms_subsystem.h"

#include <stdint.h>

#include <stm/stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"

#include <os.h>
#include <os_mem.h>

#include <stcp.h>

#define UART_TX_TIMEOUT 10

#define UART_RX_BUFFER_SIZE 32

// byte indexes in buffer
#define MESSAGE_ID_IDX      0
#define AISLE_ID_IDX        1
#define BAY_ID_IDX          2

// incoming message from warehouse for dispatch
#define MSG_DISPATCH_ID     0x1

#define MSG_LINE_FOLLOWING_ID 0xD

#define EXPECTED_LINE_FOLLOW_LENGTH 2
#define EXPECTED_DISPATCH_LENGTH    3
#define EXPECTED_GET_P_LENGTH       3
#define EXPECTED_SET_P_LENGTH       7


STCPEngine_t stcp_engine;

// NOTE: if we need a unique ID for each zumo
// have the warehouse controller assign IDs in
// the very beginning by sending some message
// instead of hardcoding it in source.

UART_HandleTypeDef uart_handle;

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint32_t rx_buffer_count = 0;

static void ProcessSmallMessage(UartSmallPacketMessage_t* msg);
static void ProcessLargeMessage(UartLargePacketMessage_t* msg);
static STCPStatus_t SendMessage(void* buffer, uint16_t length, void* instance_data);
static STCPStatus_t UnpackMessage(void* buffer, uint16_t length, void* instance_data);

__attribute__((__interrupt__)) extern void USART6_IRQHandler()
{
    OS_ISR_ENTER();

    // if RX buffer Not Empty (RXNE)
    if(__HAL_UART_GET_IT_SOURCE(&uart_handle, UART_IT_RXNE) != RESET)
    {
        // add byte in UART data register to rx buffer
        rx_buffer[rx_buffer_count++] = (uint8_t)(USART6->DR & 0xFF);

        // all incoming packets are terminated by 2 FOOTER characters
        if(UART_RX_BUFFER_SIZE <= rx_buffer_count || (rx_buffer[rx_buffer_count
         - 1] == FOOTER && rx_buffer[rx_buffer_count - 2] == FOOTER))
        {
            // don't fill or modify buffer while unpacking it
            DISABLE_INTERRUPTS();
            StcpHandleMessage(&stcp_engine, (uint8_t*)rx_buffer, rx_buffer_count);
            ENABLE_INTERRUPTS();

            rx_buffer_count = 0;
        }
    }

    HAL_NVIC_ClearPendingIRQ(USART6_IRQn);

    OS_ISR_EXIT();
}

static STCPStatus_t UnpackMessage(void* buffer, uint16_t length, void* instance_data)
{
    uint8_t *payload = (uint8_t*)buffer;
    UNUSED(instance_data);
    if (MSG_DISPATCH_ID == payload[MESSAGE_ID_IDX])
    {
        if (EXPECTED_DISPATCH_LENGTH != length)
        {
            return STCP_STATUS_UNDEFINED_ERROR;
        }

        // create dispatch message, send to state controller
        DispatchMessage_t dmsg;
        dmsg.base.id = SM_DISPATCH_FROM_IDLE_MSG_ID;
        dmsg.base.msg_size = sizeof(DispatchMessage_t);
        dmsg.aisle_id = payload[AISLE_ID_IDX];
        dmsg.bay_id = payload[BAY_ID_IDX];

        MsgQueuePut(&state_ctl_ao, &dmsg);
    }
    else if (MSG_P_GET_ID == payload[MESSAGE_ID_IDX])
    {
        if (EXPECTED_GET_P_LENGTH != length)
        {
            return STCP_STATUS_UNDEFINED_ERROR;
        }
        PropertyGetSetMessage_t msg;
        msg.base.id = GET_PROPERTY_MSG_ID;
        msg.base.msg_size = sizeof(PropertyGetSetMessage_t);
        msg.p_id = *((uint16_t *)(payload + 1));

        if (msg.p_id <= DRIVE_PROPERTY_MAX_ID)
        {
            MsgQueuePut(&drive_ss_ao, &msg);
        }
    }
    else if (MSG_P_SET_ID == payload[MESSAGE_ID_IDX])
    {
        if (EXPECTED_SET_P_LENGTH != length)
        {
            return STCP_STATUS_UNDEFINED_ERROR;
        }

        PropertyGetSetMessage_t msg;
        msg.base.id = SET_PROPERTY_MSG_ID;
        msg.base.msg_size = sizeof(PropertyGetSetMessage_t);
        msg.p_id = *((uint16_t *)(payload + 1));
        
        os_memcpy(msg.value, (void*)(payload + 3), 4);

        if (msg.p_id <= DRIVE_PROPERTY_MAX_ID)
        {
            MsgQueuePut(&drive_ss_ao, &msg);
        }
    }
    else if (MSG_LINE_FOLLOWING_ID == payload[MESSAGE_ID_IDX])
    {
        if (EXPECTED_LINE_FOLLOW_LENGTH != length)
        {
            return STCP_STATUS_UNDEFINED_ERROR;
        }

        LineFollowMessage_t lf_msg;
        lf_msg.base.id = REFARR_START_LINE_FOLLOW_MSG_ID;
        lf_msg.base.msg_size = sizeof(LineFollowMessage_t);
        lf_msg.base_speed = 0.5;
        lf_msg.intersection_count = payload[1];
        lf_msg.response = &state_ctl_ao;

        MsgQueuePut(&refarr_ss_ao, &lf_msg);
    }

    return STCP_STATUS_SUCCESS;
}


extern void Comms_Init()
{
    STCPCallbacks_t callbacks = {.Send = SendMessage, .HandleMessage = UnpackMessage};
    stcp_engine.callbacks = callbacks;
    stcp_engine.instance_data = NULL;

    // make compiler happy, used as callbacks
    UNUSED(SendMessage);
    UNUSED(UnpackMessage);

    GPIO_InitTypeDef gpio_cfg;

    gpio_cfg.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio_cfg.Mode = GPIO_MODE_AF_PP;
    gpio_cfg.Pull = GPIO_PULLUP;
    gpio_cfg.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_cfg.Alternate = GPIO_AF8_USART6;

    HAL_GPIO_Init(GPIOA, &gpio_cfg);

    uart_handle.Instance = USART6;
    uart_handle.Init.BaudRate = 115200;
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.Parity = UART_PARITY_NONE;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    HAL_UART_Init(&uart_handle);

    HAL_NVIC_SetPriority(USART6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_RXNE);
}

extern void CommsEventHandler(Message_t* msg)
{
    if(UART_SMALL_PACKET_MSG_ID == msg->id || OS_DEBUG_MSG_ID == msg->id || DRIVE_CTL_TRACE_INIT_MSG_ID == msg->id)
    {
        ProcessSmallMessage((UartSmallPacketMessage_t*)msg);
    }
    else if(UART_LARGE_PACKET_MSG_ID == msg->id || LINE_FOLLOW_TRACE_MSG_ID == msg->id || DRIVE_CTL_TRACE_MSG_ID == msg->id)
    {
        ProcessLargeMessage((UartLargePacketMessage_t*)msg);
    }
}

static void ProcessSmallMessage(UartSmallPacketMessage_t* msg)
{
    StcpWrite(&stcp_engine, msg->payload, msg->length);
}

static void ProcessLargeMessage(UartLargePacketMessage_t* msg)
{
    uint8_t* buffer = OSMemoryBlockGet(msg->mem_key);
    StcpWrite(&stcp_engine, buffer, msg->length);
    OSMemoryFreeBlock(msg->mem_key);
}

static STCPStatus_t SendMessage(void* buffer, uint16_t length, void* instance_data)
{
    UNUSED(instance_data);
    HAL_UART_Transmit(&uart_handle, (uint8_t*)buffer, length, UART_TX_TIMEOUT);

    return STCP_STATUS_SUCCESS;
}

