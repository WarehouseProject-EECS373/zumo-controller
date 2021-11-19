#include "comms_subsystem.h"

#include <stm/stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"

#include <os.h>
#include <os_mem.h>

#define UART_TX_TIMEOUT 10

#define UART_RX_BUFFER_SIZE 8

UART_HandleTypeDef uart_handle;

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint32_t rx_buffer_count = 0;


static void ProcessSmallMessage(UartSmallPacketMessage_t* msg);
static void ProcessLargeMessage(UartLargePacketMessage_t* msg);
static void SendMessage(void* buffer, uint16_t length);

__attribute__((__interrupt__)) extern void USART6_IRQHandler()
{
    OS_ISR_ENTER();
    
    if(__HAL_UART_GET_IT_SOURCE(&uart_handle, UART_IT_RXNE) != RESET)
    {
        rx_buffer[rx_buffer_count++] = (uint8_t)(USART6->DR & 0xFF);
        if (UART_RX_BUFFER_SIZE == rx_buffer_count)
        {
            rx_buffer_count = 0;
        }
    }

    HAL_NVIC_ClearPendingIRQ(USART6_IRQn);

    __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_RXNE);

    OS_ISR_EXIT();
}

extern void Comms_Init()
{
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
    if(UART_SMALL_PACKET_MSG_ID == msg->id)
    {
        ProcessSmallMessage((UartSmallPacketMessage_t*)msg);
    }
    else if(UART_LARGE_PACKET_MSG_ID == msg->id)
    {
        ProcessLargeMessage((UartLargePacketMessage_t*)msg);
    }
}

static void ProcessSmallMessage(UartSmallPacketMessage_t* msg)
{
    SendMessage(msg->payload, msg->length);
}

static void ProcessLargeMessage(UartLargePacketMessage_t* msg)
{
    uint8_t* buffer = OSMemoryBlockGet(msg->mem_key);
    SendMessage(buffer, msg->length);
    OSMemoryFreeBlock(msg->mem_key);
}

static void SendMessage(void* buffer, uint16_t length)
{
    HAL_UART_Transmit(&uart_handle, (uint8_t*)buffer, length, UART_TX_TIMEOUT);
}
