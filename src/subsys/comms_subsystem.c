#include "comms_subsystem.h"

#include <stm/stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"

#include <os.h>
#include <os_mem.h>

#define UART_TX_TIMEOUT 10

UART_HandleTypeDef uart_handle;
DMA_HandleTypeDef dma_uart_tx_handle;

static void ProcessSmallMessage(UartSmallPacketMessage_t* msg);
static void ProcessLargeMessage(UartLargePacketMessage_t* msg);
static void SendMessage(void* buffer, uint16_t length);

__attribute__((__interrupt__)) extern void DMA1_Stream6_IRQHandler()
{
    OS_ISR_ENTER();
    __HAL_DMA_CLEAR_FLAG(&dma_uart_tx_handle, __HAL_DMA_GET_TC_FLAG_INDEX(&dma_uart_tx_handle));
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

    dma_uart_tx_handle.Instance = DMA2_Stream6;
    dma_uart_tx_handle.Init.Channel = DMA_CHANNEL_5;
    dma_uart_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_uart_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_uart_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
    dma_uart_tx_handle.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_uart_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_uart_tx_handle.Init.Mode = DMA_NORMAL;
    dma_uart_tx_handle.Init.Priority = DMA_PRIORITY_LOW;
    dma_uart_tx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&dma_uart_tx_handle);

    __HAL_LINKDMA(&uart_handle, hdmatx, dma_uart_tx_handle);

    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);


    uart_handle.Instance = USART6;
    uart_handle.Init.BaudRate = 115200;
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.Parity = UART_PARITY_NONE;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    HAL_UART_Init(&uart_handle);
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
    HAL_UART_Transmit_DMA(&uart_handle, (uint8_t*)buffer, length);
}
