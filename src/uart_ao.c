#include "uart_ao.h"

#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os_mem.h>
#include <os.h>

#include "app_defs.h"

static UART_HandleTypeDef hlpuart1;

extern void UART_Init()
{
    // VddIO2 must be enabled for GPIO G bank

    HAL_PWREx_EnableVddIO2();

    GPIO_InitTypeDef gpio_cfg;

    gpio_cfg.Pin = GPIO_PIN_7 | GPIO_PIN_8;
    gpio_cfg.Mode = GPIO_MODE_AF_PP;
    gpio_cfg.Pull = GPIO_PULLUP;
    gpio_cfg.Speed = GPIO_SPEED_HIGH;
    gpio_cfg.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOG, &gpio_cfg);

    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;

    HAL_StatusTypeDef status = HAL_UART_Init(&hlpuart1);
}

extern void UARTEventHandler(Message_t *msg)
{
    if (LOG_MSG_ID == msg->id)
    {
        MemoryBlockMessage_t *in_msg = (MemoryBlockMessage_t *)msg;
        uint8_t *data = OSMemoryBlockGet(in_msg->key);
        HAL_StatusTypeDef status = HAL_UART_Transmit(&hlpuart1, (uint8_t *)data, in_msg->size, 0xffff);
        OSMemoryFreeBlock(in_msg->key);
    }
}