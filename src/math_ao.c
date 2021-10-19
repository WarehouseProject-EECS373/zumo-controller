#include "math_ao.h"

#include <stdint.h>
#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>
#include <os_util.h>

#include "app_defs.h"

#define EXP_MOVING_AVERAGE_ALPHA 0.25

// sine table
static const uint8_t SINEWAVE[] = {
    126, 136, 146, 156, 166, 175, 185, 194, 202, 210,
    217, 224, 230, 235, 240, 244, 247, 249, 251, 251,
    251, 250, 248, 246, 242, 238, 233, 227, 221, 214,
    206, 198, 189, 180, 171, 161, 151, 141, 131, 121,
    111, 101, 91, 81, 72, 63, 54, 46, 38, 31,
    25, 19, 14, 10, 6, 4, 2, 1, 1, 1,
    3, 5, 8, 12, 17, 22, 28, 35, 42, 50,
    58, 67, 77, 86, 96, 106, 116};

// data
static const int8_t NOISE[] = {90, 90, 80, 20, 10, 10, 40, 40, 50, 80, 40, 20, 20};

extern void DataProducerEventHandler(Message_t *msg)
{
    static bool enabled = false;
    static uint32_t data = 0;

    if (PUSH_BUTTON_PRESSED_MSG_ID == msg->id)
    {
        enabled = !enabled;
        data = 0;
    }
    else if (enabled && CREATE_DATA_MSG_ID == msg->id)
    {
        uint32_t t = OSGetTime();

        data = 6 * SINEWAVE[(t / 10) % 77] + 3 * NOISE[t % 13];

        DataMessage_t data_msg = {.data = data, .timestamp = t};
        data_msg.base.id = DATA_MSG_ID;
        data_msg.base.msg_size = sizeof(DataMessage_t);
        MsgQueuePut(&dac1_ao, (void *)&data_msg);
        MsgQueuePut(&averager_ao, (void *)&data_msg);

        uint16_t key;
        OSStatus_t *status = NULL;
        uint8_t *block = OSMemoryBlockNew(&key, MEMORY_BLOCK_32, status);
        os_memcpy(block, &data, sizeof(uint32_t));
        MemoryBlockMessage_t mem_msg = {.key = key, .size = sizeof(uint32_t)};
        mem_msg.base.id = LOG_MSG_ID;
        mem_msg.base.msg_size = sizeof(MemoryBlockMessage_t);

        MsgQueuePut(&uart_ao, (void *)&mem_msg);
    }
}

extern void AveragerEventHandler(Message_t *msg)
{
    static uint32_t average = 0;

    if (PUSH_BUTTON_PRESSED_MSG_ID == msg->id)
    {
        average = 0;
    }
    else if (DATA_MSG_ID == msg->id)
    {
        DataMessage_t *in_msg = (DataMessage_t *)msg;

        average = EXP_MOVING_AVERAGE_ALPHA * in_msg->data + (1 - EXP_MOVING_AVERAGE_ALPHA) * average;

        DataMessage_t data_msg = {.data = average};
        data_msg.base.id = DATA_MSG_ID;
        data_msg.base.msg_size = sizeof(DataMessage_t);
        MsgQueuePut(&dac2_ao, (void *)&data_msg);
    }
}

void IntegratorEventHandler(Message_t *msg)
{
    static uint32_t current = 0;
    static uint32_t last_time = 0;

    if (PUSH_BUTTON_PRESSED_MSG_ID == msg->id)
    {
        current = 0;
        last_time = OSGetTime();
    }
    else if (DATA_MSG_ID == msg->id)
    {
        DataMessage_t *in_msg = (DataMessage_t *)msg;

        current += in_msg->data * (in_msg->timestamp - last_time) / 1000;
        last_time = in_msg->timestamp;

        DataMessage_t data_msg = {.data = current};
        data_msg.base.id = DATA_MSG_ID;
        data_msg.base.msg_size = sizeof(DataMessage_t);
        MsgQueuePut(&dac2_ao, (void *)&data_msg);
    }
}

void DifferentiatorEventHandler(Message_t *msg)
{
    static uint32_t last_time = 0;
    static uint32_t last_data = 0;
    static uint32_t derivative = 0;

    if (PUSH_BUTTON_PRESSED_MSG_ID == msg->id)
    {
        last_time = OSGetTime();
        last_data = 0;
        derivative = 0;
    }
    else if (DATA_MSG_ID == msg->id)
    {
        DataMessage_t *in_msg = (DataMessage_t *)msg;
        derivative = (in_msg->data - last_data) / (in_msg->timestamp - last_time);

        DataMessage_t data_msg = {.data = derivative};
        data_msg.base.id = DATA_MSG_ID;
        data_msg.base.msg_size = sizeof(DataMessage_t);
        MsgQueuePut(&dac2_ao, (void *)&data_msg);

        last_time = in_msg->timestamp;
        last_data = in_msg->data;
    }
}