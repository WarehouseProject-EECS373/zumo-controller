#include "dac_ao.h"

#include <stm32l4xx.h>
#include <stm32l4xx_hal.h>

#include <os.h>

#include "app_defs.h"

static DAC_HandleTypeDef dac_handle;

extern void DAC_Start()
{
    // start both DACs
    HAL_DAC_Start(&dac_handle, DAC_CHANNEL_1);
    HAL_DAC_Start(&dac_handle, DAC_CHANNEL_2);
}

extern void DAC_Init()
{
    dac_handle.Instance = DAC1;
    HAL_DAC_Init(&dac_handle);

    DAC_ChannelConfTypeDef channel_cfg;
    channel_cfg.DAC_Trigger = DAC_TRIGGER_NONE;
    channel_cfg.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    channel_cfg.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    channel_cfg.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    channel_cfg.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    HAL_DAC_ConfigChannel(&dac_handle, &channel_cfg, DAC_CHANNEL_1);
    HAL_DAC_ConfigChannel(&dac_handle, &channel_cfg, DAC_CHANNEL_2);
}

extern void DAC1EventHandler(Message_t *msg)
{
    if (DATA_MSG_ID == msg->id)
    {
        DataMessage_t *in_msg = (DataMessage_t *)msg;
        HAL_DAC_SetValue(&dac_handle, DAC_CHANNEL_1, DAC_ALIGN_12B_R, in_msg->data);
    }
}

extern void DAC2EventHandler(Message_t *msg)
{
    if (DATA_MSG_ID == msg->id)
    {
        DataMessage_t *in_msg = (DataMessage_t *)msg;

        HAL_DAC_SetValue(&dac_handle, DAC_CHANNEL_2, DAC_ALIGN_12B_R, in_msg->data);
    }
}