#include "reflectance_array_subsystem.h"

#include <os.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32f4xx.h"

extern void REFARR_Init()
{
}

extern void ReflectanceArrayEventHandler(Message_t* msg)
{
    if(msg->id == REFARR_CALIBRATE_MSG_ID)
    {
    }
    else if(msg->id == REFARR_ON_MSG_ID)
    {
    }
    else if(msg->id == REFARR_OFF_MSG_ID)
    {
    }
    else if(msg->id == REFARR_START_READ_MSG_ID)
    {
    }
    else if(msg->id == REFARR_STOP_READ_MSG_ID)
    {
    }
}
