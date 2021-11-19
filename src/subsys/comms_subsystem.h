#ifndef COMMS_SUBSYS_H
#define COMMS_SUBSYS_H

#include <os.h>

#include "app_defs.h"


__attribute__((__interrupt__)) extern void DMA1_Stream7_IRQHandler();

extern void Comms_Init();

extern void CommsEventHandler(Message_t *msg);

#endif
