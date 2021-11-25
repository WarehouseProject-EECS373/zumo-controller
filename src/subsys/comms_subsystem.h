#ifndef COMMS_SUBSYS_H
#define COMMS_SUBSYS_H

#include <os.h>
#include <stcp.h>

#include "app_defs.h"

extern void Comms_Init();

extern void CommsEventHandler(Message_t *msg);

#ifdef OS_TRACE_ENABLED
extern void DebugPrint(uint8_t ao_id, uint32_t msg_id, uint8_t is_queue);
#endif

#endif
