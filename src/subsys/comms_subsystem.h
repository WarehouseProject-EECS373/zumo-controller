#ifndef COMMS_SUBSYS_H
#define COMMS_SUBSYS_H

#include <os.h>

#include "app_defs.h"

extern void Comms_Init();

extern void CommsEventHandler(Message_t *msg);

#endif
