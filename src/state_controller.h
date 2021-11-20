#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

#include <os.h>

#include "app_defs.h"

extern void StateController_Init();

extern void StateControllerEventHandler(Message_t *msg);


#endif

