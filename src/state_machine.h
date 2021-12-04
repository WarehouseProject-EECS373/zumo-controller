#pragma once

#include "app_defs.h"

#include <os.h>

typedef struct StateMachine_s
{
    Command_t *start;
    Command_t *current;
} StateMachine_t;

extern void StateMachineInit(StateMachine_t *sm, Command_t* start);

extern void StateMachineStart(StateMachine_t *sm, void *instance_data);

extern bool StateMachineStep(StateMachine_t *sm, Message_t *msg, void *instance_data);
