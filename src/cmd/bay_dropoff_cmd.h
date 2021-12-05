
#pragma once

#include "app_defs.h"

#include "turn_cmd.h"
#include "turn180_cmd.h"

#include "state_machine.h"

typedef struct BayDropoffCommand_s
{
    Command_t        base;
    StateMachine_t   state_machine;
    TurnCommand_t    turn_in_cmd;
    Turn180Command_t turn_around_cmd;
    TurnCommand_t    turn_out_cmd;
} BayDropoffCommand_t;

extern void BayDropoffCommandInit(BayDropoffCommand_t* cmd, uint32_t bay_id,
                                  uint32_t post_dropoff_intersection_count, Command_t* next);
