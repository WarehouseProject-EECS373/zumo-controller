
#pragma once

#include "app_defs.h"

#include "turn_cmd.h"
#include "turn180_cmd.h"
#include "electromagnet_cmd.h"
#include "zone_log_cmd.h"

#include "state_machine.h"

typedef struct ToBayCommand_s
{
    Command_t              base;
    StateMachine_t         state_machine;
    TurnCommand_t          turn_in_cmd;
    ElectromagnetCommand_t em_cmd;
    Turn180Command_t       turn_around_cmd;
    ZoneLogCommand_t       zone_log_cmd;
    TurnCommand_t          turn_out_cmd;
    uint32_t               mode;
} ToBayCommand_t;

extern void ToBayCommandInit(ToBayCommand_t* cmd, uint32_t bay_id,
                             uint32_t post_dropoff_intersection_count, uint32_t mode,
                             Command_t* next);
