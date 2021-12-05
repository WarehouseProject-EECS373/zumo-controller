#pragma once

#include "app_defs.h"

#include "state_machine.h"

#include "lf_cmd.h"
#include "drive_open_loop_cmd.h"

#include <os.h>

typedef struct TurnCommand_s
{
    Command_t              base;
    StateMachine_t         state_machine;
    DriveOpenLoopCommand_t ol_drive_cmd;
    LineFollowCommand_t    lf_cmd;
    LineFollowCommand_t    post_turn_lf_cmd;
    uint32_t               turn_direction;
    uint32_t               turn_type;
} TurnCommand_t;

extern void TurnCommandInit(TurnCommand_t* cmd, uint32_t turn_direction,
                            uint32_t post_turn_intersection_count, uint32_t turn_type,
                            float fw_speed, float rev_speed, Command_t* next);
