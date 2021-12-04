#pragma once

#include "app_defs.h"

#include "state_machine.h"

#include "lf_cmd.h"
#include "drive_open_loop_cmd.h"

#include <os.h>

typedef struct Turn90Command_s
{
    Command_t base;
    StateMachine_t state_machine;
    DriveOpenLoopCommand_t ol_drive_cmd;
    LineFollowCommand_t lf_cmd;
    LineFollowCommand_t post_turn_lf_cmd;
    uint32_t turn_direction;
    uint32_t turn_type;
} Turn90Command_t;

extern void Turn90CommandInit(Turn90Command_t *cmd, uint32_t turn_direction, uint32_t post_turn_intersection_count, uint32_t turn_type, float fw_speed, float rev_speed, Command_t* next);

extern void Turn90CommandStart(Command_t *cmd, void* instance_data);

extern bool Turn90CommandOnMessage(Command_t *cmd, Message_t *msg, void* instance_data);

extern void Turn90CommandOnEnd(Command_t *cmd, void* instance_data);
