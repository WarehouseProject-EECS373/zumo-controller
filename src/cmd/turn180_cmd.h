#pragma once

#include "app_defs.h"

#include <os.h>

#include "state_machine.h"

#include "turn_cmd.h"
#include "drive_open_loop_cmd.h"
#include "delay_cmd.h"

typedef struct Turn180Command_s
{
    Command_t base;
    StateMachine_t state_machine;
    DriveOpenLoopCommand_t rev_drive_cmd;
    DelayCommand_t rev_drive_delay_cmd;
    TurnCommand_t turn180_cmd;
} Turn180Command_t;


extern void Turn180CommandInit(Turn180Command_t *cmd, uint32_t turn_direction, uint32_t turn_type, float turn_speed, float rev_speed, uint32_t reverse_drive_time, Command_t *next);

extern void Turn180CommandStart(Command_t *cmd, void* instance_data);

extern bool Turn180CommandOnMessage(Command_t *cmd, Message_t *msg, void* instance_data);

extern void Turn180CommandOnEnd(Command_t *cmd, void* instance_data);

