#include "turn180_cmd.h"

#include <os.h>

#include "state_machine.h"
#include "drive_open_loop_cmd.h"

static void Turn180CommandStart(Command_t* cmd, void* instance_data);
static bool Turn180CommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data);
static void Turn180CommandOnEnd(Command_t* cmd, void* instance_data);

extern void Turn180CommandInit(Turn180Command_t* cmd, uint32_t turn_direction, uint32_t turn_type,
                               float turn_speed, float rev_speed, uint32_t reverse_drive_time,
                               Command_t* next)
{
    cmd->base.on_Start = Turn180CommandStart;
    cmd->base.on_Message = Turn180CommandOnMessage;
    cmd->base.on_End = Turn180CommandOnEnd;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;

    DriveOpenLoopCommandInit(&cmd->rev_drive_cmd, rev_speed, rev_speed,
                             (Command_t*)&cmd->rev_drive_delay_cmd);
    DelayCommandInit(&cmd->rev_drive_delay_cmd, reverse_drive_time, (Command_t*)&cmd->turn180_cmd);
    TurnCommandInit(&cmd->turn180_cmd, turn_direction, 1, turn_type, turn_speed, -1 * turn_speed, NULL);

    StateMachineInit(&cmd->state_machine, (Command_t*)&cmd->rev_drive_cmd);
}

static void Turn180CommandStart(Command_t* cmd, void* instance_data)
{
    Turn180Command_t* tcmd = (Turn180Command_t*)cmd;
    StateMachineStart(&tcmd->state_machine, instance_data);
}

static bool Turn180CommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data)
{
    Turn180Command_t* tcmd = (Turn180Command_t*)cmd;
    return StateMachineStep(&tcmd->state_machine, msg, instance_data);
}

static void Turn180CommandOnEnd(Command_t* cmd, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);
}
