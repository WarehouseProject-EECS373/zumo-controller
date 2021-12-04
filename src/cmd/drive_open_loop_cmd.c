#include "drive_open_loop_cmd.h"

#include "app_defs.h"

#include <os.h>

static void DriveOpenLoopCommandStart(Command_t *cmd, void* instance_data);

extern void DriveOpenLoopCommandInit(DriveOpenLoopCommand_t *cmd, float left_out, float right_out, Command_t* next)
{
    cmd->base.on_Start = DriveOpenLoopCommandStart;
    cmd->base.on_Message = NULL;
    cmd->base.on_End = NULL;
    cmd->base.end_behavior = COMMAND_ON_END_INSTANT;
    cmd->base.next = next;

    cmd->left_out = left_out;
    cmd->right_out = right_out;
}

static void DriveOpenLoopCommandStart(Command_t *cmd, void* instance_data)
{
    UNUSED(instance_data);

    DriveOpenLoopControlMessage_t olmsg;
    olmsg.base.id = DRIVE_OPEN_LOOP_MSG_ID;
    olmsg.base.msg_size = sizeof(DriveOpenLoopControlMessage_t);

    DriveOpenLoopCommand_t *dcmd = (DriveOpenLoopCommand_t*)cmd;

    olmsg.percent_left = dcmd->left_out;
    olmsg.percent_right = dcmd->right_out;

    MsgQueuePut(&drive_ss_ao, &olmsg);
}


