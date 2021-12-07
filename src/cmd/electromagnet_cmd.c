#include "electromagnet_cmd.h"

#include "app_defs.h"

#include <os.h>

static void ElectromagnetCommandStart(Command_t* cmd, void* instance_data);

extern void ElectromagnetCommandInit(ElectromagnetCommand_t* cmd, bool is_on, Command_t* next)
{
    cmd->base.on_Start = ElectromagnetCommandStart;
    cmd->base.on_Message = NULL;
    cmd->base.on_End = NULL;
    cmd->base.end_behavior = COMMAND_ON_END_INSTANT;
    cmd->base.next = next;

    Message_t msg;
    msg.msg_size = sizeof(Message_t);

    if (is_on)
    {
        msg.id = ELECTROMAGNET_ENABLE;
    }
    else
    {
        msg.id = ELECTROMAGNET_DISABLE;
    }

    cmd->em_msg = msg;
}

static void ElectromagnetCommandStart(Command_t* cmd, void* instance_data)
{
    UNUSED(instance_data);

    MsgQueuePut(&electromagnet_ss_ao, &((ElectromagnetCommand_t*)cmd)->em_msg);
}
