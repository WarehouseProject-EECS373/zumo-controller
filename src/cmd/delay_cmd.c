#include "delay_cmd.h"

#include "app_defs.h"

extern void DelayCommandInit(DelayCommand_t *cmd, uint32_t delay, Command_t *next)
{
    cmd->base.on_Start = DelayCommandStart;
    cmd->base.on_Message = DelayCommandOnMessage;
    cmd->base.on_End = NULL;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;

    cmd->delay = delay;

    Message_t msg = {.id = TIMED_EVENT_DONE_MSG_ID, sizeof(Message_t)};
    cmd->delay_msg = msg; 
}

extern void DelayCommandStart(Command_t *cmd, void* instance_data)
{
    UNUSED(instance_data);

    DelayCommand_t *dcmd = (DelayCommand_t*)cmd;
    TimedEventSimpleCreate(&dcmd->delay_event, &state_ctl_ao, &dcmd->delay_msg, dcmd->delay, TIMED_EVENT_SINGLE_TYPE);
    SchedulerAddTimedEvent(&dcmd->delay_event);
}

extern bool DelayCommandOnMessage(Command_t *cmd, Message_t *msg, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);

    return TIMED_EVENT_DONE_MSG_ID == msg->id;
}
