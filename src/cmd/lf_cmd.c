#include "lf_cmd.h"

#include "app_defs.h"

#include <os.h>

static void LineFollowCommandStart(Command_t* cmd, void* instance_data);
static bool LineFollowCommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data);
static void LineFollowCommandOnEnd(Command_t* cmd, void* instance_data);

extern void LineFollowCommandInit(LineFollowCommand_t* cmd, uint8_t mode,
                                  uint8_t intersection_count, float base_velocity,
                                  uint32_t delay_ms, Command_t* next)
{
    cmd->base.on_Start = LineFollowCommandStart;
    cmd->base.on_Message = LineFollowCommandOnMessage;
    cmd->base.on_End = LineFollowCommandOnEnd;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;

    cmd->mode = mode;
    cmd->intersection_count = intersection_count;
    cmd->base_velocity = base_velocity;
    cmd->delay_ms = delay_ms;

    cmd->lfmsg.base.id = REFARR_START_LINE_FOLLOW_MSG_ID;
    cmd->lfmsg.base.msg_size = sizeof(LineFollowMessage_t);
    cmd->lfmsg.base_speed = cmd->base_velocity;
    cmd->lfmsg.intersection_count = cmd->intersection_count;
    cmd->lfmsg.mode = cmd->mode;
    cmd->lfmsg.response = &state_ctl_ao;
}

static void LineFollowCommandStart(Command_t* cmd, void* instance_data)
{
    UNUSED(instance_data);

    LineFollowCommand_t* lfcmd = (LineFollowCommand_t*)cmd;

    if (0 == lfcmd->delay_ms)
    {
        MsgQueuePut(&refarr_ss_ao, &lfcmd->lfmsg);
    }
    else
    {
        TimedEventSimpleCreate(&lfcmd->delayed_event, &refarr_ss_ao, &lfcmd->lfmsg, lfcmd->delay_ms,
                               TIMED_EVENT_SINGLE_TYPE);
        SchedulerAddTimedEvent(&lfcmd->delayed_event);
    }
}

static bool LineFollowCommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);

    return (REFARR_INTERSECTION_COUNT_HIT == msg->id);
}

static void LineFollowCommandOnEnd(Command_t* cmd, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);
}
