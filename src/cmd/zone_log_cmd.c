#include "zone_log_cmd.h"

#include "app_defs.h"

#include <os.h>

static void ZoneLogCommandStart(Command_t* cmd, void* instance_data);

extern void ZoneLogCommandInit(ZoneLogCommand_t* cmd, uint8_t aisle, Command_t* next)
{
    cmd->base.on_Start = ZoneLogCommandStart;
    cmd->base.on_Message = NULL;
    cmd->base.on_End = NULL;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;

    cmd->msg.base.id = UART_SMALL_PACKET_MSG_ID;
    cmd->msg.base.msg_size = sizeof(ZoneLogCommand_t);
    cmd->msg.length = 1;
    cmd->msg.payload[0] = MSG_AISLE_FREE;
    cmd->msg.payload[1] = aisle;
}

static void ZoneLogCommandStart(Command_t* cmd, void* instance_data)
{
    UNUSED(instance_data);
    MsgQueuePut(&comms_ss_ao, (UartSmallPacketMessage_t*)cmd);
}
