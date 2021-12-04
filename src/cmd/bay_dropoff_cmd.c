#include "bay_dropoff_cmd.h"

#define TURN_SPEED_FW       0.45
#define TURN_SPEED_REV      -0.6

#define TURN_AROUND_SPEED   0.5
#define REVERSE_SPEED       -0.3
#define REVERSE_DRIVE_TIME  300

static void BayDropoffCommandStart(Command_t *cmd, void* instance_data);
static bool BayDropoffCommandOnMessage(Command_t *cmd, Message_t *msg, void* instance_data);
static void BayDropoffCommandOnEnd(Command_t *cmd, void* instance_data);

static uint32_t GetTurnDirection(uint32_t bay_id);



static uint32_t GetTurnDirection(uint32_t bay_id)
{
    if (0 == bay_id % 2)
    {
        return TURN_DIR_LEFT;
    }
    else
    {
        return TURN_DIR_RIGHT;
    }
}


extern void BayDropoffCommandInit(BayDropoffCommand_t *cmd, uint32_t bay_id, uint32_t remaining_bays, Command_t *next)
{
    cmd->base.on_Start = BayDropoffCommandStart;
    cmd->base.on_Message = BayDropoffCommandOnMessage;
    cmd->base.on_End = BayDropoffCommandOnEnd;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;



    TurnCommandInit(&cmd->turn_in_cmd, GetTurnDirection(bay_id), 1, TURN_TYPE_FROM_TOP, TURN_SPEED_FW, TURN_SPEED_REV, (Command_t*)&cmd->turn_around_cmd);
    Turn180CommandInit(&cmd->turn_around_cmd, TURN_DIR_LEFT, TURN_TYPE_180_LEFT, TURN_AROUND_SPEED, REVERSE_SPEED, REVERSE_DRIVE_TIME, (Command_t*)&cmd->turn_out_cmd);
   TurnCommandInit(&cmd->turn_out_cmd, GetTurnDirection(bay_id), remaining_bays, TURN_TYPE_FROM_BASE, TURN_SPEED_FW, TURN_SPEED_REV, NULL);

    StateMachineInit(&cmd->state_machine, (Command_t*)&cmd->turn_in_cmd);
}


static void BayDropoffCommandStart(Command_t *cmd, void* instance_data)
{
    BayDropoffCommand_t *bcmd = (BayDropoffCommand_t*)cmd;
    StateMachineStart(&bcmd->state_machine, instance_data);
}

static bool BayDropoffCommandOnMessage(Command_t *cmd, Message_t *msg, void* instance_data)
{
    BayDropoffCommand_t *bcmd = (BayDropoffCommand_t*)cmd;
    return StateMachineStep(&bcmd->state_machine, msg, instance_data);

}

static void BayDropoffCommandOnEnd(Command_t *cmd, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);
}

