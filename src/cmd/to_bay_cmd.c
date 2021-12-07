#include "to_bay_cmd.h"

#include "electromagnet_cmd.h"

#define TURN_SPEED_FW  0.55
#define TURN_SPEED_REV -0.6

#define TURN_AROUND_SPEED  0.5
#define REVERSE_SPEED      -0.3
#define REVERSE_DRIVE_TIME 300

static void     ToBayCommandStart(Command_t* cmd, void* instance_data);
static bool     ToBayCommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data);
static void     ToBayCommandOnEnd(Command_t* cmd, void* instance_data);
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

extern void ToBayCommandInit(ToBayCommand_t* cmd, uint32_t bay_id,
                             uint32_t post_dropoff_intersection_count, uint32_t mode,
                             Command_t* next)
{
    cmd->base.on_Start = ToBayCommandStart;
    cmd->base.on_Message = ToBayCommandOnMessage;
    cmd->base.on_End = ToBayCommandOnEnd;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;

    // turn into bay, drive until bay end
    TurnCommandInit(&cmd->turn_in_cmd, GetTurnDirection(bay_id), 1, TURN_TYPE_FROM_TOP,
                    TURN_SPEED_FW, TURN_SPEED_REV, (Command_t*)&cmd->em_cmd);

    bool em_mode;

    if (TO_BAY_DROPOFF == mode)
    {
        em_mode = false;
    }
    else
    {
        em_mode = true;
    }

    ElectromagnetCommandInit(&cmd->em_cmd, em_mode, (Command_t*)&cmd->turn_around_cmd);

    // reverse, turn around, line follow until the aisle
    Turn180CommandInit(&cmd->turn_around_cmd, TURN_DIR_LEFT, TURN_TYPE_180_LEFT, TURN_AROUND_SPEED,
                       REVERSE_SPEED, REVERSE_DRIVE_TIME, (Command_t*)&cmd->turn_out_cmd);

    // turn out of bay and drive
    TurnCommandInit(&cmd->turn_out_cmd, GetTurnDirection(bay_id), post_dropoff_intersection_count,
                    TURN_TYPE_FROM_BASE, TURN_SPEED_FW, TURN_SPEED_REV, NULL);

    StateMachineInit(&cmd->state_machine, (Command_t*)&cmd->turn_in_cmd);
}

static void ToBayCommandStart(Command_t* cmd, void* instance_data)
{
    ToBayCommand_t* bcmd = (ToBayCommand_t*)cmd;
    StateMachineStart(&bcmd->state_machine, instance_data);
}

static bool ToBayCommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data)
{
    ToBayCommand_t* bcmd = (ToBayCommand_t*)cmd;
    return StateMachineStep(&bcmd->state_machine, msg, instance_data);
}

static void ToBayCommandOnEnd(Command_t* cmd, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);
}
