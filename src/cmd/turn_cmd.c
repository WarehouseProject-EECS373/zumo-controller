#include "turn_cmd.h"

#include "lf_cmd.h"
#include "drive_open_loop_cmd.h"
#include "state_machine.h"

#include "app_defs.h"

#include <os.h>

#define DRIVE_DIR_FORWARD 1
#define DRIVE_DIR_REVERSE -1

#define OPEN_LOOP_TURN_DELAY_MS 10

typedef struct TurnTypeDirectionCfg_s
{
    uint8_t direction;
    uint8_t mode;
} TurnTypeDirectionCfg_t;

static TurnTypeDirectionCfg_t turn_type_dir_cfg[2][2] = {
    {{TURN_DIR_LEFT, REFARR_LEFT_SENSOR_ENABLE}, {TURN_DIR_LEFT, REFARR_RIGHT_SENSOR_ENABLE}},
    {{TURN_DIR_RIGHT, REFARR_RIGHT_SENSOR_ENABLE}, {TURN_DIR_RIGHT, REFARR_LEFT_SENSOR_ENABLE}}};

static void TurnCommandStart(Command_t* cmd, void* instance_data);
static bool TurnCommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data);
static void TurnCommandOnEnd(Command_t* cmd, void* instance_data);

static void ConfigureOpenLoopSpeeds(TurnTypeDirectionCfg_t* cfg, float* left_out, float* right_out,
                                    float fw_speed, float rev_speed);

static void ConfigureOpenLoopSpeeds(TurnTypeDirectionCfg_t* cfg, float* left_out, float* right_out,
                                    float fw_speed, float rev_speed)
{
    if (TURN_DIR_LEFT == cfg->direction)
    {
        *left_out = rev_speed;
        *right_out = fw_speed;
    }
    else
    {
        *left_out = fw_speed;
        *right_out = rev_speed;
    }
}

extern void TurnCommandInit(TurnCommand_t* cmd, uint32_t turn_direction,
                            uint32_t post_turn_intersection_count, uint32_t turn_type,
                            float fw_speed, float rev_speed, Command_t* next)
{
    cmd->base.on_Start = TurnCommandStart;
    cmd->base.on_Message = TurnCommandOnMessage;
    cmd->base.on_End = TurnCommandOnEnd;
    cmd->base.end_behavior = COMMAND_ON_END_WAIT_FOR_END;
    cmd->base.next = next;

    cmd->turn_direction = turn_direction;
    cmd->turn_type = turn_type;

    TurnTypeDirectionCfg_t turn_cfg = turn_type_dir_cfg[turn_direction][turn_type];

    float left_out;
    float right_out;

    ConfigureOpenLoopSpeeds(&turn_cfg, &left_out, &right_out, fw_speed, rev_speed);

    // 1. start open loop drive right away and go straight into line following for turning
    // 2. when line following for turning is done, go to post turn line follow
    DriveOpenLoopCommandInit(&cmd->ol_drive_cmd, left_out, right_out, (Command_t*)&cmd->lf_cmd);
    LineFollowCommandInit(&cmd->lf_cmd, turn_cfg.mode, 1, 0.0, 0,
                          (Command_t*)&cmd->post_turn_lf_cmd);

    uint8_t post_turn_lf_mode =
        REFARR_DRIVE_CTL_ENABLE | REFARR_LEFT_SENSOR_ENABLE | REFARR_RIGHT_SENSOR_ENABLE;
    LineFollowCommandInit(&cmd->post_turn_lf_cmd, post_turn_lf_mode, post_turn_intersection_count,
                          LINE_FOLLOW_MAX_BASE_VELOCITY, OPEN_LOOP_TURN_DELAY_MS, NULL);

    StateMachineInit(&cmd->state_machine, (Command_t*)&cmd->ol_drive_cmd);
}

static void TurnCommandStart(Command_t* cmd, void* instance_data)
{
    TurnCommand_t* tcmd = (TurnCommand_t*)cmd;
    StateMachineStart(&tcmd->state_machine, instance_data);
}

static bool TurnCommandOnMessage(Command_t* cmd, Message_t* msg, void* instance_data)
{
    TurnCommand_t* tcmd = (TurnCommand_t*)cmd;
    return StateMachineStep(&tcmd->state_machine, msg, instance_data);
}

static void TurnCommandOnEnd(Command_t* cmd, void* instance_data)
{
    UNUSED(cmd);
    UNUSED(instance_data);
}
