#pragma once

#include "app_defs.h"

#include <os.h>

typedef struct LineFollowCommand_s
{
    Command_t           base;
    TimedEventSimple_t  delayed_event;
    LineFollowMessage_t lfmsg;
    float               base_velocity;
    uint32_t            delay_ms;
    uint8_t             mode;
    uint8_t             intersection_count;
} LineFollowCommand_t;

extern void LineFollowCommandInit(LineFollowCommand_t* cmd, uint8_t mode,
                                  uint8_t intersection_count, float base_velocity,
                                  uint32_t delay_ms, Command_t* next);
