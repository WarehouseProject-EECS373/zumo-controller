#pragma once

#include <os.h>

#include "app_defs.h"

typedef struct DriveOpenLoopCommand_s
{
    Command_t base;
    float     left_out;
    float     right_out;
} DriveOpenLoopCommand_t;

extern void DriveOpenLoopCommandInit(DriveOpenLoopCommand_t* cmd, float left_out, float right_out,
                                     Command_t* next);
