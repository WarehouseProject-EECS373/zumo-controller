#pragma once

#include "state_machine.h"

#include "app_defs.h"

#include <os.h>

typedef struct ElectromagnetCommand_s
{
    Command_t base;
    Message_t em_msg;
} ElectromagnetCommand_t;

extern void ElectromagnetCommandInit(ElectromagnetCommand_t* cmd, bool is_on, Command_t* next);
