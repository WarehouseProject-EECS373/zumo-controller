#pragma once

#include "app_defs.h"

typedef struct DelayCommand_s
{
    Command_t          base;
    TimedEventSimple_t delay_event;
    Message_t          delay_msg;
    uint32_t           delay;
} DelayCommand_t;

extern void DelayCommandInit(DelayCommand_t* cmd, uint32_t delay, Command_t* next);
