#pragma once

#include "app_defs.h"

#include <os.h>

typedef struct ZoneLogCommand_s
{
    Command_t base;
    UartSmallPacketMessage_t msg;
} ZoneLogCommand_t;

extern void ZoneLogCommandInit(ZoneLogCommand_t *cmd, uint8_t aisle, Command_t *next);
