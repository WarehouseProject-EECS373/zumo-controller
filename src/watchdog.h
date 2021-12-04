#pragma once

#include <os.h>

/**
 * @brief Initialize watchdog
 *
 */
extern void Watchdog_Init();

/**
 * @brief Event handler for watchdog. Simple LED heartbeat so we know everything is ok
 *
 * @param msg
 */
extern void WatchdogEventHandler(Message_t* msg);

