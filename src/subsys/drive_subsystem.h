#pragma once

#include <os.h>

/**
 * @brief Initialize drive subsystem. GPIOs and Timer
 *
 */
extern void Drive_Init();

/**
 * @brief Event handler
 *
 * @param msg
 */
extern void DriveEventHandler(Message_t* msg);

