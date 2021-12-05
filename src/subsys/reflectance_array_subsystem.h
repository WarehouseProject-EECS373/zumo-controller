#pragma once

#include <os.h>

/**
 * @brief Initialize reflectance array
 *
 */
extern void REFARR_Init();

/**
 * @brief Reflectance array event handler
 *
 * @param msg
 */
extern void ReflectanceArrayEventHandler(Message_t* msg);
