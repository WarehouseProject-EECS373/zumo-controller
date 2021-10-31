#ifndef REFLECTANCE_SENSOR_ARRAY_H
#define REFLECTANCE_SENSOR_ARRAY_H

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

#endif
