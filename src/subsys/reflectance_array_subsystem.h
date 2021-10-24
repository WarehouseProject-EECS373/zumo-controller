#ifndef REFLECTANCE_SENSOR_ARRAY_H
#define REFLECTANCE_SENSOR_ARRAY_H

#include <os.h>

extern void REFARR_Init();

extern void ReflectanceArrayEventHandler(Message_t* msg);

#endif
