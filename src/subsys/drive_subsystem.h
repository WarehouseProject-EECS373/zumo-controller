#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H

#include <os.h>

extern void Drive_Init();

extern void Drive_SetOutputPercent(float left_percent_output, float right_percent_output);

extern void DriveEventHandler(Message_t* msg);

#endif