#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H

#include <os.h>

extern void Drive_Init();

extern void DriveEventHandler(Message_t* msg);

#endif