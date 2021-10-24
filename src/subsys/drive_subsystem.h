#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H

extern void Drive_Init();

extern void Drive_Reset();

extern void DriveEventHandler(Message_t *msg);

#endif