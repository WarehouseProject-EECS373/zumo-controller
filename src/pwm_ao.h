#ifndef PWM_H
#define PWM_H

#include <os.h>

extern void PWM_Init();

extern void PWM_Start();

extern void DualPwmEventHandler(Message_t *msg);

#endif