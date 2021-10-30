#ifndef UI_SUBSYSTEM_H
#define UI_SUBSYSTEM_H

#include <os.h>

__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void);

extern void ITCTL_Init();

extern void InputHandler(Message_t*);

#endif
