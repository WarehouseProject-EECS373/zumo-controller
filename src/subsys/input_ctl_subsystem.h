#ifndef UI_SUBSYSTEM_H
#define UI_SUBSYSTEM_H

__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void);

extern void ITCTL_Init();

#endif
