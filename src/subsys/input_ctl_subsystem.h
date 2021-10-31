#ifndef UI_SUBSYSTEM_H
#define UI_SUBSYSTEM_H

#include <os.h>

/**
 * @brief ISR for handling EXTI 10 through 15
 *
 */
__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void);

/**
 * @brief Initialize input control subsystem
 *
 */
extern void ITCTL_Init();

/**
 * @brief Event handler for input control subsystem
 *
 */
extern void InputEventHandler(Message_t*);

#endif
