#ifndef GPIO_AO_H
#define GPIO_AO_H

__attribute__((__interrupt__)) extern void EXTI15_10_IRQHandler(void);

extern void GPIO_Init();

#endif