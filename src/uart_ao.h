#ifndef UART_AO_H
#define UART_AO_H

#include <os.h>

extern void UART_Init();

extern void UARTEventHandler(Message_t *msg);

#endif