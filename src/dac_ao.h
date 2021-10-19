#ifndef DAC_AO_H
#define DAC_AO_H

#include <os.h>

extern void DAC_Init();

extern void DAC_Start();

extern void DAC1EventHandler(Message_t *msg);

extern void DAC2EventHandler(Message_t *msg);


#endif