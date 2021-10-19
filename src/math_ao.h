#ifndef MATH_AO_H
#define MATH_AO_H

#include <os.h>

extern void DataProducerEventHandler(Message_t *msg);


extern void AveragerEventHandler(Message_t *msg);


extern void IntegratorEventHandler(Message_t *msg);


extern void DifferentiatorEventHandler(Message_t *msg);

#endif