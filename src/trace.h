#ifndef TRACE_H
#define TRACE_H

#include <os.h>

#include "app_defs.h"

#ifdef LINE_FOLLOW_TRACE_ENABLED
extern void LineFollowTrace(uint16_t *measurements);
#endif

#ifdef DRIVE_CTL_TRACE_ENABLED
extern void ControlLoopTraceInit(float setpoint);
extern void ControlLoopTrace(float left_out, float right_out, float error, float actual);
#endif

#ifdef OS_TRACE_ENABLED
extern void DebugPrint(uint8_t ao_id, uint32_t msg_id, uint8_t is_queue);
#endif

#endif
