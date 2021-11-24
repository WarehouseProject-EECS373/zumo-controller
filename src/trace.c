#include "trace.h"

#include <os.h>
#include <os_mem.h>

#include "app_defs.h"


// debug message configuration
#define MSG_OS_TRACE_ID        0x2
#define MSG_OS_TRACE_SIZE      7

#define MSG_DRIVE_TRACE_ID      0x3
#define MSG_DRIVE_TRACE_SIZE    17

#define MSG_DRIVE_TRACE_INIT_ID     0x4
#define MSG_DRIVE_TRACE_INIT_SIZE   5

#define MSG_LINE_FOLLOW_ID          0x5
#define MSG_LINE_FOLLOW_SIZE        13

#if (LINE_FOLLOW_TRACE_ENABLED && OS_TRACE_ENABLED) || (DRIVE_CTL_TRACE_ENABLED && OS_TRACE_ENABLED)
#error "OS_TRACE_ENABLED along with control loop or line following trace. Consider disabling OS tracing to improve performace"
#endif

#ifdef LINE_FOLLOW_TRACE_ENABLED
extern void LineFollowTrace(uint16_t *measurements)
{
    UartLargePacketMessage_t lft_msg;

    lft_msg.base.id = LINE_FOLLOW_TRACE_MSG_ID;
    lft_msg.base.msg_size = sizeof(UartLargePacketMessage_t);
    lft_msg.length = MSG_LINE_FOLLOW_SIZE;
    
    OSStatus_t status;
    uint8_t* mem_block = OSMemoryBlockNew(&lft_msg.mem_key, MEMORY_BLOCK_32, &status);
    
    mem_block[0] = MSG_LINE_FOLLOW_ID;

    uint16_t* m_start = (uint16_t*)(mem_block + 1);
    os_memcpy(m_start, measurements, 6 * sizeof(uint16_t));

    MsgQueuePut(&comms_ss_ao, &lft_msg); 
}
#endif

#ifdef DRIVE_CTL_TRACE_ENABLED
extern void ControlLoopTraceInit(float setpoint)
{
    UartSmallPacketMessage_t clt_msg;

    clt_msg.base.id = DRIVE_CTL_TRACE_INIT_MSG_ID;
    clt_msg.base.msg_size = sizeof(UartSmallPacketMessage_t);
    clt_msg.length = MSG_DRIVE_TRACE_INIT_SIZE;

    clt_msg.payload[0] = MSG_DRIVE_TRACE_INIT_ID;
    float* sp = (float*)(clt_msg.payload + 1);
    *sp = setpoint;

    MsgQueuePut(&comms_ss_ao, &clt_msg);
}

extern void ControlLoopTrace(float left_out, float right_out, float error, float actual)
{
    UartLargePacketMessage_t clt_msg;

    clt_msg.base.id = DRIVE_CTL_TRACE_MSG_ID;
    clt_msg.base.msg_size = sizeof(UartLargePacketMessage_t);
    clt_msg.length = MSG_DRIVE_TRACE_SIZE;

    OSStatus_t status;
    uint8_t* mem_block = OSMemoryBlockNew(&clt_msg.mem_key, MEMORY_BLOCK_32, &status);

    mem_block[0] = MSG_DRIVE_TRACE_ID;
    
    float *m_start = (float*)(mem_block + 1);
    m_start[0] = left_out;
    m_start[1] = right_out;
    m_start[2] = error;
    m_start[3] = actual;

    MsgQueuePut(&comms_ss_ao, &clt_msg);
}
#endif

#ifdef OS_TRACE_ENABLED
extern void DebugPrint(uint8_t ao_id, uint32_t msg_id, uint8_t is_queue)
{
    // don't log debug messages (will create an infinite loop)
    if (OS_DEBUG_MSG_ID == msg_id)
    {
        return;
    }

    UartSmallPacketMessage_t debug_msg;

    // create and send debug message packet to comms event handler
    debug_msg.base.id = OS_DEBUG_MSG_ID;
    debug_msg.base.msg_size = sizeof(UartSmallPacketMessage_t);
    debug_msg.length = MSG_OS_TRACE_SIZE;

    debug_msg.payload[0] = MSG_OS_TRACE_ID;
    debug_msg.payload[1] = ao_id;
    debug_msg.payload[2] = is_queue;

   uint32_t *id_start = (uint32_t*)(debug_msg.payload + 3);
   *id_start = msg_id;

   MsgQueuePut(&comms_ss_ao, &debug_msg);
}
#endif
