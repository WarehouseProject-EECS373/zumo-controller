#ifndef STCP_H
#define STCP_H

#include <stdint.h>

#define HEADER ((uint8_t)0x7A)
#define FOOTER ((uint8_t)0x7F)
#define ESCAPE ((uint8_t)0x7B)

#define STCP_STATUS_SUCCESS         0
#define STCP_STATUS_UNDEFINED_ERROR 1
#define STCP_STATUS_CRC_ERROR       2

#define STCP_STATE_FUNCTIONAL   0
#define STCP_STATE_INACTIVE     1
#define STCP_STATE_ERROR        2

typedef uint8_t STCPState_t;
typedef uint8_t STCPStatus_t;

typedef struct STCPCallbacks_s
{
    STCPStatus_t (*Send)(void *buffer, uint16_t length, void *instance_data);
    STCPStatus_t (*HandleMessage)(void *buffer, uint16_t length, void *instance_data);
} STCPCallbacks_t;

typedef struct STCPEngine_s
{
    STCPCallbacks_t callbacks;
    void *instance_data;
} STCPEngine_t;

typedef STCPEngine_t *STCPEngineHandler_t;

extern uint32_t Crc32(uint8_t *buffer, uint16_t length);

extern STCPStatus_t StcpWrite(STCPEngineHandler_t instance, uint8_t *buffer, uint16_t size);

extern STCPStatus_t StcpHandleMessage(STCPEngineHandler_t instance, uint8_t *buffer, uint16_t size);

#endif
