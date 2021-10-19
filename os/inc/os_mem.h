#ifndef OS_MEM_H
#define OS_MEM_H

#include "os_defs.h"

#define MEMORY_BLOCK_32 32
#define MEMORY_BLOCK_64 64
#define MEMORY_BLOCK_128 128
#define MEMORY_BLOCK_256 256

typedef uint32_t BlockSize_t;

extern uint8_t *
OSMemoryBlockNew(uint16_t *key, BlockSize_t size, OSStatus_t *status);

extern uint8_t *OSMemoryBlockGet(uint16_t key);

extern OSStatus_t OSMemoryFreeBlock(uint16_t key);

#endif