
#include "os_mem.h"

#define POOL_SIZE 512

static uint8_t pool[POOL_SIZE] = {0};
static uint32_t used = 0;

/**
 * @brief Get a key to a block of pre-allocated memory
 *
 * @param key used to access the memory block useing OSMemoryBlockGet
 * @param size number of bytes to get (see BlockSize_t)
 * @param status OS_MEMORY_SUCCESS if able to find a free block, OS_MEMORY_BLOCK_FULL otherwise
 * @return uint8_t* pointer to block of memory
 */
extern uint8_t* OSMemoryBlockNew(uint16_t* key, BlockSize_t size, OSStatus_t* status)
{
    *key = 0;
    uint8_t* block = NULL;

    // create mask
    uint8_t block_bits = (size / MEMORY_BLOCK_32);
    uint32_t search_mask = (1U << block_bits) - 1;

    // make sure we're not going to loop forever
    if(0 >= POOL_SIZE / 32 - size)
    {
        *status = OS_ERROR;
        return NULL;
    }

    // don't want to be interrupted while getting this memory,
    // multiple objects could get keys to the same memory
    DISABLE_INTERRUPTS();

    // iterate and shift on multiple of of block size (i.e. the number of bits in "used")
    for(uint8_t i = 0; i < (POOL_SIZE / 32) - block_bits; i += block_bits)
    {
        // if an empty location is found, use it
        if(0 == (search_mask & used))
        {
            block = pool + (i * MEMORY_BLOCK_32);
            used |= search_mask;
            *key = (size << 8) | i;

            break;
        }

        search_mask <<= block_bits;
    }

    ENABLE_INTERRUPTS();

    // set status
    if(NULL == block)
    {
        *status = OS_MEMORY_BLOCK_FULL;
    }
    else
    {
        *status = OS_SUCCESS;
    }

    // return pointer to block
    return block;
}

/**
 * @brief Gets a pointer to the block of memory encoded in the key
 *
 * @param key
 * @return uint8_t* pointer to the block of memory
 */
extern uint8_t* OSMemoryBlockGet(uint16_t key)
{
    // return the index of the block
    return pool + (key & 0xff);
}

/**
 * @brief Frees a block of memory so that it can be used again.
 *
 * @param key
 * @return OSStatus_t OS_SUCCESS
 */
extern OSStatus_t OSMemoryFreeBlock(uint16_t key)
{
    // build the mask
    uint8_t block_bits = ((key >> 8) / MEMORY_BLOCK_32);
    uint32_t clear_mask = ~(((1 << block_bits) - 1) << (key & 0xff));

    // don't want to be interrupted here either
    DISABLE_INTERRUPTS();
    used &= clear_mask;
    ENABLE_INTERRUPTS();

    return OS_SUCCESS;
}