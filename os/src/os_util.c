#include "os_util.h"

#include <stdint.h>

/**
 * @brief Memory copy
 *
 * @param dest pointer to destination
 * @param src pointer to source
 * @param len bytes to copy
 * @return void* pointer to destination
 */
extern void *os_memcpy(void *dest, const void *src, uint32_t len)
{
    uint8_t *d = (uint8_t *)dest;
    const uint8_t *s = (uint8_t *)src;

    while (len--)
    {
        *d++ = *s++;
    }

    return dest;
}