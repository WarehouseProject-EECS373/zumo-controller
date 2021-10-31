#ifndef OS_DEFS_H
#define OS_DEFS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OS_BASEPRI 0x3F

#define OS_SUCCESS           0
#define OS_INVALID_ARGUMENT  1
#define OS_MEMORY_BLOCK_FULL 2
#define OS_ERROR             3

#define OS_MSG_DATA_TYPE    0
#define OS_MESSAGE_MAX_SIZE 20

// clang-format off
#define ENABLE_INTERRUPTS() __asm volatile ("cpsie i" ::: "memory");
#define DISABLE_INTERRUPTS() __asm volatile ("cpsid i" ::: "memory");

#define ERRATUM() __asm volatile("dsb" ::: "memory")
// clang-format on

#define STATIC_ASSERT(X)                                                          \
    ({                                                                            \
        extern int __attribute__((error("assertion failure: '" #X "' not true"))) \
        compile_time_check();                                                     \
        ((X) ? 0 : compile_time_check()), 0;                                      \
    })

/**
 * @brief Macro to be called upon entering an ISR
 *
 * More of a temporary placeholder for future development right now
 *
 */
#define OS_ISR_ENTER(os) \
    {                    \
    }

/**
 * @brief Macro to be called upon exiting an ISR
 *
 * Sets PendSV bit if an AO has been queued during ISR handling
 *
 */
#define OS_ISR_EXIT(os)                                \
    {                                                  \
        DISABLE_INTERRUPTS();                          \
        if(0U != Schedule())                           \
        {                                              \
            *((uint32_t*)(0xE000ED04U)) = (1U << 28U); \
        }                                              \
        ENABLE_INTERRUPTS();                           \
        ERRATUM();                                     \
    }

typedef uint8_t OSStatus_t;

//! see os.h
typedef struct OS_s OS_t;

//! see os.h
typedef struct OSCallbacksCfg_s OSCallbacksCfg_t;

//! see os_msg.h
typedef struct MessageQueue_s MessageQueue_t;

//! see os_msg.h
typedef struct MessageGeneric_s MessageGeneric_t;
typedef struct Message_s Message_t;
typedef struct DataMessage_s DataMessage_t;
typedef struct MemoryBlockMessage_s MemoryBlockMessage_t;

//! see os.h
typedef struct ActiveObject_s ActiveObject_t;

//! see os.h
typedef struct TimedEventSimple_s TimedEventSimple_t;

/**
 * @brief Event handler. Must run to completion. No blocking allowed!! Should run quickly
 *
 */
typedef void (*EventHandler_f)(Message_t*);

#endif