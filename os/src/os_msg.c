

#include "os_msg.h"
#include "os.h"

static uint16_t GetAvailableCount(MessageQueue_t *q);

static void AdvancePointer(MessageQueue_t *q);
static void RetreatPointer(MessageQueue_t *q);

bool MsgQueueIsEmpty(MessageQueue_t *q)
{
    // not full and head and tail are the same
    return !q->is_full && (q->head == q->tail);
}

/**
 * @brief get the amount of space left in the queue
 *
 * @param q
 * @return uint16_t
 */
uint16_t GetAvailableCount(MessageQueue_t *q)
{
    uint16_t size = q->size;

    if (!q->is_full)
    {
        // either subtract, or offset then substract to get size
        if (q->head >= q->tail)
        {
            size = q->head - q->tail;
        }
        else
        {
            size = q->size + q->head - q->tail;
        }
    }

    return size;
}

/**
 * @brief Move pointer to the next item in the list
 *
 * @param q
 */
void AdvancePointer(MessageQueue_t *q)
{
    // if full
    if (q->is_full)
    {
        // advance and wrap the tail
        ++q->tail;

        if (q->size == q->tail)
        {
            q->tail = 0;
        }
    }

    // always advance and wrap head
    ++q->head;

    if (q->size == q->head)
    {
        q->head = 0;
    }

    // update is_full
    q->is_full = (q->head == q->tail);
}

/**
 * @brief Call when removing item from list
 *
 * @param q
 */
void RetreatPointer(MessageQueue_t *q)
{
    q->is_full = false;

    // move and wrap tail index
    ++q->tail;

    if (q->size == q->tail)
    {
        q->tail = 0;
    }
}

void MsgQueueCreate(MessageQueue_t *q, const uint16_t size, MessageGeneric_t *queue)
{
    // set instance data
    q->size = size;
    q->queue = queue;
    q->head = 0;
    q->tail = 0;
    q->is_full = false;
}

MessageQueueStatus_t MsgQueuePut(ActiveObject_t *dest, void *msg)
{
    // critical section
    DISABLE_INTERRUPTS();
    MessageQueueStatus_t status = MSG_Q_SUCCESS;

    // add only if full
    if (!dest->msg_queue->is_full)
    {
        // copy message into buffer
        os_memcpy(&dest->msg_queue->queue[dest->msg_queue->head], msg, ((Message_t *)msg)->msg_size);
        AdvancePointer(dest->msg_queue);

        // notify scheduler to make destination AO ready
        SchedulerAddReady(dest);
    }
    else
    {
        status = MSG_Q_FULL;
    }

    ENABLE_INTERRUPTS();

    return status;
}

void *MsgQueueGet(ActiveObject_t *ao)
{
    void *data;

    while (MsgQueueIsEmpty(ao->msg_queue))
    {
        // block, should not block because MsgQueueGet wshould not be called unless message is in queue
    }

    // get first message in queue
    data = (void *)&ao->msg_queue->queue[ao->msg_queue->tail];

    // move up read index
    RetreatPointer(ao->msg_queue);

    return data;
}