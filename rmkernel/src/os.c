#include "inc/os.h"
#include "inc/os_msg.h"

//! internal OS instance pointer
static OS_t* os_ptr;

//! queue head pointers
static ActiveObject_t* activated_ao = NULL;
static TimedEventSimple_t* timed_events = NULL;

static void SchedulerActivateNextAO();
static void SchedulerProcessTimedEvents();

void KernelInit(OS_t* os, OSCallbacksCfg_t* callback_cfg)
{
    // set callbacks
    os->on_Init = callback_cfg->on_Init;
    os->on_Idle = callback_cfg->on_Idle;
    os->on_SysTick = callback_cfg->on_SysTick;

    // set init states and conditions
    os->time = 0;
    os->current_prio = 0xFF;

    // set internal pointer
    os_ptr = os;

    // hook
    if(os_ptr->on_Init)
    {
        os->on_Init();
    }
}

/**
 * @brief Runs on system tick (1ms)
 *
 */
__attribute__((__interrupt__)) void SysTick_Handler()
{
    OS_ISR_ENTER(os_ptr);

    SchedulerProcessTimedEvents();

    // hook
    if(os_ptr->on_SysTick)
    {
        os_ptr->on_SysTick();
    }

    OS_ISR_EXIT(os_ptr);
}

uint32_t OSGetTime()
{
    return os_ptr->time;
}

/**
 * @brief Updates timed events and dispatches if necessary
 *
 */
void SchedulerProcessTimedEvents()
{
    // list traversal head, trail is prev
    TimedEventSimple_t* head = timed_events;
    TimedEventSimple_t* trail = NULL;

    // for each item in list
    while(head)
    {
        head->count++;

        // dispatch if time is up
        if(head->count >= head->period)
        {
            MsgQueuePut(head->dest, (void*)head->message);
            head->count = 0;
        }

        // don't remove if periodic, remove from list if single
        if(TIMED_EVENT_PERIODIC_TYPE == head->type)
        {
            // go to next
            trail = head;
            head = head->next;
        }
        else
        {
            // default to single execution
            head = head->next;

            if(trail)
            {
                // skip over removed
                trail->next = head;
            }
        }
    }
}

void TimedEventSimpleCreate(TimedEventSimple_t* event, ActiveObject_t* dest, void* msg,
                            uint32_t period, TimedEventType_t type)
{
    // set data
    event->message = msg;
    event->period = period;
    event->type = type;
    event->dest = dest;
    event->count = 0;

    event->next = NULL;
}

void SchedulerAddTimedEvent(TimedEventSimple_t* event)
{
    // reset count
    event->count = 0;

    // no self chaining
    if(event != timed_events)
    {
        // when removed connections might not be clean
        event->next = timed_events;
    }

    // set list head pointer
    timed_events = event;
}

void ActiveObjectCreate(ActiveObject_t* ao, uint8_t priority, MessageQueue_t* queue,
                        EventHandler_f handler)
{
    // set instance data
    ao->priority = priority;
    ao->state = AO_WAITING;
    ao->msg_queue = queue;
    ao->handler = handler;

    ao->next = NULL;
    ao->prev = NULL;
}

void SchedulerRun()
{
    while(true)
    {
        // idle loop
        if(os_ptr->on_Idle)
        {
            os_ptr->on_Idle();
        }
    }
}

int Schedule()
{
    // if there's something higher in priority than what's current
    if(activated_ao->priority < os_ptr->current_prio)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void SchedulerActivateAO()
{
    // run all ready tasks
    while(activated_ao)
    {
        activated_ao->state = AO_ACTIVE;

        // set the current execution priority
        os_ptr->current_prio = activated_ao->priority;

        // empty all messages in queue
        while(!MsgQueueIsEmpty(activated_ao->msg_queue))
        {
            Message_t* msg = (Message_t*)MsgQueueGet(activated_ao);
            activated_ao->handler(msg);
        }

        // get next
        SchedulerActivateNextAO();
    }
}

void SchedulerAddReady(ActiveObject_t* ao)
{
    // just add to list if already running, no need to requeue
    if(AO_ACTIVE == ao->state || activated_ao == ao)
    {
        return;
    }

    // extract from list and reinsert to get correct priority (something else might be running too)
    if(AO_READY == ao->state)
    {
        ao->prev->next = ao->next;
        ao->next->prev = ao->prev;
        ao->next = NULL;
        ao->prev = NULL;
    }

    // to make things simple, don't interrupt current AO process, add
    // new ready task right after current running task
    // either prio is higher or lower than current
    // if higher, schedule right after, else, schedule where needed

    // traversal pointers
    ActiveObject_t* temp = activated_ao;
    ActiveObject_t* parent = NULL;

    if(!activated_ao)
    {
        // set list head if nothing has been set yet
        activated_ao = ao;
    }
    else
    {
        // find position
        while(temp && temp->priority < ao->priority)
        {
            parent = temp;
            temp = temp->next;
        }

        if(!parent)
        {
            // start of the list

            // insert after current ao if active, if READY, insert before
            if(activated_ao->state == AO_ACTIVE)
            {
                // don't want to interrupt so put it after
                if(activated_ao->next)
                {
                    activated_ao->next->prev = ao;
                }

                ao->next = activated_ao->next;
                ao->prev = activated_ao;
                activated_ao->next = ao;
            }
            else
            {
                // insert before
                ao->next = activated_ao;
                activated_ao->prev = ao;
                activated_ao = ao;
            }
        }
        else if(!temp)
        {
            // at end
            parent->next = ao;
            ao->prev = parent;
        }
        else
        {
            // list middle somewhere
            parent->next = ao;
            ao->prev = parent;
            ao->next = temp;
            temp->prev = ao;
        }
    }

    // state is ready, ao is queued
    ao->state = AO_READY;
}

void SchedulerActivateNextAO()
{
    // deactivate current
    activated_ao->state = AO_WAITING;

    // extract, break all connections
    ActiveObject_t* prev_activated = activated_ao;
    activated_ao = activated_ao->next;
    activated_ao->prev = NULL;

    prev_activated->next = NULL;
    prev_activated->prev = NULL;

    // reset current priority if all AOs have run
    if(!activated_ao)
    {
        os_ptr->current_prio = 0xFF;
    }
}
