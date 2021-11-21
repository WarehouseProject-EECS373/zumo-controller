#include "state_controller.h"

#include <os.h>

#include "app_defs.h"

// operation modes
// - idle state
// - box from input to bay
// - box from bay to output
#define MODE_IDLE           0x000
#define MODE_LOADING_TO_BAY 0x100
#define MODE_BAY_TO_OUT     0x101

// idle state
#define STATE_IDLE                          0x0

// box pickup from input
#define STATE_PICKUP                        0x10
#define STATE_PICKUP_CALIBRATE              0x11
#define STATE_PICKUP_DRIVE                  0x12
#define STATE_PICKUP_PICKUP                 0x13
#define STATE_PICKUP_REVERSE                0x14

// input to bay
#define STATE_TO_BAY                        0x30
#define STATE_TO_BAY_FORWARD                0x31
#define STATE_TO_BAY_AISLE_TURN             0x32
#define STATE_TO_BAY_AISLE_DRIVE            0x32
#define STATE_TO_BAY_BAY_TURN               0x33

// drop off box in bay
#define STATE_BAY_DROPOFF                   0x50
#define STATE_BAY_DROPOFF_DRIVE             0x51
#define STATE_BAY_DROPOFF_RELEASE           0x52
#define STATE_BAY_DROPOFF_REVERSE           0x53
#define STATE_BAY_DROPOFF_TURN              0x54

// return to idle bay after dropoff
#define STATE_IDLE_RETURN                   0x70
#define STATE_IDLE_RETURN_EXIT_AISLE        0x71
#define STATE_IDLE_RETURN_TURN              0x72
#define STATE_IDLE_RETURN_DRIVE             0x73

// current states
static uint32_t state = STATE_IDLE;
static uint32_t sub_state = STATE_IDLE;

// no aisle or bay should have ID 0xff
static uint8_t destination_bay_id = 0xff;
static uint8_t destination_aisle_id = 0xff;

static void HandleIdleState(Message_t *msg);
static void HandlePickupState(Message_t *msg);
static void HandleToBayState(Message_t *msg);
static void HandleDropoffState(Message_t *msg);
static void HandleReturnToIdleState(Message_t *msg);

static void SetNextState(uint32_t new_state, uint32_t new_sub_state);

static void SetNextState(uint32_t new_state, uint32_t new_sub_state)
{
    state = new_state;
    sub_state = new_sub_state;
}

static void HandleIdleState(Message_t *msg)
{
    if(SM_PERIODIC_EVENT_MSG_ID == msg->id)
    {
        // nothing to do for periodic event while in IDLE state
    }
    else if (SM_DISPATCH_FROM_IDLE_MSG_ID == msg->id)
    {
        // if id is correct, cast pointer to a dispatch message
        DispatchMessage_t *dmsg = (DispatchMessage_t*)msg;
        
        destination_bay_id = dmsg->bay_id;
        destination_aisle_id = dmsg->aisle_id;

        // enable the drive
        Message_t drive_enable_msg = {.id = DRIVE_ENABLE_MSG_ID, .msg_size = sizeof(Message_t)};
        MsgQueuePut(&drive_ss_ao, &drive_enable_msg);

        // start calibration process
        Message_t calibrate_msg = {.id = REFARR_CALIBRATE_MSG_ID, .msg_size = sizeof(Message_t)};
        MsgQueuePut(&refarr_ss_ao, &calibrate_msg);

        SetNextState(STATE_PICKUP, STATE_PICKUP_CALIBRATE);
    }
}

static void HandlePickupState(Message_t *msg)
{
    if (STATE_PICKUP_CALIBRATE == sub_state)
    {
        if (SM_CALIBRATE_DONE == msg->id)
        {
            // start drive to pickup action
            SetNextState(STATE_PICKUP, STATE_PICKUP_DRIVE);
        }
    }
    else if (STATE_PICKUP_DRIVE == sub_state)
    {
        // TODO: drive until limit switch
        // TODO: start electromagnet
    }
    else if (STATE_PICKUP_PICKUP == sub_state)
    {
        // TODO: start reverse
    }
    else if (STATE_PICKUP_REVERSE == sub_state)
    {
        // TODO: go to drive to bay state
    }
}

static void HandleToBayState(Message_t *msg)
{
    UNUSED(msg);

    
}

static void HandleDropoffState(Message_t *msg)
{
    UNUSED(msg);
}

static void HandleReturnToIdleState(Message_t *msg)
{
    UNUSED(msg);
}

extern void StateController_Init()
{
}

extern void StateControllerEventHandler(Message_t *msg)
{
   if (STATE_IDLE == state)
   {
       HandleIdleState(msg);
   }
   else if (STATE_PICKUP == state)
   {
       HandlePickupState(msg);
   }
   else if (STATE_TO_BAY == state)
   {
       HandleToBayState(msg);
   }
   else if (STATE_BAY_DROPOFF == state)
   {
        HandleDropoffState(msg);
   }
   else if (STATE_IDLE_RETURN == state)
   {
       HandleReturnToIdleState(msg);
   }

}
