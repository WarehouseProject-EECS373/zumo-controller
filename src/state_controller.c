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
            // start line following at slower speed, speed up once box is picked up
            LineFollowMessage_t line_msg;

            line_msg.base.id = REFARR_START_LINE_FOLLOW_MSG_ID;
            line_msg.base.msg_size = sizeof(LineFollowMessage_t);
            line_msg.response = &state_ctl_ao;
            line_msg.base_speed = 0.25;
            line_msg.intersection_count = destination_aisle_id;

            MsgQueuePut(&refarr_ss_ao, &line_msg);

            // TODO: start electromagnet

            SetNextState(STATE_PICKUP, STATE_PICKUP_DRIVE);
        }
    }
    else if (STATE_PICKUP_DRIVE == sub_state)
    {
        if (FRONT_LIMIT_SWITCH_TRIPPED_MSG_ID == msg->id)
        {
            // speed up after box pickup but don't stop line following yet
            DriveBaseVelocityMessage_t bv_msg;
            bv_msg.base.id = DRIVE_BASE_VELOCITY_MSG_ID;
            bv_msg.base.msg_size = sizeof(DriveBaseVelocityMessage_t);
            bv_msg.base_velocity = 0.5;

            MsgQueuePut(&drive_ss_ao, &bv_msg);
            
            // enter drive to bay state once box is picked up
            SetNextState(STATE_TO_BAY, STATE_TO_BAY_AISLE_DRIVE);
        }
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
