#include "state_controller.h"

#include <os.h>

#include "app_defs.h"

#define MODE_IDLE           0x000
#define MODE_LOADING_TO_BAY 0x100
#define MODE_BAY_TO_OUT     0x101


#define STATE_IDLE                          0x0

#define STATE_PICKUP                        0x10
#define STATE_PICKUP_CALIBRATE              0x11
#define STATE_PICKUP_DRIVE                  0x12
#define STATE_PICKUP_PICKUP                 0x13
#define STATE_PICKUP_REVERSE                0x14

#define STATE_TO_BAY                        0x30
#define STATE_TO_BAY_FORWARD                0x31
#define STATE_TO_BAY_AISLE_TURN             0x32
#define STATE_TO_BAY_AISLE_DRIVE            0x32
#define STATE_TO_BAY_BAY_TURN               0x33

#define STATE_BAY_DROPOFF                   0x50
#define STATE_BAY_DROPOFF_DRIVE             0x51
#define STATE_BAY_DROPOFF_RELEASE           0x52
#define STATE_BAY_DROPOFF_REVERSE           0x53
#define STATE_BAY_DROPOFF_TURN              0x54

#define STATE_IDLE_RETURN                   0x70
#define STATE_IDLE_RETURN_EXIT_AISLE        0x71
#define STATE_IDLE_RETURN_TURN              0x72
#define STATE_IDLE_RETURN_DRIVE             0x73


static uint32_t mode = MODE_IDLE;
static uint32_t state = STATE_IDLE;
static uint32_t sub_state = STATE_IDLE;

static void HandleIdleState(Message_t *msg);
static void HandlePickupState(Message_t *msg);
static void HandleToBayState(Message_t *msg);
static void HandleDropoffState(Message_t *msg);
static void HandleReturnToIdleState(Message_t *msg);

static void HandleIdleState(Message_t *msg)
{
    mode = MODE_IDLE;
    sub_state = STATE_IDLE;

    UNUSED(msg);
}

static void HandlePickupState(Message_t *msg)
{
    UNUSED(msg);
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
   if(STATE_IDLE == state)
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
