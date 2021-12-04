#include "state_controller.h"

#include <os.h>

#include "app_defs.h"

#include "state_machine.h"

#include "cmd/delay_cmd.h"
#include "cmd/drive_open_loop_cmd.h"
#include "cmd/lf_cmd.h"
#include "cmd/turn180_cmd.h"
#include "cmd/turn_cmd.h"

static void HandleDisptach(DispatchMessage_t *msg);


static void HandleDisptach(DispatchMessage_t *msg)
{
    UNUSED(msg);
}

extern void StateController_Init()
{
}

extern void StateControllerEventHandler(Message_t *msg)
{
    if (SM_DISPATCH_FROM_IDLE_MSG_ID == msg->id)
    {
        HandleDisptach((DispatchMessage_t*)msg);
    }
}
