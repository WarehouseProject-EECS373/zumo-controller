#include "reflectance_array_subsystem.h"

#include <os.h>
#include <stm32f4xx_hal.h>

#include "app_defs.h"
#include "stm/stm32f4xx.h"

// states for periodic event
#define STATE_DISABLED       0x0
#define STATE_CALIBRATE      0x1
#define STATE_FUNCTIONAL     0x2
#define STATE_LINE_FOLLOWING 0x3

// calibration states
#define STATE_CALIBRATE_LEFT_TURN   0x0
#define STATE_CALIBRATE_RIGHT_TURN  0x1
#define STATE_CALIBRATE_RETURN      0x2

// duration of turn of calibration
// should not really be timed but we don't have encoders
// can go to gyro/magnetometer eventually if have time
#define CALIBRATION_TIMED_TURN_DURATION 500


#define REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD  10

static TimedEventSimple_t line_follow_periodic_event;
static  Message_t line_follow_periodic_msg = {.id = REFARR_PERIODIC_EVENT_MSG_ID, .msg_size = sizeof(Message_t)}; 
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static TIM_HandleTypeDef htim5;
static GPIO_InitTypeDef gpioc = {0};
static sensor_values[6];


// current states
static uint32_t state = STATE_DISABLED;
static uint32_t calibration_state = STATE_CALIBRATE_LEFT_TURN;

static uint16_t intersection_count_target = 0;
static uint16_t intersection_count_current = 0;
static ActiveObject_t* line_follow_done_response = NULL;

static void HandlePeriodicEvent();
static void HandleStartCalibration();
static void HandleTimedTurnDoneMsg();
static void StartLineFollow(LineFollowMessage_t *msg);
static void StopLineFollow();

__attribute__((__interrupt__)) extern void TIM3_IRQHandler()
{
    OS_ISR_ENTER();
    HAL_TIM_IRQHandler(&htim3);
    OS_ISR_EXIT();
}

__attribute__((__interrupt__)) extern void TIM4_IRQHandler()
{
    OS_ISR_ENTER();
    HAL_TIM_IRQHandler(&htim4);
    OS_ISR_EXIT();
}

__attribute__((__interrupt__)) extern void TIM5_IRQHandler()
{
    OS_ISR_ENTER();
    HAL_TIM_IRQHandler(&htim5);
    OS_ISR_EXIT();
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

	  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		   TIM3->CR1 &= ~((uint32_t)1);
		   TIM3->CNT = 0;
		   gpioc.Mode = GPIO_MODE_INPUT;
           HAL_GPIO_Init(GPIOC, &gpioc);
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_3);
		   HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_4);
		   HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);
		   HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_2);
		   TIM4->CR1 |= 1;
		   TIM5->CR1 |= 1;
      } 
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM4){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			sensor_values[0] = htim->Instance->CCR1;
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			sensor_values[1] = htim->Instance->CCR2;
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
			sensor_values[2] = htim->Instance->CCR3;
		}
		else {
			sensor_values[3] = htim->Instance->CCR4;
		}
	}
	else if (htim->Instance == TIM5){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			sensor_values[4] = htim->Instance->CCR1;
		}
		else{
			sensor_values[5] = htim->Instance->CCR2;
		}
	}
	if (glob_count == 6){
		TIM4->CR1 &= ~((uint32_t)1);
		TIM5->CR1 &= ~((uint32_t)1);
		TIM4->CNT = 0;
		TIM5->CNT = 0;
		glob_count = 0;
		printf("* %d * %d * %d * %d * %d * %d *\r\n",sensor_values[0],sensor_values[1],
																	sensor_values[2],sensor_values[3],
																	sensor_values[4],sensor_values[5]);
	}


static void HandleStartCalibration()
{
    state = STATE_CALIBRATE;
        
    // start calibration process with timed turn left
    DriveTimedTurn_t ttmsg;
    ttmsg.base.id = DRIVE_TIMED_TURN_MSG_ID;
    ttmsg.base.msg_size = sizeof(DriveTimedTurn_t);
    ttmsg.direction = DRIVE_TURN_DIR_LEFT;
    ttmsg.time = CALIBRATION_TIMED_TURN_DURATION;
    ttmsg.response = &refarr_ss_ao;

    MsgQueuePut(&drive_ss_ao, &ttmsg);

    calibration_state = STATE_CALIBRATE_LEFT_TURN;

}

static void HandleTimedTurnDoneMsg()
{
    if (STATE_CALIBRATE_LEFT_TURN == calibration_state)
    {
        // second calibration step, right rotation
        DriveTimedTurn_t ttmsg;
        ttmsg.base.id = DRIVE_TIMED_TURN_MSG_ID;
        ttmsg.base.msg_size = sizeof(DriveTimedTurn_t);
        ttmsg.direction = DRIVE_TURN_DIR_RIGHT;
        ttmsg.response = &refarr_ss_ao;
        ttmsg.time = 2 * CALIBRATION_TIMED_TURN_DURATION;

        MsgQueuePut(&drive_ss_ao, &ttmsg);
        calibration_state = STATE_CALIBRATE_RIGHT_TURN;
    }
    else if (STATE_CALIBRATE_RIGHT_TURN == calibration_state)
    {
        // third calibration step, left rotation back to start
        DriveTimedTurn_t ttmsg;
        ttmsg.base.id = DRIVE_TIMED_TURN_MSG_ID;
        ttmsg.base.msg_size = sizeof(DriveTimedTurn_t);
        ttmsg.direction = DRIVE_TURN_DIR_LEFT;
        ttmsg.response = &refarr_ss_ao;
        ttmsg.time = CALIBRATION_TIMED_TURN_DURATION;

        MsgQueuePut(&drive_ss_ao, &ttmsg);
        calibration_state = STATE_CALIBRATE_RETURN;
    }
    else if (STATE_CALIBRATE_RETURN == calibration_state)
    {
        // TODO: stop taking repeated calibration measurements of sensors

        Message_t cd_msg = {.id = SM_CALIBRATE_DONE, .msg_size = sizeof(Message_t)};
        MsgQueuePut(&state_ctl_ao, &cd_msg);

        state = STATE_FUNCTIONAL;
    }
}

static void HandlePeriodicEvent()
{
    if (STATE_LINE_FOLLOWING == state)
    {
        if (intersection_count_current > intersection_count_target)
        {
            Message_t ich_msg = {.id = REFARR_INTERSECTION_COUNT_HIT, .msg_size = sizeof(Message_t)};
            MsgQueuePut(line_follow_done_response, &ich_msg);

            StopLineFollow();

        }
        // TODO: take measurements
        // TODO: get new actual position and send to drive
        // TODO: monitor intersection count and send message to state controller
    }
}

static void start_read_sequence(char port, GPIO_InitTypeDef* gpio){
	//initialize read buffer
    uint16_t read_buffer[6] = {3000,3000,3000,3000,3000,3000};
    //Set GPIOs as output
    gpioc.Mode = GPIO_MODE_OUTPUT_PP;
	write_gpio(port, gpio->Pin);
    HAL_GPIO_Init(GPIOC, &gpioc);
    //write GPIOs
    GPIOC->BSRR = gpioc.pin;
    //start sensor read sequence
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
	TIM3->CR1 |= 1;
}



static void StartLineFollow(LineFollowMessage_t *msg)
{
    state = STATE_LINE_FOLLOWING;

    intersection_count_current = 0;
    intersection_count_target = msg->intersection_count;
    line_follow_done_response = msg->response;


    // load "middle of the line" setpoint into drive subsystem
    // drive control loop will use this setpoint when calculating
    // error using the periodic actual position update from REFARR
    DriveSetpointMessage_t sp_msg;
    sp_msg.base.id = DRIVE_SETPOINT_MSG_ID;
    sp_msg.base.msg_size = sizeof(DriveSetpointMessage_t);
    sp_msg.setpoint = 0.0; // FIXME: what is middle of robot?

    DriveBaseVelocityMessage_t bv_msg;
    bv_msg.base.id = DRIVE_BASE_VELOCITY_MSG_ID;
    bv_msg.base.msg_size = sizeof(DriveBaseVelocityMessage_t);
    bv_msg.base_velocity = msg->base_speed;

    MsgQueuePut(&drive_ss_ao, &sp_msg);
    MsgQueuePut(&drive_ss_ao, &bv_msg);


    TimedEventSimpleCreate(&line_follow_periodic_event, &refarr_ss_ao, &line_follow_periodic_msg, REFLECTANCE_ARRAY_LINE_FOLLOW_PERIOD, TIMED_EVENT_PERIODIC_TYPE);
    SchedulerAddTimedEvent(&line_follow_periodic_event);
}

static void StopLineFollow()
{
    state = STATE_FUNCTIONAL;

    DriveOpenLoopControlMessage_t olctl_msg;
    olctl_msg.base.id = DRIVE_OPEN_LOOP_MSG_ID;
    olctl_msg.base.msg_size = sizeof(DriveOpenLoopControlMessage_t);
    olctl_msg.percent_left = 0.0;
    olctl_msg.percent_right = 0.0;

    MsgQueuePut(&drive_ss_ao, &olctl_msg);
    TimedEventDisable(&line_follow_periodic_event);
}

extern void REFARR_Init()
{
    //init gpios
gpioc.Pull = GPIO_NOPULL;
gpioc.Speed = GPIO_SPEED_FREQ_LOW;
gpioc.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                                  |GPIO_PIN_4|GPIO_PIN_5;
    //init compare timer
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  TIM3->CCR1 = delay1;
    
//capture timers init
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

//timer5

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 15;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}



extern void ReflectanceArrayEventHandler(Message_t* msg)
{
    if (msg->id == REFARR_CALIBRATE_MSG_ID)
    {
        HandleStartCalibration();
    }
    else if (DRIVE_TIMED_TURN_DONE_MSG_ID == msg->id)
    {
        HandleTimedTurnDoneMsg();
    }
    else if (REFARR_START_LINE_FOLLOW_MSG_ID == msg->id)
    {
        StartLineFollow((LineFollowMessage_t*)msg);
    }
    else if (REFARR_STOP_LINE_FOLLOW_MSG_ID == msg->id)
    {
        StopLineFollow();
    }
    else if (REFARR_PERIODIC_EVENT_MSG_ID == msg->id)
    {
        HandlePeriodicEvent();       
    }
}
