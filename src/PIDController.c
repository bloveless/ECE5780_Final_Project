/*
 * PIDController.c
 *
 *  Created on: Apr 2, 2016
 *      Author: brennon
 */

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "PIDController.h"

#define PIDController_AUTOMATIC 1
#define PIDController_MANUAL 0

TIM_HandleTypeDef PIDController_htim1;
TIM_HandleTypeDef PIDController_htim2;
TIM_HandleTypeDef PIDController_htim3;
TIM_OC_InitTypeDef PIDController_htim3ConfigOC;
osThreadId PIDController_pidControllerTaskHandle;

uint32_t PIDController_leftEncoderCount;
uint32_t PIDController_rightEncoderCount;

// PID Controller tunings
// Completely based on https://github.com/br3ttb/Arduino-PID-Library
uint8_t inAuto;
float *myInput, *myOutput, *mySetpoint, ITerm, lastInput;
uint16_t outMin = 150;
uint16_t outMax = 250;
// Adjust these values to tune the PID Controller
float ki = 5;
float kd = 1;
float kp = 2;
// Default sample time is .1 seconds
// This also controls the delay in the task
uint8_t SampleTime = 250;
// uint32_t lastTime;
// Pulse is the width of the pwm signal (0 - 65535)
float PIDController_Tim1Pulse = 0;
float PIDController_Tim1Last = 0;
float PIDController_Tim1Diff = 0;
float PIDController_Tim1Goal = 5;

/* TIM1 init function */
void PIDController_MX_TIM1_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  PIDController_htim1.Instance = TIM1;
  PIDController_htim1.Init.Prescaler = 0;
  PIDController_htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  PIDController_htim1.Init.Period = 65535;
  PIDController_htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  PIDController_htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&PIDController_htim1);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&PIDController_htim1, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&PIDController_htim1, &sMasterConfig);

}

/* TIM2 init function */
void PIDController_MX_TIM2_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  PIDController_htim2.Instance = TIM2;
  PIDController_htim2.Init.Prescaler = 0;
  PIDController_htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  PIDController_htim2.Init.Period = 65535;
  PIDController_htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&PIDController_htim2);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&PIDController_htim2, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&PIDController_htim2, &sMasterConfig);

}

/* TIM3 init function */
void PIDController_MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  PIDController_htim3.Instance = TIM3;
  PIDController_htim3.Init.Prescaler = 0;
  PIDController_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  PIDController_htim3.Init.Period = 250;
  PIDController_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&PIDController_htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&PIDController_htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&PIDController_htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&PIDController_htim3, &sMasterConfig);

  PIDController_htim3ConfigOC.OCMode = TIM_OCMODE_PWM1;
  PIDController_htim3ConfigOC.Pulse = 0;
  PIDController_htim3ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  PIDController_htim3ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&PIDController_htim3, &PIDController_htim3ConfigOC, TIM_CHANNEL_1);

  PIDController_htim3ConfigOC.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&PIDController_htim3, &PIDController_htim3ConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&PIDController_htim3);

}

void PIDController_SetCounterMode(TIM_HandleTypeDef *htim, uint32_t CounterMode)
{
  assert_param(IS_TIM_INSTANCE(htim->Instance));
  assert_param(IS_TIM_COUNTER_MODE(CounterMode));

  // The timer forgets it's current count when changing modes
  // so save the current counter, then change the mode, then
  // reset the counter
  uint32_t currentCounterValue = __HAL_TIM_GET_COUNTER(htim);
  htim->Init.CounterMode = CounterMode;
  HAL_TIM_Base_Init(htim);
  __HAL_TIM_SET_COUNTER(htim, currentCounterValue);
}

// Completely based on https://github.com/br3ttb/Arduino-PID-Library
void PIDController_ControllerInit(float* Input, float* Output, float* Setpoint,
    float Kp, float Ki, float Kd)
{

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = 0;

  PIDController_ControllerSetTunings(Kp, Ki, Kd);

  // lastTime = HAL_GetTick()-SampleTime;
}

void PIDController_ControllerCompute(void)
{
  // If we are in manual mode then just move along
  if(!inAuto) return;

  // The task will make sure this is run every 100ms
  // no need to do it here
  // unsigned long now = HAL_GetTick();
  // unsigned long timeChange = (now - lastTime);
  // if(timeChange>=SampleTime)
  // {
  /*Compute all the working error variables*/
  volatile float input = *myInput;
  volatile float error = *mySetpoint - input;
  ITerm+= (ki * error);
  if(ITerm > outMax) ITerm= outMax;
  else if(ITerm < outMin) ITerm= outMin;
  volatile float dInput = (input - lastInput);

  /*Compute PID Output*/
  volatile float output = kp * error + ITerm - kd * dInput;

  if(output > outMax) output = outMax;
  else if(output < outMin) output = outMin;
  *myOutput = output;

  /*Remember some variables for next time*/
  lastInput = input;
  // lastTime = now;
  // return 1;
  // }
  //else return 0;
}

void PIDController_ControllerSetTunings(float Kp, float Ki, float Kd)
{
  if (Kp<0 || Ki<0 || Kd<0) return;

  float SampleTimeInSec = ((float)SampleTime)/1000;

  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

void PIDController_ControllerSetMode(int Mode)
{
  uint8_t newAuto = (Mode == PIDController_AUTOMATIC);
  if(newAuto == !inAuto)
  {
    /*we just went from manual to auto*/
    PIDController_ControllerReInitialize();
  }
  inAuto = newAuto;
}

void PIDController_ControllerReInitialize(void)
{
  ITerm = *myOutput;
  lastInput = *myInput;
  if(ITerm > outMax) ITerm = outMax;
  else if(ITerm < outMin) ITerm = outMin;
}

void PIDController_Init(void)
{
  PIDController_MX_TIM1_Init();
  PIDController_MX_TIM2_Init();
  PIDController_MX_TIM3_Init();
  PIDController_ControllerInit(&PIDController_Tim1Diff, &PIDController_Tim1Pulse, &PIDController_Tim1Goal,
      kp, ki, kd);
  PIDController_ControllerSetMode(PIDController_AUTOMATIC);
}

void PIDController_Register(void)
{
  osThreadDef(pidControllerTask, PIDController_Task, osPriorityNormal, 0, 128);
  PIDController_pidControllerTaskHandle = osThreadCreate(osThread(pidControllerTask), NULL);
}

void PIDController_Task(void const * argument)
{
  HAL_TIM_Base_Start(&PIDController_htim1);
  HAL_TIM_Base_Start(&PIDController_htim2);
  HAL_TIM_Base_Start(&PIDController_htim3);
  HAL_TIM_PWM_Start(&PIDController_htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&PIDController_htim3, TIM_CHANNEL_2);

  while(1)
  {
    PIDController_leftEncoderCount = __HAL_TIM_GET_COUNTER(&PIDController_htim1);
    // Calculate how far the wheel as spun since the last interval
    PIDController_Tim1Diff = PIDController_leftEncoderCount - PIDController_Tim1Last;
    PIDController_Tim1Last = PIDController_leftEncoderCount;

    // Update the PWM here to speed up/slow down the motor
    // The new value will be stored in PIDController_Tim1Pulse and PIDController_Tim2Pulse

    // PIDController_rightEncoderCount = __HAL_TIM_GET_COUNTER(&PIDController_htim2);

    // Process the variables for the next time around
    PIDController_ControllerCompute();

    PIDController_htim3ConfigOC.Pulse = PIDController_Tim1Pulse;
    HAL_TIM_PWM_ConfigChannel(&PIDController_htim3, &PIDController_htim3ConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&PIDController_htim3, TIM_CHANNEL_1);

    osDelay(SampleTime);
  }
}

// This is the code to change the counter direction when the motor changes directions
// I needed it out of the way while I work on the PID controller
#if false

    // If the pin is low then we will count up
    if(HAL_GPIO_ReadPin(Counter_Direction_GPIO_Port, Counter_Direction_Pin) == GPIO_PIN_RESET)
    {
      PIDController_SetCounterMode(&PIDController_htim1, TIM_COUNTERMODE_UP);
      PIDController_SetCounterMode(&PIDController_htim2, TIM_COUNTERMODE_UP);
    }
    // If the pin is high then we will count down
    else {
      PIDController_SetCounterMode(&PIDController_htim1, TIM_COUNTERMODE_DOWN);
      PIDController_SetCounterMode(&PIDController_htim2, TIM_COUNTERMODE_DOWN);
    }

#endif
