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

// Default sample time is .1 seconds
// This also controls the delay in the task
uint8_t SampleTime = 250;

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
  // Starting at 4Mhz
  PIDController_htim3.Init.Prescaler = 2;
  PIDController_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Should get us down to 100Hz
  PIDController_htim3.Init.Period = 20000;
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
void PIDController_ControllerCompute(PIDController_Config* pidControllerConfig)
{
  // If we are in manual mode then just move along
  if(!(pidControllerConfig->InAuto)) return;

  // The task will make sure this is run every SampleTime(ms)
  // no need to do it here

  /*Compute all the working error variables*/
  volatile float input = pidControllerConfig->Input;
  volatile float error = pidControllerConfig->Goal - input;
  pidControllerConfig->ITerm += (pidControllerConfig->Ki * error);

  /* Make sure the ITerm is within the OutMin and OutMax limits */
  if(pidControllerConfig->ITerm > pidControllerConfig->OutMax)
  {
    pidControllerConfig->ITerm= pidControllerConfig->OutMax;
  }
  else if(pidControllerConfig->ITerm < pidControllerConfig->OutMin)
  {
    pidControllerConfig->ITerm = pidControllerConfig->OutMin;
  }

  volatile float dInput = (input - pidControllerConfig->LastInput);

  /*Compute PID Output*/
  volatile float output = pidControllerConfig->Kp * error
      + pidControllerConfig->ITerm
      - pidControllerConfig->Kd * dInput;

  if(output > pidControllerConfig->OutMax) {
    output = pidControllerConfig->OutMax;
  }
  else if(output < pidControllerConfig->OutMin) {
    output = pidControllerConfig->OutMin;
  }
  pidControllerConfig->Output = output;

  /*Remember some variables for next time*/
  pidControllerConfig->LastInput = input;
}

void PIDController_ControllerUpdateTunings(PIDController_Config* pidControllerConfig)
{
  if ((pidControllerConfig->Kp < 0)
      || (pidControllerConfig->Ki < 0)
      || (pidControllerConfig->Kd<0)) {
    return;
  }

  float SampleTimeInSec = ((float)SampleTime)/1000;

  pidControllerConfig->Ki = pidControllerConfig->Ki * SampleTimeInSec;
  pidControllerConfig->Kd = pidControllerConfig->Kd / SampleTimeInSec;
}

void PIDController_ControllerSetMode(PIDController_Config* pidControllerConfig, int Mode)
{
  uint8_t newAuto = (Mode == PIDController_AUTOMATIC);

  if(newAuto == !pidControllerConfig->InAuto)
  {
    /*we just went from manual to auto*/
    PIDController_ControllerReInitialize(pidControllerConfig);
  }
  pidControllerConfig->InAuto = newAuto;
}

void PIDController_ControllerReInitialize(PIDController_Config* pidControllerConfig)
{
  pidControllerConfig->ITerm = pidControllerConfig->Output;
  pidControllerConfig->LastInput = pidControllerConfig->Input;

  if(pidControllerConfig->ITerm > pidControllerConfig->OutMax) {
    pidControllerConfig->ITerm = pidControllerConfig->OutMax;
  }
  else if(pidControllerConfig->ITerm < pidControllerConfig->OutMin) {
    pidControllerConfig->ITerm = pidControllerConfig->OutMin;
  }
}

void PIDController_Init(void)
{
  PIDController_MX_TIM1_Init();
  PIDController_MX_TIM2_Init();
  PIDController_MX_TIM3_Init();
  PIDController_ControllerSetMode(&tim1Config, PIDController_AUTOMATIC);
  PIDController_ControllerSetMode(&tim2Config, PIDController_AUTOMATIC);
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

  tim1Config.Ki = 5;
  tim1Config.Kp = 2;
  tim1Config.Kd = 1;
  tim1Config.Input = 0;
  tim1Config.Output = 0;
  tim1Config.Goal = 5;
  tim1Config.OutMin = 10000;
  tim1Config.OutMax = 20000;
  tim1Config.Last = 0;
  tim1Config.Diff = 0;
  tim1Config.LastInput = 0;
  tim1Config.ITerm = 0;
  PIDController_ControllerUpdateTunings(&tim1Config);

  tim2Config.Ki = 5;
  tim2Config.Kp = 2;
  tim2Config.Kd = 1;
  tim2Config.Input = 0;
  tim2Config.Output = 0;
  tim2Config.Goal = 5;
  tim2Config.OutMin = 10000;
  tim2Config.OutMax = 20000;
  tim2Config.Last = 0;
  tim2Config.Diff = 0;
  tim2Config.LastInput = 0;
  tim2Config.ITerm = 0;
  PIDController_ControllerUpdateTunings(&tim2Config);

  int32_t leftEncoderCount, rightEncoderCount;

  // Wait for 5 seconds before we begin
  osDelay(5000);

  while(1)
  {
    leftEncoderCount = __HAL_TIM_GET_COUNTER(&PIDController_htim1);
    rightEncoderCount = __HAL_TIM_GET_COUNTER(&PIDController_htim2);

    // Calculate how far the wheel as spun since the last interval
    tim1Config.Diff = leftEncoderCount - tim1Config.Last;
    tim1Config.Last = leftEncoderCount;

    tim2Config.Diff = rightEncoderCount - tim2Config.Last;
    tim2Config.Last = rightEncoderCount;

    // Update the PWM here to speed up/slow down the motor
    __HAL_TIM_SET_COMPARE(&PIDController_htim3, TIM_CHANNEL_1, (uint32_t) tim1Config.Output);
    __HAL_TIM_SET_COMPARE(&PIDController_htim3, TIM_CHANNEL_2, (uint32_t) tim2Config.Output);

    // Process the variables for the next time around
    PIDController_ControllerCompute(&tim1Config);
    PIDController_ControllerCompute(&tim2Config);

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
