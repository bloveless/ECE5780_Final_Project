/*
 * PIDController.c
 *
 *  Created on: Apr 2, 2016
 *      Author: brennon
 */

#include "PIDController.h"

// Default sample time is .1 seconds
// This also controls the delay in the task
uint8_t SampleTime = 100;

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

/**
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
**/

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

void PIDController_Stop()
{
  // Disable the PIDController
  PIDController_ControllerSetMode(&leftTrackConfig, PIDController_MANUAL);
  PIDController_ControllerSetMode(&rightTrackConfig, PIDController_MANUAL);

  // And stop the motors
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
}

void PIDController_Start()
{
  PIDController_ControllerSetMode(&leftTrackConfig, PIDController_AUTOMATIC);
  PIDController_ControllerSetMode(&rightTrackConfig, PIDController_AUTOMATIC);
}

void PIDController_SetDirection(int direction)
{
  if(direction == PIDController_FORWARD)
  {
    HAL_GPIO_WritePin(Left_Motor_Dir_1_GPIO_Port, Left_Motor_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Left_Motor_Dir_2_GPIO_Port, Left_Motor_Dir_2_Pin, GPIO_PIN_RESET);
  }
  if(direction == PIDController_SPIN)
  {
    HAL_GPIO_WritePin(Left_Motor_Dir_1_GPIO_Port, Left_Motor_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Left_Motor_Dir_2_GPIO_Port, Left_Motor_Dir_2_Pin, GPIO_PIN_SET);
  }
}

void PIDController_Init(void)
{
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);

  // PWM Right Track
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  // PWM Left Track
  HAL_TIM_Base_Start(&htim17);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  PIDController_ControllerSetMode(&leftTrackConfig, PIDController_AUTOMATIC);
  PIDController_ControllerSetMode(&rightTrackConfig, PIDController_AUTOMATIC);
}

void PIDController_Register(void)
{
  osThreadDef(pidControllerTask, PIDController_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  PIDController_pidControllerTaskHandle = osThreadCreate(osThread(pidControllerTask), NULL);
}

void PIDController_Task(void const * argument)
{
  leftTrackConfig.Ki = 1000;
  leftTrackConfig.Kp = 400;
  leftTrackConfig.Kd = 50;
  leftTrackConfig.Input = 0;
  leftTrackConfig.Output = 0;
  leftTrackConfig.Goal = 3;
  leftTrackConfig.OutMin = 0;
  leftTrackConfig.OutMax = 65535;
  leftTrackConfig.Last = 0;
  leftTrackConfig.LastInput = 0;
  leftTrackConfig.ITerm = 0;
  leftTrackConfig.InAuto = PIDController_MANUAL;

  rightTrackConfig.Ki = 1000;
  rightTrackConfig.Kp = 400;
  rightTrackConfig.Kd = 50;
  rightTrackConfig.Input = 0;
  rightTrackConfig.Output = 0;
  rightTrackConfig.Goal = 3;
  rightTrackConfig.OutMin = 0;
  rightTrackConfig.OutMax = 65535;
  rightTrackConfig.Last = 0;
  rightTrackConfig.LastInput = 0;
  rightTrackConfig.ITerm = 0;
  rightTrackConfig.InAuto = PIDController_MANUAL;

  volatile int32_t leftEncoderCount = 0;
  volatile int32_t rightEncoderCount = 0;

  // Wait for 5 seconds before we begin
  osDelay(5000);

  // Set the motors to stay stopped
  // PIDController_SetDirection(PIDController_STOP);
  // PIDController_Stop();

  PIDController_SetDirection(PIDController_FORWARD);
  PIDController_Start();

  while(1)
  {
    /*
    if(doSpin)
    {
      // Wait for a complete stop
      osDelay(1000);

      // found a wall so turn around
      PIDController_SetDirection(PIDController_SPINRIGHT);
      PIDController_Start();

      osDelay(2000);

      PIDController_Stop();
      doSpin = 0;
    }
    */

    // Update the PWM here to speed up/slow down the motor
    if(rightTrackConfig.InAuto == PIDController_AUTOMATIC)
    {
      rightEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

      // Calculate how far the wheel as spun since the last interval
      rightTrackConfig.Input = rightEncoderCount - rightTrackConfig.Last;
      rightTrackConfig.Last = rightEncoderCount;

      // Process the variables for the next time around
      PIDController_ControllerCompute(&rightTrackConfig);

      __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (uint32_t) rightTrackConfig.Output);
    }

    if(leftTrackConfig.InAuto == PIDController_AUTOMATIC)
    {
      leftEncoderCount = __HAL_TIM_GET_COUNTER(&htim1);

      // Calculate how far the wheel as spun since the last interval
      leftTrackConfig.Input = leftEncoderCount - leftTrackConfig.Last;
      leftTrackConfig.Last = leftEncoderCount;

      // Process the variables for the next time around
      PIDController_ControllerCompute(&leftTrackConfig);

      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (uint32_t) leftTrackConfig.Output);
    }

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
