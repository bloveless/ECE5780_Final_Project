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
  PIDController_ControllerSetMode(&tim1Config, PIDController_MANUAL);
  PIDController_ControllerSetMode(&tim2Config, PIDController_MANUAL);

  // And stop the motors
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void PIDController_Start()
{
  PIDController_ControllerSetMode(&tim1Config, PIDController_AUTOMATIC);
  PIDController_ControllerSetMode(&tim2Config, PIDController_AUTOMATIC);
}

void PIDController_SetDirection(int direction)
{
  /*
  if(direction == PIDController_STOP)
  {
    HAL_GPIO_WritePin(Motor_1_Dir_1_GPIO_Port, Motor_1_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_1_Dir_2_GPIO_Port, Motor_1_Dir_2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(Motor_2_Dir_1_GPIO_Port, Motor_2_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_2_Dir_2_GPIO_Port, Motor_2_Dir_2_Pin, GPIO_PIN_RESET);

    return;
  }

  if(direction == PIDController_FORWARD)
  {
    HAL_GPIO_WritePin(Motor_1_Dir_1_GPIO_Port, Motor_1_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_1_Dir_2_GPIO_Port, Motor_1_Dir_2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(Motor_2_Dir_1_GPIO_Port, Motor_2_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_2_Dir_2_GPIO_Port, Motor_2_Dir_2_Pin, GPIO_PIN_RESET);

    return;
  }

  if(direction == PIDController_LEFT)
  {
    HAL_GPIO_WritePin(Motor_1_Dir_1_GPIO_Port, Motor_1_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_1_Dir_2_GPIO_Port, Motor_1_Dir_2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(Motor_2_Dir_1_GPIO_Port, Motor_2_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_2_Dir_2_GPIO_Port, Motor_2_Dir_2_Pin, GPIO_PIN_RESET);
    return;
  }

  if(direction == PIDController_RIGHT)
  {
    HAL_GPIO_WritePin(Motor_1_Dir_1_GPIO_Port, Motor_1_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_1_Dir_2_GPIO_Port, Motor_1_Dir_2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(Motor_2_Dir_1_GPIO_Port, Motor_2_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_2_Dir_2_GPIO_Port, Motor_2_Dir_2_Pin, GPIO_PIN_RESET);

    return;
  }

  // ES NO BUENO
  if(direction == PIDController_SPINLEFT)
  {
    HAL_GPIO_WritePin(Motor_1_Dir_1_GPIO_Port, Motor_1_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_1_Dir_2_GPIO_Port, Motor_1_Dir_2_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(Motor_2_Dir_1_GPIO_Port, Motor_2_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_2_Dir_2_GPIO_Port, Motor_2_Dir_2_Pin, GPIO_PIN_RESET);

    return;
  }

  if(direction == PIDController_SPINRIGHT)
  {
    HAL_GPIO_WritePin(Motor_1_Dir_1_GPIO_Port, Motor_1_Dir_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_1_Dir_2_GPIO_Port, Motor_1_Dir_2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(Motor_2_Dir_1_GPIO_Port, Motor_2_Dir_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_2_Dir_2_GPIO_Port, Motor_2_Dir_2_Pin, GPIO_PIN_SET);

    return;
  }
  */
}

void PIDController_Init(void)
{
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  PIDController_ControllerSetMode(&tim1Config, PIDController_AUTOMATIC);
  PIDController_ControllerSetMode(&tim2Config, PIDController_AUTOMATIC);
}

void PIDController_Register(void)
{
  osThreadDef(pidControllerTask, PIDController_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  PIDController_pidControllerTaskHandle = osThreadCreate(osThread(pidControllerTask), NULL);
}

void PIDController_Task(void const * argument)
{
  tim1Config.Ki = 1000;
  tim1Config.Kp = 400;
  tim1Config.Kd = 50;
  tim1Config.Input = 0;
  tim1Config.Output = 0;
  tim1Config.Goal = 3;
  tim1Config.OutMin = 0;
  tim1Config.OutMax = 20000;
  tim1Config.Last = 0;
  tim1Config.LastInput = 0;
  tim1Config.ITerm = 0;
  tim1Config.InAuto = PIDController_MANUAL;

  tim2Config.Ki = 1000;
  tim2Config.Kp = 400;
  tim2Config.Kd = 50;
  tim2Config.Input = 0;
  tim2Config.Output = 0;
  tim2Config.Goal = 3;
  tim2Config.OutMin = 0;
  tim2Config.OutMax = 20000;
  tim2Config.Last = 0;
  tim2Config.LastInput = 0;
  tim2Config.ITerm = 0;
  tim2Config.InAuto = PIDController_MANUAL;

  volatile int32_t leftEncoderCount, rightEncoderCount;

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
    if(tim1Config.InAuto == PIDController_AUTOMATIC)
    {
      leftEncoderCount = __HAL_TIM_GET_COUNTER(&htim3);

      // Calculate how far the wheel as spun since the last interval
      tim1Config.Input = leftEncoderCount - tim1Config.Last;
      tim1Config.Last = leftEncoderCount;

      // Process the variables for the next time around
      PIDController_ControllerCompute(&tim1Config);

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t) tim1Config.Output);
    }

    if(tim2Config.InAuto == PIDController_AUTOMATIC)
    {
      rightEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

      // Calculate how far the wheel as spun since the last interval
      tim2Config.Input = rightEncoderCount - tim2Config.Last;
      tim2Config.Last = rightEncoderCount;

      // Process the variables for the next time around
      PIDController_ControllerCompute(&tim2Config);

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t) tim2Config.Output);
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
