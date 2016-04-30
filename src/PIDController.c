/*
 * PIDController.c
 *
 *  Created on: Apr 2, 2016
 *      Author: brennon
 */

#include "PIDController.h"

/**
 * To start the PIDController we will start timer 1 and 2 as the
 * encoder counters.
 *
 * Then start timer 16 and 17 for the PWM for the Right and Left
 * track of the bot.
 */
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

  SampleTime = 100;
  PIDController_CurrentMode = PIDController_NORMALMODE;
}

/**
 * Register the PIDController with FreeRTOS
 */
void PIDController_Register(void)
{
  osThreadDef(pidControllerTask, PIDController_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  PIDController_pidControllerTaskHandle = osThreadCreate(osThread(pidControllerTask), NULL);
}

/**
 * The PIDController task will compare the encoders with the goal speed
 * and adjust the PWM output to achieve this goal
 *
 * The task also takes care of spinning the robot around when it detects
 * a wall
 */
void PIDController_Task(void const * argument)
{
  PIDController_ControllerReset(&leftTrackConfig);
  PIDController_ControllerReset(&rightTrackConfig);

  uint16_t leftEncoderCount = 0;
  uint16_t rightEncoderCount = 0;
  uint16_t leftSpinStartCount = 0;
  uint8_t leftSpinCountThreshold = 15;
  uint16_t rightSpinStartCount = 0;
  uint8_t rightSpinCountThreshold = 13;
  uint8_t caseIndex = 0;

  // Wait for 5 seconds before we begin
  osDelay(5000);

  /**
   * Set the robot to drive forward and tell the PIDControllers
   * to start driving
   */
  PIDController_SetDirection(PIDController_FORWARD);
  PIDController_Start();

  while(1)
  {
    /**
     * Update the PWM here to speed up/slow down the motor
     */
    if(rightTrackConfig.InAuto == PIDController_AUTOMATIC)
    {
      rightEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

      /**
       * Calculate how far the wheel as spun since the last interval
       */
      rightTrackConfig.Input = rightEncoderCount - rightTrackConfig.Last;
      rightTrackConfig.Last = rightEncoderCount;

      /**
       * Process the variables for the next time around
       */
      PIDController_ControllerCompute(&rightTrackConfig);

      __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (uint32_t) rightTrackConfig.Output);
    }

    /**
     * Update the PWM here to speed up/slow down the motor
     */
    if(leftTrackConfig.InAuto == PIDController_AUTOMATIC)
    {
      leftEncoderCount = __HAL_TIM_GET_COUNTER(&htim1);

      /**
       * Calculate how far the wheel as spun since the last interval
       */
      leftTrackConfig.Input = leftEncoderCount - leftTrackConfig.Last;
      leftTrackConfig.Last = leftEncoderCount;

      /**
       * Process the variables for the next time around
       */
      PIDController_ControllerCompute(&leftTrackConfig);

      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (uint32_t) leftTrackConfig.Output);
    }

    /**
     * When the Proximity module signals for a spin the PIDController
     * will execute this code
     */
    if(PIDController_CurrentMode == PIDController_SPINMODE)
    {
      /**
       * The PIDController must execute the next few command in order
       * and wait for them to complete.
       */
      switch(caseIndex)
      {
        /**
         * First set on of the wheels to spin in the reverse direction
         */
        case 0:
          leftSpinStartCount = leftEncoderCount;
          rightSpinStartCount = rightEncoderCount;
          PIDController_SetDirection(PIDController_SPIN);
          PIDController_ControllerSetMode(&leftTrackConfig, PIDController_AUTOMATIC);
          PIDController_ControllerSetMode(&rightTrackConfig, PIDController_MANUAL);
          caseIndex++;
          break;
        /**
         * And drive for leftSpinCountThreshold to execute the first
         * half of the spin
         */
        case 1:
          if((leftEncoderCount > (leftSpinStartCount + leftSpinCountThreshold)))
            caseIndex++;
          break;
        /**
         * Then the second half of the spin will set the robot to drive forward
         * but only enable the drive on one wheel.
         */
        case 2:
          PIDController_Stop();
          PIDController_SetDirection(PIDController_FORWARD);
          PIDController_ControllerSetMode(&leftTrackConfig, PIDController_MANUAL);
          PIDController_ControllerSetMode(&rightTrackConfig, PIDController_AUTOMATIC);
          caseIndex++;
          break;
        /**
         * And drive for rightSpinCountThreshold to execute the second
         * half of the spin
         */
        case 3:
          if((rightEncoderCount > (rightSpinStartCount + rightSpinCountThreshold)))
            caseIndex++;
          break;
        /**
         * After the spin is completed resume driving forward
         */
        case 4:
          PIDController_Stop();
          PIDController_SetMode(PIDController_NORMALMODE);
          PIDController_SetDirection(PIDController_FORWARD);
          PIDController_Start();
          caseIndex = 0;
          spinning = 0;
      }
    }

    osDelay(SampleTime);
  }
}

/**
 * Stop everything!
 */
void PIDController_Stop()
{
  /**
   * Disable the PIDController
   */
  PIDController_ControllerSetMode(&leftTrackConfig, PIDController_MANUAL);
  PIDController_ControllerSetMode(&rightTrackConfig, PIDController_MANUAL);

  /**
   * And stop the motors
   */
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);

  /**
   * Also reset the PIDController since it will be starting from a stop
   */
  PIDController_ControllerReset(&leftTrackConfig);
  PIDController_ControllerReset(&rightTrackConfig);
}

/**
 * Enable the PIDController on both tracks
 */
void PIDController_Start()
{
  PIDController_ControllerSetMode(&leftTrackConfig, PIDController_AUTOMATIC);
  PIDController_ControllerSetMode(&rightTrackConfig, PIDController_AUTOMATIC);
}

/**
 * The PIDController can function in Forward Mode or Spin Mode
 * This function sets the mode
 */
void PIDController_SetMode(uint8_t newMode)
{
  PIDController_CurrentMode = newMode;
}

/**
 * This function sets the direction of the one track that can go both
 * forward and backward by changing polarity the motor direction lines
 */
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

/******************
 * These are the actual PID methods
 ******************/

/**
 * When resetting the PIDController (from a stop or starting for the first
 * time) we need to reset all the variables for the controller
 */
void PIDController_ControllerReset(PIDController_Config *controllerConfig)
{
  controllerConfig->Ki = 1000;
  controllerConfig->Kp = 400;
  controllerConfig->Kd = 50;
  controllerConfig->Input = 0;
  controllerConfig->Output = 0;
  controllerConfig->Goal = 2;
  controllerConfig->OutMin = 0;
  controllerConfig->OutMax = 65535;
  controllerConfig->Last = 0;
  controllerConfig->LastInput = 0;
  controllerConfig->ITerm = 0;
  controllerConfig->InAuto = PIDController_MANUAL;
}

/*********************************************************************************
 ******* Completely based on https://github.com/br3ttb/Arduino-PID-Library *******
 *********************************************************************************/

/**
 * The meat of the PIDController actually computes the error and updates
 * the output variable
 */
void PIDController_ControllerCompute(PIDController_Config* pidControllerConfig)
{
  /**
   * If we are in manual mode then just move along
   */
  if(!(pidControllerConfig->InAuto)) return;

  /**
   * The task will make sure this is run every SampleTime(ms)
   * no need to do it here
   */

  /**
   * Compute all the working error variables
   */
  volatile float input = pidControllerConfig->Input;
  volatile float error = pidControllerConfig->Goal - input;
  pidControllerConfig->ITerm += (pidControllerConfig->Ki * error);

  /**
   * Make sure the ITerm is within the OutMin and OutMax limits
   */
  if(pidControllerConfig->ITerm > pidControllerConfig->OutMax)
  {
    pidControllerConfig->ITerm= pidControllerConfig->OutMax;
  }
  else if(pidControllerConfig->ITerm < pidControllerConfig->OutMin)
  {
    pidControllerConfig->ITerm = pidControllerConfig->OutMin;
  }

  volatile float dInput = (input - pidControllerConfig->LastInput);

  /**
   * Compute PID Output
   */
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

  /**
   * Remember some variables for next time
   */
  pidControllerConfig->LastInput = input;
}

/**
 * The PIDController needs to be able to be turned off (MANUAL) or on (AUTOMATIC)
 */
void PIDController_ControllerSetMode(PIDController_Config* pidControllerConfig, int Mode)
{
  uint8_t newAuto = (Mode == PIDController_AUTOMATIC);

  if(newAuto == !pidControllerConfig->InAuto)
  {
    /**
     * we just went from manual to auto
     */
    PIDController_ControllerReInitialize(pidControllerConfig);
  }
  pidControllerConfig->InAuto = newAuto;
}
