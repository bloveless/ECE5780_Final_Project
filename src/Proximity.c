#include "Proximity.h"

//TUNNING VARIABLES//////////////////////////////////////////////////////////////////////
uint32_t ADC_Threshold = 2100;///////////////////////////////////////////////////////////
uint32_t UltraSonic_Threshold = 15;
/////////////////////////////////////////////////////////////////////////////////////////

osThreadId proximityTaskHandle;
uint32_t UltraSonic_Enabled = 1;

/*
 * Start the ADC in continuous mode and enable the TIM15 Input Capture interupt.
 */
void Proximity_Init(void)
{
  spinning = 0;
  HAL_ADC_Start(&hadc1);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
}

void Proximity_Task(void const * argument)
{
  volatile uint32_t adcValue = 0;
  uint32_t UltraSonic_Enabled = 1;

  for(;;)
  {
	  /*
	   * Loop forever and keep reading ADC value (IR censor distance)
	   */
    if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
    {
      adcValue = HAL_ADC_GetValue(&hadc1);
      /*
       * If the value in above a threshold (close enough to a wall) we fire the ultrasonic
       */
      if(adcValue > ADC_Threshold && UltraSonic_Enabled && !spinning)
      {
        UltraSonic_Enabled = 0;
        PIDController_Stop();	//Stop once we've seen a wall
        osDelay(1000);			//Wait a second for robot to come to a complete stop

        //Switch the ultrasonic pin to an output so we can send a trigger pulse.
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = Ultrasonic_1_Echo_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Ultrasonic_1_Echo_GPIO_Port, &GPIO_InitStruct);

        //Send a trigger pulse of 1ms
        HAL_GPIO_WritePin(Ultrasonic_1_Echo_GPIO_Port, Ultrasonic_1_Echo_Pin, GPIO_PIN_SET);
        osDelay(1);
        HAL_GPIO_WritePin(Ultrasonic_1_Echo_GPIO_Port, Ultrasonic_1_Echo_Pin, GPIO_PIN_RESET);

        //Set pin back to input capture mode to read the echo back from the ultrasonic
        HAL_TIM_Base_MspInit(&htim15);

      }
      //ADC is disabled after it senses a wall, until the robot stops spinning and
      //a wall isn't in front of the robot anymore
      else if(adcValue < (ADC_Threshold - 300) && !spinning)
      {
        UltraSonic_Enabled = 1;
        Servo_SetPosition(0);
      }
    }
    osDelay(50);
  }
}

/*
 * Regester the task for FreeRTOS
 */
void Proximity_Register(void)
{
  osThreadDef(proximityTask, Proximity_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  proximityTaskHandle = osThreadCreate(osThread(proximityTask), NULL);
}

/*
 * Read ultrasonic echo value and determine if we have a stair or a wall
 */
void ProcessUltrasonic(TIM_HandleTypeDef *htim)
{
  volatile uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

  // found a stair
  if(captureValue > UltraSonic_Threshold)
  {
    Servo_SetPosition(75);	//Put out arm and
    PIDController_Start();	//start climbing
    spinning = 1;			//Disables certain actions while in spin state
    return;					//spinning is confusingly named, as it is any state
    						//where robot isn't looking for an obstacle in front of it.
  }

  // Tell the PID Controller to execute a spin
  PIDController_SetMode(PIDController_SPINMODE);
  spinning = 1;
}
