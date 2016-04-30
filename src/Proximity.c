#include "Proximity.h"

//TUNNING VARIABLES//////////////////////////////////////////////////////////////////////
uint32_t ADC_Threshold = 1500;///////////////////////////////////////////////////////////
uint32_t UltraSonic_Threshold = 15;
/////////////////////////////////////////////////////////////////////////////////////////

osThreadId proximityTaskHandle;
uint32_t UltraSonic_Enabled = 1;

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
    if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
    {
      adcValue = HAL_ADC_GetValue(&hadc1);
      if(adcValue > ADC_Threshold && UltraSonic_Enabled && !spinning)
      {
        UltraSonic_Enabled = 0;
        PIDController_Stop();
        osDelay(1000);

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = Ultrasonic_1_Echo_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(Ultrasonic_1_Echo_GPIO_Port, &GPIO_InitStruct);

        HAL_GPIO_WritePin(Ultrasonic_1_Echo_GPIO_Port, Ultrasonic_1_Echo_Pin, GPIO_PIN_SET);
        osDelay(1);
        HAL_GPIO_WritePin(Ultrasonic_1_Echo_GPIO_Port, Ultrasonic_1_Echo_Pin, GPIO_PIN_RESET);

        HAL_TIM_Base_MspInit(&htim15);

      }
      else if(adcValue < (ADC_Threshold - 300) && !spinning)
      {
        UltraSonic_Enabled = 1;
        //Servo_SetPosition(0);
      }
    }
    osDelay(50);
  }
}

void Proximity_Register(void)
{
  osThreadDef(proximityTask, Proximity_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  proximityTaskHandle = osThreadCreate(osThread(proximityTask), NULL);
}

void ProcessUltrasonic(TIM_HandleTypeDef *htim)
{
  volatile uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

  // found a stair
  if(captureValue > UltraSonic_Threshold)
  {
    Servo_SetPosition(90);
    PIDController_Start();
    spinning = 1; ///DON'T LEAVE THIS
    return;
  }

  // Tell the PID Controller to execute a spin
  PIDController_SetMode(PIDController_SPINMODE);
  spinning = 1;
}
