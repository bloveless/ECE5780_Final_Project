#include "Proximity.h"

//TUNNING VARIABLES//////////////////////////////////////////////////////////////////////
uint32_t ADC_Threshold = 1500;///////////////////////////////////////////////////////////
uint32_t UltraSonic_Threshold = 10;
/////////////////////////////////////////////////////////////////////////////////////////

osThreadId proximityTaskHandle;
TIM_HandleTypeDef htim15;
ADC_HandleTypeDef hadc1;
uint32_t UltraSonic_Enabled = 1;

void Proximity_Init(void)
{
  HAL_ADC_Start(&hadc1);
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
      if(adcValue > ADC_Threshold && UltraSonic_Enabled)
      {
        UltraSonic_Enabled = 0;
        PIDController_Stop();
        osDelay(5000);
        HAL_GPIO_WritePin(GPIOB, Ultrasonic_1_Pulse_Pin, GPIO_PIN_SET);
        osDelay(1);
        HAL_GPIO_WritePin(GPIOB, Ultrasonic_1_Pulse_Pin, GPIO_PIN_RESET);
      }
      else if(adcValue < 750)
      {
        UltraSonic_Enabled = 1;
        Servo_SetPosition(0);
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  volatile uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

  // found a stair
  if(captureValue > UltraSonic_Threshold)
  {
    Servo_SetPosition(50);
    return;
  }

  // found a wall so turn around
  PIDController_SetDirection(PIDController_SPINRIGHT);
  PIDController_Start();
}
