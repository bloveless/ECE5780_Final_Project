#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
//#include "Servo.h"
//#include "PIDController.h"

//TUNNING VARIABLES//////////////////////////////////////////////////////////////////////
uint32_t ADC_Threshold = 2750;///////////////////////////////////////////////////////////
uint32_t UltraSonic_Threshold = 10;
/////////////////////////////////////////////////////////////////////////////////////////

osThreadId proximityTaskHandle;
TIM_HandleTypeDef htim15;
ADC_HandleTypeDef hadc1;

void Proximity_Init(void)
{
  //TIM15 Init
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim15);

  HAL_TIM_IC_Init(&htim15);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);

  //ADC1 Init
  ADC_ChannelConfTypeDef sConfig;

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Start(&hadc1);
}

void Proximity_Task(void const * argument)
{
  uint32_t adcValue = 0;
  uint32_t UltraSonic_Enabled = 1;
  for(;;)
  {
    if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
    {
      adcValue = HAL_ADC_GetValue(&hadc1);
      if(adcValue > ADC_Threshold && UltraSonic_Enabled)
      {
        UltraSonic_Enabled = 0;
        HAL_GPIO_WritePin(GPIOB, Ultrasonic_1_Pulse_Pin, GPIO_PIN_SET);
        osDelay(1);
        HAL_GPIO_WritePin(GPIOB, Ultrasonic_1_Pulse_Pin, GPIO_PIN_RESET);
        //PIDController_Stop();
      }
      else if(adcValue < 2000)
        UltraSonic_Enabled = 1;
    }
    osDelay(1);
  }
}

void Proximity_Register(void)
{
  osThreadDef(proximityTask, Proximity_Task, osPriorityNormal, 0, 128);
  proximityTaskHandle = osThreadCreate(osThread(proximityTask), NULL);
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim15);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  volatile uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
  if(captureValue > UltraSonic_Threshold)
  {
    //Servo_SetPosition(75);
  }
}
