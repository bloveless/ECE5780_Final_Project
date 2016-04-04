/*
 * Servo.c
 *
 *  Created on: Apr 3, 2016
 *      Author: brennon
 */

#include "Servo.h"

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim16.Instance = TIM16;
  // Get the timer down to 1Mhz
  htim16.Init.Prescaler = 8;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Get the timer down to 20Hz
  htim16.Init.Period = 15000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

  HAL_TIM_PWM_Init(&htim16);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(&htim16);

}

void Servo_Init()
{
  MX_TIM16_Init();
}

void Servo_Register()
{
  osThreadDef(servoTask, Servo_Task, osPriorityNormal, 0, 128);
  Servo_TaskHandle = osThreadCreate(osThread(servoTask), NULL);
}

void Servo_Task()
{
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  // Arms under carriage @ 550
  // Arms completely retracted @ 1000
  // Arms completely extended @ 2300
  uint32_t servoPosition = 1000;

  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, servoPosition);

  while(1)
  {
    // __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, servoPosition);
    osDelay(100);
  }
}
