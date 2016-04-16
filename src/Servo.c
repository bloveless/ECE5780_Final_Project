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
  osThreadDef(servoTask, Servo_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  Servo_TaskHandle = osThreadCreate(osThread(servoTask), NULL);
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Position should be between 0 - 100
 * 50 indicates the servos natural position
 */
void Servo_SetPosition(uint8_t position)
{
  // Map the position to a valid percentage
  uint16_t scaledPosition = map(position, 0, 100, 5, 14);

  // Convert the position to a pwmSignal
  uint16_t pwmPosition = (uint16_t) (htim16.Init.Period * (((float) scaledPosition) / 100));
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwmPosition);
}

void Servo_Task()
{
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  // Set the servo to it's natural position
  // this should generate a pulse width of 1.5ms
  // Servo_SetPosition(0);

  while(1)
  {
    // servoPosition += 50;
    // __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, servoPosition);
    osDelay(100);
  }
}
