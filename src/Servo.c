/*
 * Servo.c
 *
 *  Created on: Apr 3, 2016
 *      Author: brennon
 */

#include "Servo.h"

void Servo_Init()
{
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
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
  // Set the servo to it's natural position
  // this should generate a pulse width of 1.5ms
  Servo_SetPosition(100);

  while(1)
  {
    // servoPosition += 50;
    // __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, servoPosition);
    osDelay(100);
  }
}
