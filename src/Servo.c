/*
 * Servo.c
 *
 *  Created on: Apr 3, 2016
 *      Author: brennon
 */

#include "Servo.h"

uint32_t counter;
uint16_t scaledPosition;
uint32_t count;

void Servo_Init()
{
  counter = 0;
  scaledPosition = 5;
  count = 0;

  HAL_TIM_Base_Start_IT(&htim7);
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
  scaledPosition = map(position, 0, 100, 5, 25);
}

void Servo_Task()
{
  // Set the servo to it's natural position
  // this should generate a pulse width of 1.5ms
  Servo_SetPosition(0);

  while(1)
  {
    // servoPosition += 50;
    // __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, servoPosition);
    osDelay(1000);
  }
}

void Handle_PWM(void)
{
  counter++;
  if(counter == 200)
    counter = 0;
  if(counter < scaledPosition)
    HAL_GPIO_WritePin(Servo_PWM_GPIO_Port, Servo_PWM_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(Servo_PWM_GPIO_Port, Servo_PWM_Pin, GPIO_PIN_RESET);
}
