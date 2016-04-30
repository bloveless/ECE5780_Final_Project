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

/*
 * Initiate variables and start timer with interupt
 */
void Servo_Init()
{
  counter = 0;
  scaledPosition = 5;
  count = 0;

  HAL_TIM_Base_Start_IT(&htim7);
}

/*
 * Register the servo task with FreeRTOS
 */
void Servo_Register()
{
  osThreadDef(servoTask, Servo_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  Servo_TaskHandle = osThreadCreate(osThread(servoTask), NULL);
}

/*
 * Maps a value from 0-100 to 5-25. A 5ms pulse retracts arms and a 25ms pulse extends arms
 */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void Servo_SetPosition(uint8_t position)
{
  // Map the position to a valid percentage
  scaledPosition = map(position, 0, 100, 5, 25);
}

/*
 * Dummy task. All the work is done in the timer interupt.
 */
void Servo_Task()
{
  // Set the servo to it's natural position
  Servo_SetPosition(0);

  while(1)
  {
    osDelay(1000);
  }
}

/*
 * Called from timer interupt. Generates a PWM signal for the servo depending on
 * the value in scaledPosition
 */
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
