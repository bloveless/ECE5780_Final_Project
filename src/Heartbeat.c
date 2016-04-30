/*
 * Heartbeat.c
 *
 *  Created on: Apr 2, 2016
 *      Author: brennon
 */

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "Heartbeat.h"

osThreadId heartbeatTaskHandle;

/**
 * The heartbeat has no initialization since the LED is
 * configured in the gpio.c file
 */
void Heartbeat_Init(void)
{

}

/**
 * Register the heartbeat task with FreeRTOS
 */
void Heartbeat_Register(void)
{
  osThreadDef(heartbeatTask, Heartbeat_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);
}

/**
 * Heartbeat will beat every second for us to see if the
 * MCU is working correctly
 */
void Heartbeat_Task(void const * argument)
{
  while(1)
  {
    HAL_GPIO_TogglePin(Heartbeat_LED_GPIO_Port, Heartbeat_LED_Pin);
    osDelay(500);
  }
}
