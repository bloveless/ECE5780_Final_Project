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

void Heartbeat_Init(void)
{

}

void Heartbeat_Register(void)
{
  osThreadDef(heartbeatTask, Heartbeat_Task, osPriorityNormal, 0, 128);
  heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);
}

void Heartbeat_Task(void const * argument)
{
  while(1)
  {
    HAL_GPIO_TogglePin(Heartbeat_LED_GPIO_Port, Heartbeat_LED_Pin);
    osDelay(500);
  }
}
