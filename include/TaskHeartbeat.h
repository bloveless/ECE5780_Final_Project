/*
 * TaskHeartbeat.h
 *
 *  Created on: Mar 16, 2016
 *      Author: brennon
 */

#ifndef TASKHEARTBEAT_H_
#define TASKHEARTBEAT_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
osThreadId heartbeatTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void TaskHeartbeat_Init();
void TaskHeartbeat_Register();
void TaskHeartbeat_Task(void const * argument);

#endif /* TASKHEARTBEAT_H_ */
