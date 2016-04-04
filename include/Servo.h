/*
 * Servo.h
 *
 *  Created on: Apr 3, 2016
 *      Author: brennon
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

TIM_HandleTypeDef htim16;
osThreadId Servo_TaskHandle;

// This function is actually from stm32f3xx_hal_msp but it gets rid of a warning
// if it is included here
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void MX_TIM16_Init(void);
void Servo_Init(void);
void Servo_Register(void);
void Servo_Task(void);

#endif /* SERVO_H_ */
