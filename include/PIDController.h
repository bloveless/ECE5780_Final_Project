/*
 * PIDController.h
 *
 *  Created on: Apr 2, 2016
 *      Author: brennon
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void PIDController_Init(void);
void PIDController_Register(void);
void PIDController_Task(void const * argument);


#endif /* PIDCONTROLLER_H_ */
