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

// This function is actually from stm32f3xx_hal_msp but it gets rid of a warning
// if it is included here
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void PIDController_MX_TIM1_Init(void);
void PIDController_MX_TIM2_Init(void);
void PIDController_MX_TIM3_Init(void);
void PIDController_ControllerInit(float* Input, float* Output, float* Setpoint,
    float Kp, float Ki, float Kd);
void PIDController_ControllerCompute(void);
void PIDController_ControllerSetTunings(float Kp, float Ki, float Kd);
void PIDController_ControllerSetMode(int Mode);
void PIDController_ControllerReInitialize(void);
void PIDController_Init(void);
void PIDController_Register(void);
void PIDController_Task(void const * argument);


#endif /* PIDCONTROLLER_H_ */
