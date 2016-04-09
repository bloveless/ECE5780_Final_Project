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

typedef struct {
    uint8_t InAuto;
    float Ki;
    float Kp;
    float Kd;
    float Input;
    float Output;
    float Goal;
    int OutMin;
    int OutMax;
    float Last;
    float LastInput;
    float ITerm;
} PIDController_Config;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_OC_InitTypeDef PIDController_htim3ConfigOC;
osThreadId PIDController_pidControllerTaskHandle;

uint32_t PIDController_leftEncoderCount;
uint32_t PIDController_rightEncoderCount;

PIDController_Config tim1Config;
PIDController_Config tim2Config;

// Default sample time is .1 seconds
// This also controls the delay in the task
uint8_t SampleTime;

// This function is actually from stm32f3xx_hal_msp but it gets rid of a warning
// if it is included here
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void PIDController_MX_TIM1_Init(void);
void PIDController_MX_TIM2_Init(void);
void PIDController_MX_TIM3_Init(void);
void PIDController_ControllerCompute(PIDController_Config* pidControllerConfig);
void PIDController_ControllerUpdateTunings(PIDController_Config* pidControllerConfig);
void PIDController_ControllerSetMode(PIDController_Config* pidControllerConfig, int Mode);
void PIDController_ControllerReInitialize(PIDController_Config* pidControllerConfig);
void PIDController_Init(void);
void PIDController_Register(void);
void PIDController_Task(void const * argument);


#endif /* PIDCONTROLLER_H_ */
