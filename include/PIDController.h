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
#include "tim.h"
#include "Proximity.h"

#define PIDController_AUTOMATIC 1
#define PIDController_MANUAL 0

#define PIDController_STOP 1
#define PIDController_FORWARD 2
#define PIDController_SPIN 3

#define PIDController_NORMALMODE 0
#define PIDController_SPINMODE 1

uint8_t PIDController_CurrentMode;

// Default sample time is .1 seconds
// This also controls the delay in the task
uint8_t SampleTime;

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
osThreadId PIDController_pidControllerTaskHandle;

uint32_t PIDController_leftEncoderCount;
uint32_t PIDController_rightEncoderCount;

PIDController_Config leftTrackConfig;
PIDController_Config rightTrackConfig;

// Default sample time is .1 seconds
// This also controls the delay in the task
uint8_t SampleTime;

// This function is actually from stm32f3xx_hal_msp but it gets rid of a warning
// if it is included here
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

void PIDController_Init(void);
void PIDController_Register(void);
void PIDController_Task(void const * argument);
void PIDController_Stop();
void PIDController_Start();
void PIDController_SetMode(uint8_t newMode);
void PIDController_NormalMode();
void PIDController_SpinMode();
void PIDController_SetDirection(int direction);

/******************
 * These are the actual PID methods
 ******************/
void PIDController_ControllerReset(PIDController_Config *controllerConfig);
void PIDController_ControllerCompute(PIDController_Config* pidControllerConfig);
void PIDController_ControllerSetMode(PIDController_Config* pidControllerConfig, int Mode);
void PIDController_ControllerReInitialize(PIDController_Config* pidControllerConfig);


#endif /* PIDCONTROLLER_H_ */
