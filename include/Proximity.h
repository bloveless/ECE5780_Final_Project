#ifndef PROXIMITY_H
#define PROXIMITY_H

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "Servo.h"
#include "PIDController.h"

void Proximity_Init(void);
void Proximity_Register(void);
void Proximity_Task(void const * argument);

#endif PROXIMITY_H
