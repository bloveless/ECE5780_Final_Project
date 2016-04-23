/*
 * MPU6050.h
 *
 *  Created on: Apr 9, 2016
 *      Author: mikemp
 */
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "diag/Trace.h"
#include "i2c.h"

#ifndef MPU6050_H_
#define MPU6050_H_


typedef struct
{
	int16_t AccelX;
	int16_t AccelY;
	int16_t AccelZ;
	int16_t Temperature;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;

} MPU6050_TypeDef;

MPU6050_TypeDef rData;
MPU6050_TypeDef rDataHist[5];
osThreadId mpu6050TaskHandle;


void MPU6050_Init();

void MPU6050_Task();

void MPU6050_Reg();

void MPU6050_Reset();

void MPU6050_Read(MPU6050_TypeDef *rData);

#endif /* MPU6050_H_ */
