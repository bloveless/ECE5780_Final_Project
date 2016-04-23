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

///// TODO: Pre-Init
//// pg 15 MPU-6050_DataSheet
////I2C ADDRESS 110100x
////AD0 x = 0
////AD0 x = 1
//#define MPU_6050_ADO 0
//#define MPU_6050_I2C_ADDRESS (0b1101000 | MPU_6050_ADO)
//uint8_t aTxBuffer[] = "Hello World";
//#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)

// #define TRACE_I2C_DEBUG

typedef struct
{
	int16_t AccelX;
	int16_t AccelY;
	int16_t AccelZ;
	int16_t Temperature;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;

}MPU6050_TypeDef;

I2C_HandleTypeDef hi2c1;
MPU6050_TypeDef rData;
osThreadId mpu6050TaskHandle;

void MPU6050_Init();
void MPU6050_Task();
void MPU6050_Reg();
void MPU6050_Read(MPU6050_TypeDef *rData);

#endif /* MPU6050_H_ */
