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

#define MPU6050_ADDRESS 0xD0
#define I2C_TIMEOUT			(uint32_t)(50)

#define MPU6050_PWR_MGMT_1					(uint8_t)(0x6B)
#define MPU6050_PWR_MGMT_1_CLKSEL_0			(uint8_t)(0x00) //Internal 8MHz oscillator
#define MPU6050_PWR_MGMT_1_DEVICE_RESET		(uint8_t)(1<<7)

#define MPU6050_ACCEL_CONFIG				(uint8_t)(0x1C)
#define	MPU6050_ACCEL_CONFIG_AFS_SEL_2G		(uint8_t)(0x00)
#define	MPU6050_ACCEL_CONFIG_XA_ST			(uint8_t)(1<<7)
#define	MPU6050_ACCEL_CONFIG_YA_ST			(uint8_t)(1<<6)
#define	MPU6050_ACCEL_CONFIG_ZA_ST			(uint8_t)(1<<5)

#define MPU6050_GYRO_CONFIG				(uint8_t)(0x1B)
#define MPU6050_GYRO_CONFIG_FS_SEL_250	(uint8_t)(0x00)
#define MPU6050_GYRO_CONFIG_FS_SEL_500	(uint8_t)(0x01<<3)
#define MPU6050_GYRO_CONFIG_FS_SEL_1000	(uint8_t)(0x02<<3)
#define MPU6050_GYRO_CONFIG_FS_SEL_2000	(uint8_t)(0x03<<3)
#define MPU6050_GYRO_CONFIG_XG_ST		(uint8_t)(1<<7)
#define MPU6050_GYRO_CONFIG_YG_ST		(uint8_t)(1<<6)
#define MPU6050_GYRO_CONFIG_ZG_ST		(uint8_t)(1<<5)

#define MPU6050_SIGNAL_PATH_RESET					(uint8_t)(0x68)
#define MPU6050_SIGNAL_PATH_RESET_TEMP_RESET		(uint8_t)(0x01)
#define MPU6050_SIGNAL_PATH_RESET_ACCEL_RESET		(uint8_t)(1<<1)
#define MPU6050_SIGNAL_PATH_RESET_GYRO_RESET		(uint8_t)(1<<2)
#define MPU6050_SIGNAL_PATH_RESET_ALL_RESET			(uint8_t)(0x07)

#define MPU6050_USER_CTRL							(uint8_t)(0x68)
#define MPU6050_USER_CTRL_SIG_COND_RESET			(uint8_t)(0x68)

#define MPU6050_ACCEL_XOUT_H			(uint8_t)(0x3B)

#define MPU6050_CONFIG					(uint8_t)(0x1A)
#define MPU6050_CONFIG_DLPF_CFG_0		(uint8_t)(0x00)
#define MPU6050_CONFIG_DLPF_CFG_1		(uint8_t)(0x01)
#define MPU6050_CONFIG_DLPF_CFG_2		(uint8_t)(0x02)
#define MPU6050_CONFIG_DLPF_CFG_3		(uint8_t)(0x03)
#define MPU6050_CONFIG_DLPF_CFG_4		(uint8_t)(0x04)
#define MPU6050_CONFIG_DLPF_CFG_5		(uint8_t)(0x05)
#define MPU6050_CONFIG_DLPF_CFG_6		(uint8_t)(0x06)


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
