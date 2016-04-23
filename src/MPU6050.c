/*
 * MPU6050.c
 *
 *  Created on: Apr 9, 2016
 *      Author: mikemp
 */

#include "MPU6050.h"

//  MPU6050_Init(&hi2c1);
//  MPU6050_TypeDef MPU6050_DATA;
//  MPU6050_Read(&hi2c1, &MPU6050_DATA);
//  MPU6050_Read(&hi2c1, &MPU6050_DATA);
//  MPU6050_Read(&hi2c1, &MPU6050_DATA);

#define MPU_WRITE_ADDRESS 0xD0
#define MPU_READ_ADDRESS 0xD1

uint8_t tx_data[2];
uint8_t rx_data[14];

void MPU6050_Init(){
	#if defined(TRACE_I2C_DEBUG)
		trace_initialize();
		trace_puts("Trace Debugging Enabled");
	#endif

	volatile HAL_StatusTypeDef lastStatus;

	lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, MPU_READ_ADDRESS, 1, 100);

	if(lastStatus == HAL_OK)
	{
    tx_data[0] = 0x75;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 1, 1000);
    lastStatus = HAL_I2C_Master_Receive(&hi2c1, 0xD0, &rx_data[0], 1, 1000);

    tx_data[0] = 0x6B;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 1, 1000);
    lastStatus = HAL_I2C_Master_Receive(&hi2c1, 0xD0, &rx_data[0], 1, 1000);

    tx_data[0] = 0x6B;
    tx_data[1] = 1<<7;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 2, 1000);
    lastStatus = HAL_I2C_Master_Receive(&hi2c1, 0xD0, &rx_data[0], 1, 1000);
    HAL_Delay(100);

    tx_data[0] = 0x6B;
    tx_data[1] = 0x00;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 2, 1000);
    lastStatus = HAL_I2C_Master_Receive(&hi2c1, 0xD0, &rx_data[0], 1, 1000);
    HAL_Delay(100);
    // SET Range +-2G
    tx_data[0] = 0x1C;
    tx_data[1] = 0x00;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 2, 1000);
    // GYRO config
    tx_data[0] = 0x1B;
    tx_data[1] = 0x00;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 2, 1000);

    tx_data[0] = 0x68;
    tx_data[1] = 0x07;
    lastStatus = HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 2, 1000);
    lastStatus = HAL_I2C_Master_Receive(&hi2c1, 0xD0, &rx_data[0], 1, 1000);
    HAL_Delay(100);
	}
}

void MPU6050_Reg(){
	  osThreadDef(mpu6050Task, MPU6050_Task, osPriorityNormal, 0, 128);
	  mpu6050TaskHandle = osThreadCreate(osThread(mpu6050Task), NULL);
}

void MPU6050_Task(){
	while(1) {
		MPU6050_Read(&rData);
    trace_printf("AccelX:\t\t%d\r", rData.AccelX/16);
    trace_printf("Temperature:\t%d\r", rData.Temperature);
		osDelay(1000);
	}
}

void MPU6050_Read(MPU6050_TypeDef *rData){
	tx_data[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, &tx_data[0], 1, 1000);
	if( HAL_I2C_Master_Receive(&hi2c1, 0xD0, &rx_data[0], 14, 1000) == HAL_OK) {

		rData->AccelX = ((int16_t)rx_data[0] << 8) | rx_data[1];
		rData->AccelY      = ((int16_t)rx_data[2] << 8) | rx_data[3];
		rData->AccelZ      = ((int16_t)rx_data[4] << 8) | rx_data[5];
		int16_t tempRaw = ((int16_t)rx_data[6] << 8) | rx_data[7];
		rData->Temperature = (int16_t)(((float)tempRaw)/340+36.53);
		rData->GyroX       = ((int16_t)rx_data[8] << 8) | rx_data[9];
		rData->GyroY       = ((int16_t)rx_data[10] << 8) | rx_data[11];
		rData->GyroZ       = ((int16_t)rx_data[12] << 8) | rx_data[13];
	#if defined(TRACE_I2C_DEBUG)
			//			trace_printf("AccelX:\t\t%d\r", rData->AccelX);
			//			trace_printf("AccelY:\t\t%d\r", rData->AccelY);
			//			trace_printf("AccelZ:\t\t%d\r", rData->AccelZ);
			//			trace_printf("Temperature:\t%d\r", rData->Temperature);
			//			trace_printf("GyroX:\t\t%d\r", rData->GyroX);
			//			trace_printf("GyroY:\t\t%d\r", rData->GyroY);
			//			trace_printf("GyroZ:\t\t%d\r", rData->GyroZ);
			//			trace_puts("\n");
			trace_printf("AccelX:\t\t%d\r", rData->AccelX/16);
			trace_printf("AccelY:\t\t%d\r", rData->AccelY/16);
			trace_printf("AccelZ:\t\t%d\r", rData->AccelZ/16);
			trace_printf("Temperature:\t%d\r", rData->Temperature);
			trace_printf("GyroX:\t\t%d\r", rData->GyroX/131);
			trace_printf("GyroY:\t\t%d\r", rData->GyroY/131);
			trace_printf("GyroZ:\t\t%d\r", rData->GyroZ/131);
			trace_puts("\n");
	#endif
	}
}
