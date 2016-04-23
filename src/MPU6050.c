/*
 * MPU6050.c
 *
 *  Created on: Apr 9, 2016
 *      Author: mikemp
 */

#include "MPU6050.h"

#define MPU6050_ADDRESS 0xD0
#define I2C_TIMEOUT			(uint32_t)(50)
// #define TRACE_I2C_DEBUG

uint8_t tx_data[2];
uint8_t rx_data[14];


void MPU6050_Init(){
  rData.AccelX = 0;
  rData.AccelY = 0;
  rData.AccelZ = 0;
  rData.GyroX = 0;
  rData.GyroY = 0;
  rData.GyroZ = 0;
  rData.Temperature = 25;

  rDataHist[0] = rData;
  rDataHist[1] = rData;
  rDataHist[2] = rData;
  rDataHist[3] = rData;
  rDataHist[5] = rData;


	#if defined(TRACE_I2C_DEBUG)
		trace_puts("Trace Debugging Enabled");
	#endif

	volatile HAL_StatusTypeDef lastStatus;

	lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 1, I2C_TIMEOUT);
	if(lastStatus == HAL_TIMEOUT){
		MPU6050_Reset();
		lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 1, I2C_TIMEOUT);
	}
	if(lastStatus == HAL_OK) {
		tx_data[0] = 0x6B;
		tx_data[1] = 1<<7;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(100);

		tx_data[0] = 0x6B;
		tx_data[1] = 0x00;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(100);

		// SET Range +-2G
		tx_data[0] = 0x1C;
		tx_data[1] = 0x00;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);

		// GYRO config
		tx_data[0] = 0x1B;
		tx_data[1] = 0x00;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);

		tx_data[0] = 0x68;
		tx_data[1] = 0x07;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
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
		if((rData.AccelX) >= 17000){
		  HAL_GPIO_WritePin(MPU6050_Status_LED_GPIO_Port,MPU6050_Status_LED_Pin,GPIO_PIN_SET);
		}
		else{
		  HAL_GPIO_WritePin(MPU6050_Status_LED_GPIO_Port,MPU6050_Status_LED_Pin,GPIO_PIN_RESET);
		}
//		trace_printf("G: %d\t\tA: %d\t\tT: %d\n", (&rData)->GyroX/16, (&rData)->AccelX, (&rData)->Temperature);
//		trace_printf("GX:\t%d\n",(&rData)->GyroX/16);
//		trace_printf("AX:\t%d\n",(&rData)->AccelX/131);
//		trace_printf("TM:\t%d\r", (&rData)->Temperature);
		osDelay(100);
	}
}


void MPU6050_Reset(){
	tx_data[0] = 0x00;
	tx_data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
}

void MPU6050_Read(MPU6050_TypeDef *rData){
	tx_data[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 1, I2C_TIMEOUT);
	if( HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS, &rx_data[0], 14, I2C_TIMEOUT) == HAL_OK) {
		int16_t tempRaw = ((int16_t)rx_data[6] << 8) | rx_data[7];
		rData->Temperature = (int16_t)(((float)tempRaw)/340+36.53);
		if(rData->Temperature <= 30 && rData->Temperature >= 19){
//		  // Shift History
//		  rDataHist[4] = rDataHist[3];
//		  rDataHist[3] = rDataHist[2];
//		  rDataHist[2] = rDataHist[1];
//		  rDataHist[1] = rDataHist[0];
//		  rDataHist[0] = *rData;

		  //Calculate rData values
			rData->AccelX = ((int16_t)rx_data[0] << 8) | rx_data[1];
			rData->AccelY      = ((int16_t)rx_data[2] << 8) | rx_data[3];
			rData->AccelZ      = ((int16_t)rx_data[4] << 8) | rx_data[5];
			rData->Temperature = (int16_t)(((float)tempRaw)/340+36.53);
			rData->GyroX       = ((int16_t)rx_data[8] << 8) | rx_data[9];
			rData->GyroY       = ((int16_t)rx_data[10] << 8) | rx_data[11];
			rData->GyroZ       = ((int16_t)rx_data[12] << 8) | rx_data[13];

//			//Average rData values
////			rData->AccelX = (int16_t)( ((int32_t)rData->AccelX + (int32_t)rDataHist[0].AccelX + (int32_t)rDataHist[1].AccelX +
////			    (int32_t)rDataHist[2].AccelX + (int32_t)rDataHist[3].AccelX + (int32_t)rDataHist[4].AccelX)/6 );
//			rData->AccelX = (int16_t)( ((int32_t)rData->AccelX + (int32_t)rDataHist[0].AccelX + (int32_t)rDataHist[1].AccelX +
//			    (int32_t)rDataHist[2].AccelX)/4 );
//			rData->AccelY = (int16_t)( ((int32_t)rData->AccelY + (int32_t)rDataHist[0].AccelY + (int32_t)rDataHist[1].AccelY +
//          (int32_t)rDataHist[2].AccelY + (int32_t)rDataHist[3].AccelY + (int32_t)rDataHist[4].AccelY)/6 );
//			rData->AccelZ      = ((int16_t)rx_data[4] << 8) | rx_data[5];
//			rData->Temperature = (int16_t)(((float)tempRaw)/340+36.53);
//			rData->GyroX       = ((int16_t)rx_data[8] << 8) | rx_data[9];
//			rData->GyroY       = ((int16_t)rx_data[10] << 8) | rx_data[11];
//			rData->GyroZ       = ((int16_t)rx_data[12] << 8) | rx_data[13];

		}
		else {
			MPU6050_Reset();
			MPU6050_Init();
		}

	#if defined(TRACE_I2C_DEBUG)
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
