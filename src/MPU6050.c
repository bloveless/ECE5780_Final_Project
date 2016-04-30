/*
 * MPU6050.c
 *
 *  Created on: Apr 9, 2016
 *      Author: mikemp
 */

#include "MPU6050.h"

uint8_t tx_data[2];
uint8_t rx_data[15];
uint8_t looking_down;

/**
 * Send the appropriate bytes to configure the MPU6050
 */
void MPU6050_Init(){

	#if defined(TRACE_I2C_DEBUG)
		trace_puts("Trace Debugging Enabled");
	#endif

	HAL_StatusTypeDef lastStatus;

	/**
	 * Check that a device is ready at the MPU6050's address
	 */
	HAL_Delay(250);
	lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 1, I2C_TIMEOUT);
	HAL_Delay(250);

	/**
	 * If a device was not ready then the MPU might be in a stalled state
	 * so attempt to reset it
	 */
	if(lastStatus == HAL_TIMEOUT){
		MPU6050_Reset();
		lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 1, I2C_TIMEOUT);
		HAL_Delay(250);
	}

	/**
	 * If the MPU6050 responded correctly then configure it
	 */
	if(lastStatus == HAL_OK) {
	  /**
	   * Reset the device so we start in a standard state
	   */
		tx_data[0] = MPU6050_PWR_MGMT_1;
		tx_data[1] = MPU6050_PWR_MGMT_1_DEVICE_RESET;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(250);

		/**
		 * Use the internal 8MHz oscillator
		 */
		tx_data[0] = MPU6050_PWR_MGMT_1;
		tx_data[1] = MPU6050_PWR_MGMT_1_CLKSEL_0;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(250);

		/**
		 * Configure the accelerometer with a +-2G Range
		 */
		tx_data[0] = MPU6050_ACCEL_CONFIG;
		tx_data[1] = MPU6050_ACCEL_CONFIG_AFS_SEL_2G;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(250);

		/**
		 * Configure the GYRO with a +-250 Range
		 */
		tx_data[0] = MPU6050_GYRO_CONFIG;
		tx_data[1] = MPU6050_GYRO_CONFIG_FS_SEL_250;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(250);

		/**
		 * Enable the on-chip Digital Low Pass Filter
		 * Rolling average of 6 measurements
		 */
		tx_data[0] = MPU6050_CONFIG;
		tx_data[1] = MPU6050_CONFIG_DLPF_CFG_6;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
		HAL_Delay(250);

		/**
		 * Signal conditioning reset
		 * Sets the internal buffers to zero and clears the internal ADC
		 */
		tx_data[0] = MPU6050_USER_CTRL;
		tx_data[1] = MPU6050_USER_CTRL_SIG_COND_RESET;
		lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
	  HAL_Delay(250);

	  /**
	   * Verify that the MPU6050 is still ready
	   */
	  lastStatus = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 1, I2C_TIMEOUT);
	}
}

/**
 * Register the MPU6050 Task
 */
void MPU6050_Reg(){
	  osThreadDef(mpu6050Task, MPU6050_Task, osPriorityNormal, 0, 128);
	  mpu6050TaskHandle = osThreadCreate(osThread(mpu6050Task), NULL);
}

/**
 * Check check the angle of the robot and set the Servo/PIDController appropriately
 */
void MPU6050_Task(){

  uint8_t index=0;

	while(1) {
	  /**
	   * Read the data from the MPU6050
	   */
		MPU6050_Read(&rData);

		/**
		 * The threshold for when the robot is flat
		 * This is used to determine and upward and downward angle
		 */
		int16_t upperBound = 2000;
		int16_t lowerBound = -2000;

		/**
		 * This process is used to reset the arms after it has climbed over the stair
		 *
		 * The robot needs to face upward to "arm" this process then when the robot
		 * faces downward it will reset the outer track to their starting position
		 */
		switch (index) {
		  /**
		   * Wait until the robot faces upward and "arm"
		   */
		  case 0:
		    if((rData.AccelZ) >= upperBound){
		      index++;
		    }
		    break;
      /**
       * Wait until the robot faces downward and "fire"
       */
		  case 1:
		    if((rData.AccelZ) <= upperBound){
		      index++;
		    }
		    break;
		  /**
		   * Reset the robot to it's default state and resume executing
		   */
		  case 2:
		    osDelay(40);
	      Servo_SetPosition(0);
	      PIDController_Stop();
	      spinning = 0;
	      osDelay(1000);
	      PIDController_Start();
		    index = 0;
		  break;
		}

		/**
		 * Debugging LED turns off when robot is flat and turns on when robot it
		 * outside flat threshold
		 */
		if((rData.AccelZ >= lowerBound) && (rData.AccelZ <= upperBound)){
		  HAL_GPIO_WritePin(MPU6050_Status_LED_GPIO_Port,MPU6050_Status_LED_Pin,GPIO_PIN_RESET);
		}
		else{
		  HAL_GPIO_WritePin(MPU6050_Status_LED_GPIO_Port,MPU6050_Status_LED_Pin,GPIO_PIN_SET);
		}

//		trace_printf("G: %d\t\tA: %d\t\tT: %d\n", (&rData)->GyroX/16, (&rData)->AccelX, (&rData)->Temperature);
		osDelay(10);
	}
}

/**
 * If we detect that the MPU6050 is in a poor state we will try and reset it
 * by writing two 0 bytes (toggling the clock line 16 times) and then
 * resetting the MPU6050
 */
void MPU6050_Reset(){
  /**
   * Toggle the clock line 16 times
   */
	tx_data[0] = 0x00;
	tx_data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);

	/**
	 * Reset the MPU
	 */
	tx_data[0] = MPU6050_SIGNAL_PATH_RESET;
	tx_data[1] = MPU6050_SIGNAL_PATH_RESET_ALL_RESET;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 2, I2C_TIMEOUT);
}

/**
 * Read data from the MPU
 */
void MPU6050_Read(MPU6050_TypeDef *rData){

  /**
   * The starting register for the accelerometer
   */
	tx_data[0] = MPU6050_ACCEL_XOUT_H;

	HAL_StatusTypeDef lastStatus;

	/**
	 * Set the register to start reading from
	 */
	lastStatus = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, &tx_data[0], 1, I2C_TIMEOUT);
	osDelay(250);

	/**
	 * Then read all 14 register that hold the acceleromter and gyroscope data
	 */
	lastStatus = HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS, &rx_data[0], 14, I2C_TIMEOUT);

	if(lastStatus == HAL_OK) {

	  /**
	   * The temperature is used as data verification
	   */
		int16_t tempRaw = ((int16_t)rx_data[6] << 8) | rx_data[7];
		rData->Temperature = (int16_t)(((float)tempRaw)/340+36.53);
		uint8_t offset = 0;

		/**
		 * If the temperature is obsurd then we need to shift all the registers
		 * that we read by one.
		 *
		 * Not sure why this happens, but when the temperature is obsurd the I2C
		 * module will read a blank byte as the first byte forcing us to
		 * offset every read by one.
		 */
		if(!(rData->Temperature <= 30 && rData->Temperature >= 19)){
			offset = 1;
		}

		/**
		 * Calculate rData values
		 */
    rData->AccelX       = ((int16_t)rx_data[0+offset] << 8) | rx_data[1+offset];
    rData->AccelY       = ((int16_t)rx_data[2+offset] << 8) | rx_data[3+offset];
    rData->AccelZ       = ((int16_t)rx_data[4+offset] << 8) | rx_data[5+offset];
    tempRaw             = ((int16_t)rx_data[6+offset] << 8) | rx_data[7+offset];
    rData->Temperature  = (int16_t)(((float)tempRaw)/340+36.53);
    rData->GyroX        = ((int16_t)rx_data[8+offset] << 8) | rx_data[9+offset];
    rData->GyroY        = ((int16_t)rx_data[10+offset] << 8) | rx_data[11+offset];
    rData->GyroZ        = ((int16_t)rx_data[12+offset] << 8) | rx_data[13+offset];

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
