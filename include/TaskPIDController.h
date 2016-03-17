#ifndef TASKPIDCONTROLLER_H_
#define TASKPIDCONTROLLER_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
osThreadId pidControllerTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void TaskPIDController_Init();
void TaskPIDController_Register();
void TaskPIDController_Task(void const * argument);

#endif /* TASKPIDCONTROLLER_H_ */
