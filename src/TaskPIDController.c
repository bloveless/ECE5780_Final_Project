#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "TaskPIDController.h"

void TaskPIDController_Init()
{

}

void TaskPIDController_Register()
{
    osThreadDef(pidControllerTask, TaskPIDController_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    pidControllerTaskHandle = osThreadCreate(osThread(pidControllerTask), NULL);
}

void TaskPIDController_Task(void const * argument)
{
    for (;;) {
    }
}
