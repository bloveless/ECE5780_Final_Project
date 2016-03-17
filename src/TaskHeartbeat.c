#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "TaskHeartbeat.h"

void TaskHeartbeat_Init()
{
    // Enable the GPIOC clock
    __GPIOC_CLK_ENABLE();

    // PC13 as an output (on board LED)
    GPIO_InitTypeDef GPIOC_InitStruct;
    GPIOC_InitStruct.Pin = GPIO_PIN_13;
    GPIOC_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOC_InitStruct.Pull = GPIO_NOPULL;
    GPIOC_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIOC_InitStruct);
}

void TaskHeartbeat_Register()
{
    osThreadDef(heartbeatTask, TaskHeartbeat_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);
}

void TaskHeartbeat_Task(void const * argument)
{
    for (;;) {
        GPIOC->ODR = (1 << 13);
        osDelay(500);
        GPIOC->ODR = 0;
        osDelay(500);
    }
}
