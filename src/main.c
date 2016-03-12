/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId stepperTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void const * argument);
void StartStepperTask(void const * argument);
void InitPins(void);

int main(void)
{
    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    InitPins();

    /* add mutexes, ... */

    /* add semaphores, ... */

    /* start timers, add new ones, ... */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    osThreadDef(stepperTask, StartStepperTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
    stepperTaskHandle = osThreadCreate(osThread(stepperTask), NULL);

    /* add queues, ... */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    while (1) {
    }
}

/**
 * System Clock Configuration
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
    /* Infinite loop */
    for (;;) {
        GPIOC->ODR = (1 << 13);
        osDelay(500);
        GPIOC->ODR = 0;
        osDelay(500);
    }

}

void StartStepperTask(void const * argument)
{
    int stage = 0;

    for (;;) {
        switch (stage) {
            case 0:
                // Set 12 and reset the other pins
                GPIOB->ODR = (1 << 12);
                break;
            case 1:
                // Set 13 and reset the other pins
                GPIOB->ODR = (1 << 13);
                break;
            case 2:
                // Set 14 and reset the other pins
                GPIOB->ODR = (1 << 14);
                break;
            case 3:
                // Set 15 and reset the other pins
                GPIOB->ODR = (1 << 15);
                break;
        }

        // Go to the next stage
        stage++;

        // And reset it if it gets too high
        if (stage > 3) {
            stage = 0;
        }

        osDelay(10);
    }
}

void InitPins(void)
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

    // Enable the GPIOB clock
    __GPIOB_CLK_ENABLE();

    // Pin B12, B13, B14, B15 as output for stepper driving
    GPIO_InitTypeDef GPIOB_InitStruct;
    GPIOB_InitStruct.Pin =
    GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIOB_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIOB_InitStruct.Pull = GPIO_NOPULL;
    GPIOB_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIOB_InitStruct);
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
