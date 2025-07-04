/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_IN1_PORT				GPIOC
#define M_IN1_PIN					GPIO_PIN_12
#define M_IN2_PORT				GPIOC
#define M_IN2_PIN					GPIO_PIN_10
#define M_IN3_PORT				GPIOA
#define M_IN3_PIN					GPIO_PIN_6
#define M_IN4_PORT				GPIOA
#define M_IN4_PIN					GPIO_PIN_7

#define BT_B_SPEED				400
#define BT_M_SPEED				65
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rxData_1 = 0, rxData_2 = 0;

uint8_t mcMode;
uint8_t mtMode_L1 = 0, mtMode_L2 = 0;
uint8_t mtMode_R1 = 0, mtMode_R2 = 0;
uint16_t mtSpeed_L = 0, mtSpeed_R = 0;

volatile uint16_t adcValue[2] = {0};
uint16_t adcX, adcY;

uint8_t ctrMode = 2;

uint8_t btMode = 0, btSpeed = 0;
/* USER CODE END Variables */
/* Definitions for MotorCTR */
osThreadId_t MotorCTRHandle;
const osThreadAttr_t MotorCTR_attributes = {
  .name = "MotorCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for JoystickCTR */
osThreadId_t JoystickCTRHandle;
const osThreadAttr_t JoystickCTR_attributes = {
  .name = "JoystickCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BluetoothCTR */
osThreadId_t BluetoothCTRHandle;
const osThreadAttr_t BluetoothCTR_attributes = {
  .name = "BluetoothCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ModeCTR */
osThreadId_t ModeCTRHandle;
const osThreadAttr_t ModeCTR_attributes = {
  .name = "ModeCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (rxData_1 > 200) {
			btSpeed = 0;
			btMode = 0;
		}
		else if (rxData_1 >= 48 && rxData_1 <= 57) {
			btSpeed = rxData_1 - 48;
		}
		else if (rxData_1 > 64) {
			btMode = rxData_1;
		}
		HAL_UART_Receive_DMA(&huart1, &rxData_1, 1);
	}

	if (huart->Instance == USART2) {
		if (rxData_2 == 'w') {
			mcMode = 1;
			mtSpeed_R = 500;
			mtSpeed_L = 500;
		}
		else if (rxData_2 == 'x') {
			mcMode = 2;
			mtSpeed_R = 500;
			mtSpeed_L = 500;
		}
		else if (rxData_2 == 'a') {
			mcMode = 3;
			mtSpeed_R = 500;
			mtSpeed_L = 500;
		}
		else if (rxData_2 == 'd') {
			mcMode = 4;
			mtSpeed_R = 500;
			mtSpeed_L = 500;
		}
		else if (rxData_2 == 's') {
			mcMode = 0;
			mtSpeed_R = 0;
			mtSpeed_L = 0;
		}
		else if (rxData_2 == 'r') {
			mtSpeed_R += 50;
			if (mtSpeed_R >= 950) {
				mtSpeed_R = 950;
			}
		}
		else if (rxData_2 == 'f') {
			mtSpeed_R -= 50;
			if (mtSpeed_R < 200) {
				mtSpeed_R = 0;
			}
		}
		else if (rxData_2 == 't') {
			mtSpeed_L += 50;
			if (mtSpeed_L >= 950) {
				mtSpeed_L = 950;
			}
		}
		else if (rxData_2 == 'g') {
			mtSpeed_L -= 50;
			if (mtSpeed_L < 200) {
				mtSpeed_L = 0;
			}
		}
		HAL_UART_Receive_DMA(&huart2, &rxData_2, 1);
	}



}
/* USER CODE END FunctionPrototypes */

void MotorCTRTask(void *argument);
void JoystickCTRTask(void *argument);
void BluetoothCTRTask(void *argument);
void ModeCTRTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
HAL_UART_Receive_DMA(&huart1, &rxData_1, 1);
HAL_UART_Receive_DMA(&huart2, &rxData_2, 1);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorCTR */
  MotorCTRHandle = osThreadNew(MotorCTRTask, NULL, &MotorCTR_attributes);

  /* creation of JoystickCTR */
  JoystickCTRHandle = osThreadNew(JoystickCTRTask, NULL, &JoystickCTR_attributes);

  /* creation of BluetoothCTR */
  BluetoothCTRHandle = osThreadNew(BluetoothCTRTask, NULL, &BluetoothCTR_attributes);

  /* creation of ModeCTR */
  ModeCTRHandle = osThreadNew(ModeCTRTask, NULL, &ModeCTR_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MotorCTRTask */
/**
  * @brief  Function implementing the MotorCTR thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotorCTRTask */
void MotorCTRTask(void *argument)
{
  /* USER CODE BEGIN MotorCTRTask */
  /* Infinite loop */
	for(;;) {
		switch (mcMode) {
		case 0:
			mtMode_R1 = 0;
			mtMode_R2 = 0;
			mtMode_L1 = 0;
			mtMode_L2 = 0;
			break;

		case 1:
			mtMode_R1 = 1;
			mtMode_R2 = 0;
			mtMode_L1 = 1;
			mtMode_L2 = 0;
			break;

		case 2:
			mtMode_R1 = 0;
			mtMode_R2 = 1;
			mtMode_L1 = 0;
			mtMode_L2 = 1;
			break;

		case 3:
			mtMode_R1 = 1;
			mtMode_R2 = 0;
			mtMode_L1 = 0;
			mtMode_L2 = 0;
			break;

		case 4:
			mtMode_R1 = 0;
			mtMode_R2 = 0;
			mtMode_L1 = 1;
			mtMode_L2 = 0;
			break;
		}

		HAL_GPIO_WritePin(M_IN2_PORT, M_IN2_PIN, mtMode_R1);
		HAL_GPIO_WritePin(M_IN1_PORT, M_IN1_PIN, mtMode_R2);
		HAL_GPIO_WritePin(M_IN3_PORT, M_IN3_PIN, mtMode_L1);
		HAL_GPIO_WritePin(M_IN4_PORT, M_IN4_PIN, mtMode_L2);

		TIM4->CCR2 = mtSpeed_R;
		TIM4->CCR1 = mtSpeed_L;

		osDelay(10);
	}
  /* USER CODE END MotorCTRTask */
}

/* USER CODE BEGIN Header_JoystickCTRTask */
/**
* @brief Function implementing the JoystickCTR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JoystickCTRTask */
void JoystickCTRTask(void *argument)
{
  /* USER CODE BEGIN JoystickCTRTask */
	HAL_ADC_Start_DMA(&hadc1, adcValue, 2);
  /* Infinite loop */
	for(;;) {
		if (ctrMode == 1) {
			adcX = adcValue[0] / 100;
			adcY = (adcValue[1] + 50) / 100;

			if (adcX >= 19 && adcX <= 21) {
				if (adcY >= 19 && adcY <= 21 ) {
					mcMode = 0;
					mtSpeed_R = 0;
					mtSpeed_L = 0;
				}
				else if (adcY < 19) {
					mcMode = 1;
					mtSpeed_R = (19 - adcY) * 50;
					mtSpeed_L = (19 - adcY) * 50;
				}
				else if (adcY > 21) {
					mcMode = 2;
					mtSpeed_R = (adcY - 21) * 50;
					mtSpeed_L = (adcY - 21) * 50;
				}
			}

			else if (adcX < 19) {
				if (adcY >= 19 && adcY <= 21 ) {
					mcMode = 3;
					mtSpeed_R = (19 - adcX) * 50;
					mtSpeed_L = 0;
				}
				else if (adcY < 19) {
					mcMode = 1;
					mtSpeed_R = (19 - adcY) * 50;
					if (adcX >= 17) {
						mtSpeed_L = (19 - adcY) * 45;
					}
					else if (adcX >= 15) {
						mtSpeed_L = (19 - adcY) * 40;
					}
					else if (adcX >= 13) {
						mtSpeed_L = (19 - adcY) * 35;
					}
					else if (adcX >= 11) {
						mtSpeed_L = (19 - adcY) * 30;
					}
					else if (adcX >= 9) {
						mtSpeed_L = (19 - adcY) * 25;
					}
					else if (adcX >= 7) {
						mtSpeed_L = (19 - adcY) * 20;
					}
					else if (adcX >= 5) {
						mtSpeed_L = (19 - adcY) * 15;
					}
					else if (adcX >= 3) {
						mtSpeed_L = (19 - adcY) * 10;
					}
					else if (adcX < 3) {
						mtSpeed_L = 0;
					}
				}
				else if (adcY > 21) {
					mcMode = 2;
					mtSpeed_R = (adcY - 21) * 50;
					if (adcX >= 17) {
						mtSpeed_L = (adcY - 21) * 45;
					}
					else if (adcX >= 15) {
						mtSpeed_L = (adcY - 21) * 40;
					}
					else if (adcX >= 13) {
						mtSpeed_L = (adcY - 21) * 35;
					}
					else if (adcX >= 11) {
						mtSpeed_L = (adcY - 21) * 30;
					}
					else if (adcX >= 9) {
						mtSpeed_L = (adcY - 21) * 25;
					}
					else if (adcX >= 7) {
						mtSpeed_L = (adcY - 21) * 20;
					}
					else if (adcX >= 5) {
						mtSpeed_L = (adcY - 21) * 15;
					}
					else if (adcX >= 3) {
						mtSpeed_L = (adcY - 21) * 10;
					}
					else if (adcX < 3) {
						mtSpeed_L = 0;
					}
				}
			}

			else if (adcX > 21) {
				if (adcY >= 19 && adcY <= 21 ) {
					mcMode = 4;
					mtSpeed_R = (adcX - 21) * 50;
					mtSpeed_L = 0;
				}
				else if (adcY < 19) {
					mcMode = 1;
					mtSpeed_L = (19 - adcY) * 50;
					if (adcX <= 23) {
						mtSpeed_R = (19 - adcY) * 45;
					}
					else if (adcX <= 25) {
						mtSpeed_R = (19 - adcY) * 40;
					}
					else if (adcX <= 27) {
						mtSpeed_R = (19 - adcY) * 35;
					}
					else if (adcX <= 29) {
						mtSpeed_R = (19 - adcY) * 30;
					}
					else if (adcX <= 31) {
						mtSpeed_R = (19 - adcY) * 25;
					}
					else if (adcX <= 33) {
						mtSpeed_R = (19 - adcY) * 20;
					}
					else if (adcX <= 35) {
						mtSpeed_R = (19 - adcY) * 15;
					}
					else if (adcX <= 37) {
						mtSpeed_R = (19 - adcY) * 10;
					}
					else if (adcX > 37) {
						mtSpeed_R = 0;
					}
				}
				else if (adcY > 21) {
					mcMode = 2;
					mtSpeed_L = (adcY - 21) * 50;
					if (adcX <= 23) {
						mtSpeed_R = (adcY - 21) * 45;
					}
					else if (adcX <= 25) {
						mtSpeed_R = (adcY - 21) * 40;
					}
					else if (adcX <= 27) {
						mtSpeed_R = (adcY - 21) * 35;
					}
					else if (adcX <= 29) {
						mtSpeed_R = (adcY - 21) * 30;
					}
					else if (adcX <= 31) {
						mtSpeed_R = (adcY - 21) * 25;
					}
					else if (adcX <= 33) {
						mtSpeed_R = (adcY - 21) * 20;
					}
					else if (adcX <= 35) {
						mtSpeed_R = (adcY - 21) * 15;
					}
					else if (adcX <= 37) {
						mtSpeed_R = (adcY - 21) * 10;
					}
					else if (adcX > 37) {
						mtSpeed_R = 0;
					}
				}
			}
		}

		osDelay(10);
	}
  /* USER CODE END JoystickCTRTask */
}

/* USER CODE BEGIN Header_BluetoothCTRTask */
/**
* @brief Function implementing the BluetoothCTR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BluetoothCTRTask */
void BluetoothCTRTask(void *argument)
{
  /* USER CODE BEGIN BluetoothCTRTask */

  /* Infinite loop */
	for(;;) {
		if (ctrMode == 2) {
			if (btMode == 'S') {
				mcMode = 0;
				mtSpeed_R = 0;
				mtSpeed_L = 0;
			}
			else if (btMode == 'F') {
				mcMode = 1;
				mtSpeed_R = BT_B_SPEED + (btSpeed * BT_M_SPEED);
				mtSpeed_L = BT_B_SPEED + (btSpeed * BT_M_SPEED);
			}
			else if (btMode == 'B') {
				mcMode = 2;
				mtSpeed_R = BT_B_SPEED + (btSpeed * BT_M_SPEED);
				mtSpeed_L = BT_B_SPEED + (btSpeed * BT_M_SPEED);
			}
			else if (btMode == 'L') {
				mcMode = 3;
				mtSpeed_R = BT_B_SPEED + (btSpeed * BT_M_SPEED);
				mtSpeed_L = 0;
			}
			else if (btMode == 'R') {
				mcMode = 4;
				mtSpeed_R = 0;
				mtSpeed_L = BT_B_SPEED + (btSpeed * BT_M_SPEED);
			}
			else if (btMode == 'G') {
				mcMode = 1;
				mtSpeed_R = BT_B_SPEED + (btSpeed * BT_M_SPEED);
				mtSpeed_L = (BT_B_SPEED + (btSpeed * BT_M_SPEED)) * 0.5;
			}
			else if (btMode == 'H') {
				mcMode = 1;
				mtSpeed_R = (BT_B_SPEED + (btSpeed * BT_M_SPEED)) * 0.5;
				mtSpeed_L = BT_B_SPEED + (btSpeed * BT_M_SPEED);
			}
			else if (btMode == 'I') {
				mcMode = 2;
				mtSpeed_R = BT_B_SPEED + (btSpeed * BT_M_SPEED);
				mtSpeed_L = (BT_B_SPEED + (btSpeed * BT_M_SPEED)) * 0.5;
			}
			else if (btMode == 'J') {
				mcMode = 1;
				mtSpeed_R = (BT_B_SPEED + (btSpeed * BT_M_SPEED)) * 0.5;
				mtSpeed_L = BT_B_SPEED + (btSpeed * BT_M_SPEED);
			}
		}

		osDelay(10);
	}
  /* USER CODE END BluetoothCTRTask */
}

/* USER CODE BEGIN Header_ModeCTRTask */
/**
* @brief Function implementing the ModeCTR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModeCTRTask */
void ModeCTRTask(void *argument)
{
  /* USER CODE BEGIN ModeCTRTask */
	ctrMode = 2;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ModeCTRTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

