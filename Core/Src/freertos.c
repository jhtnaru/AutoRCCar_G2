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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"

#include "button.h"
#include "buzzer.h"
#include "delay_us.h"
#include "fnd.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_BASE				450
#define SPEED_RATIO				0.1
#define SPEED_M_BT				60

#define DIST_STD_1				14
#define DIST_STD_2				19
#define DIST_STD_3				26
#define DIST_STD_4				40

#define DIST_STD_11				15
#define DIST_STD_12				25
#define DIST_STD_13				35
#define DIST_STD_14				45

#define BUZZER_TICK				50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
/* With GCC small printf (option LD Linker->Libraries->Small printf
 * set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int  __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int  fputc(int ch, FILE *f)
#endif /* __GNUC__*/

/** @brief Retargets the C library printf function to the USART.
 *  @param None
 *  @retval None
 */
PUTCHAR_PROTOTYPE {
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop
     until the end of transmission */
  if(ch == '\n')
    HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rxData_1 = 0, rxData_2 = 0;
uint8_t mcMode;
uint16_t baseSpeed_0 = 0;
uint16_t baseSpeed_1 = 0;
uint16_t baseSpeed_2 = 0;
uint16_t mtSpeed_R = 0, mtSpeed_L = 0;

uint8_t ctrMode = 0;

uint8_t btMode = 'S';
uint8_t btSpeed = 0;

extern uint8_t distCenter, distLeft, distRight;

uint32_t tickCur_1 = 0, tickLast_1 = 0;		// Buzzer용

uint8_t buzzerFlag_1 = 0, songIndex_1 = 0;	//
uint8_t buzzerFlag_2 = 0, songIndex_2 = 0;
/* USER CODE END Variables */
/* Definitions for MotorCTR */
osThreadId_t MotorCTRHandle;
const osThreadAttr_t MotorCTR_attributes = {
  .name = "MotorCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
/* Definitions for UltrasonicCTR */
osThreadId_t UltrasonicCTRHandle;
const osThreadAttr_t UltrasonicCTR_attributes = {
  .name = "UltrasonicCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AutoCTR */
osThreadId_t AutoCTRHandle;
const osThreadAttr_t AutoCTR_attributes = {
  .name = "AutoCTR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (rxData_1 == 'Y') {
			buttonFlag = 1;
		}

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
		HAL_UART_Receive_DMA(&huart2, &rxData_2, 1);
	}
}

/* USER CODE END FunctionPrototypes */

void MotorCTRTask(void *argument);
void JoystickCTRTask(void *argument);
void BluetoothCTRTask(void *argument);
void ModeCTRTask(void *argument);
void UltrasonicCTRTask(void *argument);
void AutoCTRTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
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

  /* creation of UltrasonicCTR */
  UltrasonicCTRHandle = osThreadNew(UltrasonicCTRTask, NULL, &UltrasonicCTR_attributes);

  /* creation of AutoCTR */
  AutoCTRHandle = osThreadNew(AutoCTRTask, NULL, &AutoCTR_attributes);

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
		motorSelect(mcMode);
		baseSpeed_0 = SPEED_BASE + (btSpeed * SPEED_M_BT);

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
//	HAL_ADC_Start_DMA(&hadc1, adcValue, 2);
  /* Infinite loop */
	for(;;) {
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
		if (ctrMode == 1) {
			if (btMode == 'S') {
				mcMode = 0;
				mtSpeed_R = 0;
				mtSpeed_L = 0;
			}
			else if (btMode == 'F') {
				mcMode = 1;
				mtSpeed_R = baseSpeed_0;
				mtSpeed_L = baseSpeed_0;
			}
			else if (btMode == 'B') {
				mcMode = 2;
				mtSpeed_R = baseSpeed_0;
				mtSpeed_L = baseSpeed_0;
			}
			else if (btMode == 'L') {
				mcMode = 5;
				mtSpeed_R = baseSpeed_0;
				mtSpeed_L = baseSpeed_0;
			}
			else if (btMode == 'R') {
				mcMode = 6;
				mtSpeed_R = baseSpeed_0;
				mtSpeed_L = baseSpeed_0;
			}
			else if (btMode == 'G') {
				mcMode = 1;
				mtSpeed_R = baseSpeed_0;
				mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
			}
			else if (btMode == 'H') {
				mcMode = 1;
				mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
				mtSpeed_L = baseSpeed_0;
			}
			else if (btMode == 'I') {
				mcMode = 2;
				mtSpeed_R = baseSpeed_0;
				mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
			}
			else if (btMode == 'J') {
				mcMode = 2;
				mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
				mtSpeed_L = baseSpeed_0;
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

  /* Infinite loop */
	for(;;) {
		if (buttonGetPressed(0)) {
			buttonFlag = 1;
		}

		if (buttonFlag == 1) {			// 정면 기준 1
			ctrMode++;
			buttonFlag = 0;
			rxData_1 = 'S';
			if (ctrMode >= 9) {
				ctrMode = 0;
			}
		}

		if (ctrMode == 0 || ctrMode == 3 || ctrMode == 5 || ctrMode == 7) {
			mcMode = 0;
			mtSpeed_R = 0;
			mtSpeed_L = 0;
		}

		if (ctrMode == 1 && rxData_1 == 'X') {
			buzzerFlag_1 = 1;
		}
		else if (ctrMode == 1 && rxData_1 == 'x') {
			buzzerFlag_1 = 0;
		}
		else if (ctrMode == 2) {
			buzzerFlag_1 = 1;
		}
		else {
			buzzerFlag_1 = 0;
		}

		if (ctrMode == 1 && mcMode == 2) {
			buzzerFlag_2 = 1;
		}
		else {
			buzzerFlag_2 = 0;
		}

		tickCur_1 = xTaskGetTickCount();

		if (buzzerFlag_1 == 1) {
			if ((tickCur_1 - tickLast_1) >= (BUZZER_TICK * songLength_4[songIndex_1])) {
				songIndex_1++;
				if (songIndex_1 >= songNote_4_Len) {
					songIndex_1 = 0;
				}
				tickLast_1 = tickCur_1;
			}
			buzzerStart(songNote_4[songIndex_1], baseSpeed_0);
		}
		else if (buzzerFlag_1 == 0 && buzzerFlag_2 == 1) {
			if ((tickCur_1 - tickLast_1) >= (BUZZER_TICK * songLength_2[songIndex_2])) {
				songIndex_2++;
				if (songIndex_2 >= songNote_2_Len) {
					songIndex_2 = 0;
				}
				tickLast_1 = tickCur_1;
			}
			buzzerStart(songNote_2[songIndex_2], baseSpeed_0);
		}
		else if (buzzerFlag_1 == 0 && buzzerFlag_2 == 0) {
			buzzerStart(24, 0);
		}

		fndOut(ctrMode);

		osDelay(10);
	}
  /* USER CODE END ModeCTRTask */
}

/* USER CODE BEGIN Header_UltrasonicCTRTask */
/**
* @brief Function implementing the UltrasonicCTR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasonicCTRTask */
void UltrasonicCTRTask(void *argument)
{
  /* USER CODE BEGIN UltrasonicCTRTask */

  /* Infinite loop */
	for(;;) {
		__HAL_TIM_SET_COUNTER(&htim3, 0);

		HCSR04_TRIG();
		osDelay(50);
	}
  /* USER CODE END UltrasonicCTRTask */
}

/* USER CODE BEGIN Header_AutoCTRTask */
/**
* @brief Function implementing the AutoCTR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AutoCTRTask */
void AutoCTRTask(void *argument)
{
  /* USER CODE BEGIN AutoCTRTask */
  /* Infinite loop */
	for(;;) {
		//  자율주행 기본
		if (ctrMode == 2) {
			// 정면 기준 1
			if (distCenter <= DIST_STD_1) {
				if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}

				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_2) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight > DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
				}

				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight > DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
				}

				else if (distLeft > DIST_STD_4) {
					if (distLeft >= distRight) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distLeft < distRight) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}

			// 정면 기준 2
			else if (distCenter <= DIST_STD_2) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if(distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_2) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}

				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight > DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
				}

				else if (distLeft > DIST_STD_2) {
					if (distLeft >= distRight) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distLeft < distRight) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}

			// 정면 기준 3
			else if (distCenter <= DIST_STD_3) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_1) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}

				else if (distLeft <= DIST_STD_4) {
					if (distLeft >= distRight) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distLeft < distRight) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}

				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}

			// 정면 기준 4
			else if (distCenter > DIST_STD_3) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_1) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}

				else if (distLeft <= DIST_STD_3) {
					if (distLeft >= distRight) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distLeft < distRight) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}

				else if (distLeft > DIST_STD_3) {
					if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}
		}

		// 자율주행 Test 1
		else if (ctrMode == 4) {

		}

		// 자율주행 Test 2
		else if (ctrMode == 6) {

		}

		// 자율주행 Test 3
		else if (ctrMode == 8) {

		}

		else if (ctrMode == 9) {
/*			// Test Base
 * 			// 정면 기준 1
			if (distCenter <= DIST_STD_1) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_2) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;

					}
					else if (distRight > DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
				}
			}

			// 정면 기준 2
			else if (distCenter <= DIST_STD_2) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if(distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_2) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if(distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
				}
			}

			// 정면 기준 3
			else if (distCenter <= DIST_STD_3) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_1) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}

			// 정면 기준 4
			else if (distCenter <= DIST_STD_4) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_1) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}

			// 정면 기준 5
			else if (distCenter > DIST_STD_4) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_2) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = baseSpeed_0;
							mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
							mtSpeed_L = baseSpeed_0;
						}
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = baseSpeed_0 * SPEED_RATIO;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0 * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_3) {
						mcMode = 1;
						mtSpeed_R = baseSpeed_0;
						mtSpeed_L = baseSpeed_0;
					}
				}
			}
*/
		}

		osDelay(10);
	}
  /* USER CODE END AutoCTRTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

