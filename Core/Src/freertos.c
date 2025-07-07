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
#include "stdio.h"
#include "delay_us.h"
#include "button.h"

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

#define TRIG1_PORT				GPIOB
#define TRIG1_PIN					GPIO_PIN_15
#define TRIG2_PORT				GPIOB
#define TRIG2_PIN					GPIO_PIN_14
#define TRIG3_PORT				GPIOB
#define TRIG3_PIN					GPIO_PIN_13

#define SPEED_BASE				400
#define SPEED_RATIO				0.2

#define SPEED_M_BT				65
#define SPEED_AUTO_1				70
#define SPEED_AUTO_2				70
#define SPEED_AUTO_3				70

#define DIST_STD_0				5
#define DIST_STD_1				15
#define DIST_STD_2				25
#define DIST_STD_3				40
#define DIST_STD_4				55
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
uint8_t mtMode_L1 = 0, mtMode_L2 = 0;
uint8_t mtMode_R1 = 0, mtMode_R2 = 0;
uint16_t mtSpeed_R = 0, mtSpeed_L = 0;

volatile uint16_t adcValue[2] = {0};
uint16_t adcX, adcY;

uint8_t ctrMode = 0;
uint8_t buttonFlag = 0;

uint8_t btMode = 'S', btSpeed = 0;

uint16_t IC_Value_1[2] = {0};
uint16_t IC_Value_2[2] = {0};
uint16_t IC_Value_3[2] = {0};

uint16_t echoTime_1 = 0, echoTime_2 = 0, echoTime_3 = 0;
uint8_t captureFlag_1 = 0, captureFlag_2 = 0, captureFlag_3 = 0;
uint8_t distCenter = 0, distLeft = 0, distRight = 0;

uint32_t tickCur_1 = 0, tickCur_2 = 0, tickCur_3 = 0;
uint32_t tickLast_1 = 0, tickLast_2 = 0, tickLast_3 = 0;
uint32_t tickBase = 5;

uint8_t tickIndex_1 = 0, tickIndex_2 = 0, tickIndex_3 = 0;

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
		else if (rxData_1 > 200) {
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
}

void HCSR04_TRIG1(void) {
	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
}

void HCSR04_TRIG2(void) {
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
}

void HCSR04_TRIG3(void) {
	HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG3_PORT, TRIG3_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC3);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (captureFlag_1 == 0) {					// Capture 안했다면
			IC_Value_1[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			captureFlag_1 = 1;					// Capture 했음
			// Capture 극성을 Rising에서 Falling으로 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (captureFlag_1 == 1) {			// Capture 했다면
			IC_Value_1[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(&htim2, 0);

			if (IC_Value_1[1] > IC_Value_1[0]) {		// 같은 주기인 경우
				echoTime_1 =  IC_Value_1[1] - IC_Value_1[0];
			}
			else if (IC_Value_1[0] > IC_Value_1[1]) {	// 주기가 바뀐 경우
				echoTime_1 = (0xffff - IC_Value_1[0]) + IC_Value_1[1];
			}

			distCenter = echoTime_1 / 58;
			captureFlag_1 = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);		// 한번 끝나면 Disable
		}
	}

	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (captureFlag_2 == 0) {					// Capture 안했다면
			IC_Value_2[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			captureFlag_2 = 1;					// Capture 했음
			// Capture 극성을 Rising에서 Falling으로 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (captureFlag_2 == 1) {			// Capture 했다면
			IC_Value_2[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);

			if (IC_Value_2[1] > IC_Value_2[0]) {		// 같은 주기인 경우
				echoTime_2 =  IC_Value_2[1] - IC_Value_2[0];
			}
			else if (IC_Value_2[0] > IC_Value_2[1]) {	// 주기가 바뀐 경우
				echoTime_2 = (0xffff - IC_Value_2[0]) + IC_Value_2[1];
			}

			distLeft = echoTime_2 / 58;
			captureFlag_2 = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);		// 한번 끝나면 Disable
		}
	}

	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (captureFlag_3 == 0) {					// Capture 안했다면
			IC_Value_3[0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			captureFlag_3 = 1;					// Capture 했음
			// Capture 극성을 Rising에서 Falling으로 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (captureFlag_3 == 1) {			// Capture 했다면
			IC_Value_3[1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			__HAL_TIM_SET_COUNTER(&htim2, 0);

			if (IC_Value_3[1] > IC_Value_3[0]) {		// 같은 주기인 경우
				echoTime_3 =  IC_Value_3[1] - IC_Value_3[0];
			}
			else if (IC_Value_3[0] > IC_Value_3[1]) {	// 주기가 바뀐 경우
				echoTime_3 = (0xffff - IC_Value_3[0]) + IC_Value_3[1];
			}

			distRight = echoTime_3 / 58;
			captureFlag_3 = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC3);		// 한번 끝나면 Disable
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_2) {
		buttonFlag = 1;
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
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

HAL_TIM_Base_Start(&htim11);

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
		tickCur_1 = xTaskGetTickCount();

		if ((tickCur_1 - tickLast_1) > 100) {
			tickIndex_1 = !tickIndex_1;
			tickLast_1 = tickCur_1;
		}

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

		case 5:
			mtMode_R1 = 1;
			mtMode_R2 = 0;
			mtMode_L1 = 1;
			mtMode_L2 = 0;
			break;

		case 6:
			mtMode_R1 = 1;
			mtMode_R2 = 0;
			mtMode_L1 = 1;
			mtMode_L2 = 0;
			break;

		case 7:
			mtMode_R1 = 0;
			mtMode_R2 = 1;
			mtMode_L1 = 0;
			mtMode_L2 = 1;
			break;

		case 8:
			mtMode_R1 = 0;
			mtMode_R2 = 1;
			mtMode_L1 = 0;
			mtMode_L2 = 1;
			break;

		case 9:
			mtMode_R1 = 1;
			mtMode_R2 = 1;
			mtMode_L1 = 1;
			mtMode_L2 = 1;
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
//	HAL_ADC_Start_DMA(&hadc1, adcValue, 2);
  /* Infinite loop */
	for(;;) {
		if (ctrMode == 2) {

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
		tickCur_2 = xTaskGetTickCount();

		if ((tickCur_2 - tickLast_2) > 100) {
			tickIndex_2 = !tickIndex_2;
			tickLast_2 = tickCur_2;
		}

		if (ctrMode == 1) {
			if (btMode == 'S') {
				mcMode = 0;
				mtSpeed_R = 0;
				mtSpeed_L = 0;
			}
			else if (btMode == 'F') {
				mcMode = 1;
				mtSpeed_R = SPEED_BASE + (btSpeed * SPEED_M_BT);
				mtSpeed_L = SPEED_BASE + (btSpeed * SPEED_M_BT);
			}
			else if (btMode == 'B') {
				mcMode = 2;
				mtSpeed_R = SPEED_BASE + (btSpeed * SPEED_M_BT);
				mtSpeed_L = SPEED_BASE + (btSpeed * SPEED_M_BT);
			}
			else if (btMode == 'L') {
				mcMode = 3;
				mtSpeed_R = SPEED_BASE + (btSpeed * SPEED_M_BT);
				mtSpeed_L = 0;
			}
			else if (btMode == 'R') {
				mcMode = 4;
				mtSpeed_R = 0;
				mtSpeed_L = SPEED_BASE + (btSpeed * SPEED_M_BT);
			}
			else if (btMode == 'G') {
				mcMode = 5;
				mtSpeed_R = SPEED_BASE + (btSpeed * SPEED_M_BT);
				mtSpeed_L = (SPEED_BASE + (btSpeed * SPEED_M_BT)) * SPEED_RATIO;
			}
			else if (btMode == 'H') {
				mcMode = 6;
				mtSpeed_R = (SPEED_BASE + (btSpeed * SPEED_M_BT)) * SPEED_RATIO;
				mtSpeed_L = SPEED_BASE + (btSpeed * SPEED_M_BT);
			}
			else if (btMode == 'I') {
				mcMode = 7;
				mtSpeed_R = SPEED_BASE + (btSpeed * SPEED_M_BT);
				mtSpeed_L = (SPEED_BASE + (btSpeed * SPEED_M_BT)) * SPEED_RATIO;
			}
			else if (btMode == 'J') {
				mcMode = 8;
				mtSpeed_R = (SPEED_BASE + (btSpeed * SPEED_M_BT)) * SPEED_RATIO;
				mtSpeed_L = SPEED_BASE + (btSpeed * SPEED_M_BT);
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

		if (buttonFlag == 1) {
			ctrMode++;
			buttonFlag = 0;
			rxData_1 = 'S';
			if (ctrMode >= 4) {
				ctrMode = 0;
			}
		}

		if (ctrMode == 0) {
			mcMode = 0;
			mtSpeed_R = 0;
			mtSpeed_L = 0;
		}

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
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  /* Infinite loop */
	for(;;) {
		HCSR04_TRIG1();
		osDelay(60);

		HCSR04_TRIG2();
		osDelay(60);

		HCSR04_TRIG3();
		osDelay(40);
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
		if  (ctrMode == 3) {
			if (distCenter <= DIST_STD_0 || distLeft <= DIST_STD_0 || distRight <= DIST_STD_0) {
				if (distLeft >= distRight) {
					mcMode = 8;
					mtSpeed_R = SPEED_BASE * SPEED_RATIO;
					mtSpeed_L = SPEED_BASE;
				}
				else if (distLeft < distRight) {
					mcMode = 7;
					mtSpeed_R = SPEED_BASE;
					mtSpeed_L = SPEED_BASE * SPEED_RATIO;
				}
			}

			// 정면 기준 거리 1 이내
			else if (distCenter <= DIST_STD_1) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 3;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = SPEED_BASE * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = SPEED_BASE * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = (SPEED_BASE) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
						}
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;

					}
					else if (distRight > DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
						}
					}
				}
			}

			// 정면 기준 거리 2 이내
			else if (distCenter <= DIST_STD_2) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = 0;
						}
						else if(distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 4;
						mtSpeed_R = 0;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 3;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = 0;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = SPEED_BASE * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = SPEED_BASE * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = SPEED_BASE * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
						}
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
						}
						else if(distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
						}
					}
				}
			}

			// 정면 기준 거리 3 이내
			else if (distCenter <= DIST_STD_3) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;

					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = SPEED_BASE * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = SPEED_BASE * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
						}
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = (SPEED_BASE) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
			}

			// 정면 기준 거리 4 이내
			else if (distCenter <= DIST_STD_4) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 3;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = 0;
						}
						else if (distLeft < distRight) {
							mcMode = 4;
							mtSpeed_R = 0;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = SPEED_BASE * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = SPEED_BASE * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = (SPEED_BASE) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
			}

			// 정면 기준 거리 4 Over
			else if (distCenter > DIST_STD_4) {
				if (distLeft <= DIST_STD_1) {
					if (distRight <= DIST_STD_1) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE;
							mtSpeed_L = SPEED_BASE * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = SPEED_BASE * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE;
						}
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = SPEED_BASE * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
					}
				}
				else if (distLeft <= DIST_STD_2) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_1;
						}
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
				else if (distLeft <= DIST_STD_3) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE;
						mtSpeed_L = SPEED_BASE * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						if (distLeft >= distRight) {
							mcMode = 5;
							mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
							mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
						}
						else if (distLeft < distRight) {
							mcMode = 6;
							mtSpeed_R = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
							mtSpeed_L = SPEED_BASE + SPEED_AUTO_2;
						}
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 6;
						mtSpeed_R = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
				else if (distLeft <= DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
				else if (distLeft > DIST_STD_4) {
					if (distRight <= DIST_STD_1) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_1;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_1) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_2) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_2;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_2) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_3) {
						mcMode = 5;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = (SPEED_BASE + SPEED_AUTO_3) * SPEED_RATIO;
					}
					else if (distRight <= DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
					else if (distRight > DIST_STD_4) {
						mcMode = 1;
						mtSpeed_R = SPEED_BASE + SPEED_AUTO_3;
						mtSpeed_L = SPEED_BASE + SPEED_AUTO_3;
					}
				}
			}
		}
		osDelay(10);
	}
  /* USER CODE END AutoCTRTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

