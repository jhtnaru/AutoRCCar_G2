/*
 * ultrasonic.c
 *
 *  Created on: Jul 10, 2025
 *      Author: user16
 */
// ultrasonic.c
#include "ultrasonic.h"

uint16_t IC_Value_1[2] = {0};
uint16_t IC_Value_2[2] = {0};
uint16_t IC_Value_3[2] = {0};

uint16_t echoTime_1 = 0, echoTime_2 = 0, echoTime_3 = 0;
uint8_t captureFlag_1 = 0, captureFlag_2 = 0, captureFlag_3 = 0;

uint8_t distCenter = 0, distLeft = 0, distRight = 0;

void HCSR04_TRIG(void) {
	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG1_PORT, TRIG1_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (captureFlag_1 == 0) {					// Capture 안했다면
			IC_Value_1[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
			captureFlag_1 = 1;					// Capture 했음
			// Capture 극성을 Rising에서 Falling으로 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (captureFlag_1 == 1) {			// Capture 했다면
			IC_Value_1[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

			if (IC_Value_1[1] > IC_Value_1[0]) {		// 같은 주기인 경우
				echoTime_1 =  IC_Value_1[1] - IC_Value_1[0];
			}
			else if (IC_Value_1[0] > IC_Value_1[1]) {	// 주기가 바뀐 경우
				echoTime_1 = (0xffff - IC_Value_1[0]) + IC_Value_1[1];
			}

			distCenter = echoTime_1 / 58;
			if (distCenter >= 50) {
				distCenter = 50;
			}
			captureFlag_1 = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);		// 한번 끝나면 Disable
		}
	}

	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (captureFlag_2 == 0) {					// Capture 안했다면
			IC_Value_2[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
			captureFlag_2 = 1;					// Capture 했음
			// Capture 극성을 Rising에서 Falling으로 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (captureFlag_2 == 1) {			// Capture 했다면
			IC_Value_2[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

			if (IC_Value_2[1] > IC_Value_2[0]) {		// 같은 주기인 경우
				echoTime_2 =  IC_Value_2[1] - IC_Value_2[0];
			}
			else if (IC_Value_2[0] > IC_Value_2[1]) {	// 주기가 바뀐 경우
				echoTime_2 = (0xffff - IC_Value_2[0]) + IC_Value_2[1];
			}

			distLeft = echoTime_2 / 58;
			if (distLeft >= 50) {
				distLeft = 50;
			}
			captureFlag_2 = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);		// 한번 끝나면 Disable
		}
	}

	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (captureFlag_3 == 0) {					// Capture 안했다면
			IC_Value_3[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
			captureFlag_3 = 1;					// Capture 했음
			// Capture 극성을 Rising에서 Falling으로 변경
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (captureFlag_3 == 1) {			// Capture 했다면
			IC_Value_3[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);

			if (IC_Value_3[1] > IC_Value_3[0]) {		// 같은 주기인 경우
				echoTime_3 =  IC_Value_3[1] - IC_Value_3[0];
			}
			else if (IC_Value_3[0] > IC_Value_3[1]) {	// 주기가 바뀐 경우
				echoTime_3 = (0xffff - IC_Value_3[0]) + IC_Value_3[1];
			}

			distRight = echoTime_3 / 58;
			if (distRight >= 50) {
				distRight = 50;
			}
			captureFlag_3 = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC3);		// 한번 끝나면 Disable
		}
	}
}
