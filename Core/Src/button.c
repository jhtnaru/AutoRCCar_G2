/*
 * button.c
 *
 *  Created on: Jun 27, 2025
 *      Author: user16
 */
// button.c
#include "button.h"

// Button Port, Pin 정보, Pull-Up이므로 0에서 ON
BUTTON_CONTROL button[1] = {
		{GPIOD, GPIO_PIN_2, 0}
};

uint8_t buttonFlag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_2) {
		buttonFlag = 1;
	}
}

// Button이 눌리면 Debounce 처리해서 True 반환
bool buttonGetPressed(uint8_t num) {
	static uint32_t prevTime = 0xffffffff;		// 초기값 최대치 설정, 처음부터 Debounce

	if(prevTime == 0xffffffff) {
		prevTime = HAL_GetTick();				// prevTime 초기화 작업
	}

	bool ret = false;

	if(HAL_GPIO_ReadPin(button[num].port, button[num].pinNumber) == button[num].onState) {
		uint32_t currTime = HAL_GetTick();

		if(currTime - prevTime >= 300) {		// Debounce Code
			if(HAL_GPIO_ReadPin(button[num].port, button[num].pinNumber) == button[num].onState) {
				ret = true;
			}
			prevTime = currTime;
		}
	}

	return ret;
}

