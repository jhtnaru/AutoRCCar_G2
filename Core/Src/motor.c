/*
 * motor.c
 *
 *  Created on: Jul 8, 2025
 *      Author: user16
 */
// motor.c
#include "motor.h"

uint8_t mtMode_L1 = 0, mtMode_L2 = 0;
uint8_t mtMode_R1 = 0, mtMode_R2 = 0;

void motorSelect(uint8_t select) {
	switch (select) {
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
		mtMode_L1 = 0;
		mtMode_L2 = 1;
		break;

	case 6:
		mtMode_R1 = 0;
		mtMode_R2 = 1;
		mtMode_L1 = 1;
		mtMode_L2 = 0;
		break;

	case 7:
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
}

