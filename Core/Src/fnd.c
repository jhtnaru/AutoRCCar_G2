/*
 * fnd.c
 *
 *  Created on: Jul 9, 2025
 *      Author: user16
 */
// fnd.c
#include "fnd.h"

// FND 출력용 숫자 배열, 0 ~ 9, 0. ~ 9.
uint8_t segNum[20] = { // .GFEDCBA
    0x3f, 0x06,     // 00111111 --FEDCBA 0, 00000110 -----CB- 1
    0x5b, 0x4f,     // 01011011 -G-ED-BA 2, 01001111 -G--DCBA 3
    0x66, 0x6d,     // 01100110 -GF--CB- 4, 01101101 -GF-DC-A 5
    0x7d, 0x27,     // 01111101 -GFEDC-A 6, 00100111 --F--CBA 7
    0x7f, 0x67,     // 01111111 -GFEDCBA 8, 01100111 -GF--CBA 9
    0xbf, 0x86,     // 10111111 .-FEDCBA 0, 10000110 .----CB- 1
    0xdb, 0xcf,     // 11011011 .G-ED-BA 2, 11001111 .G--DCBA 3
    0xe6, 0xed,     // 11100110 .GF--CB- 4, 11101101 .GF-DC-A 5
    0xfd, 0xa7,     // 11111101 .GFEDC-A 6, 10100111 .-F--CBA 7
    0xff, 0xe7      // 11111111 .GFEDCBA 8, 11100111 .GF--CBA 9
};

void fndOut(uint8_t data) {
	for (int8_t i = 7; i  >= 0 ; i--) {		// MSB → LSB
		if (segNum[data] & (1 << i)) {
			HAL_GPIO_WritePin(SER_PORT, SER_PIN, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(SER_PORT, SER_PIN, GPIO_PIN_RESET);
		}

		// Shift Clock 생성
		HAL_GPIO_WritePin(SRCLK_PORT, SRCLK_PIN, GPIO_PIN_SET);
		delay_us(5);
		HAL_GPIO_WritePin(SRCLK_PORT, SRCLK_PIN, GPIO_PIN_RESET);
		delay_us(5);
	}

	// 저장 Clock
	HAL_GPIO_WritePin(RCLK_PORT, RCLK_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(RCLK_PORT, RCLK_PIN, GPIO_PIN_RESET);
}
