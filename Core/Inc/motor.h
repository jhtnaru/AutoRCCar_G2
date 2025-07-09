/*
 * motor.h
 *
 *  Created on: Jul 8, 2025
 *      Author: user16
 */
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

#define M_IN1_PORT				GPIOC
#define M_IN1_PIN					GPIO_PIN_12
#define M_IN2_PORT				GPIOC
#define M_IN2_PIN					GPIO_PIN_10
#define M_IN3_PORT				GPIOA
#define M_IN3_PIN					GPIO_PIN_6
#define M_IN4_PORT				GPIOA
#define M_IN4_PIN					GPIO_PIN_7

void motorSelect(uint8_t select);

#endif /* INC_MOTOR_H_ */
