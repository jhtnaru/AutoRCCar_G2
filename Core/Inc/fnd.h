/*
 * fnd.h
 *
 *  Created on: Jul 9, 2025
 *      Author: user16
 */
#ifndef INC_FND_H_
#define INC_FND_H_

#include "main.h"
#include "delay_us.h"

#define SER_PORT			GPIOB
#define SER_PIN			GPIO_PIN_3
#define RCLK_PORT			GPIOB
#define RCLK_PIN			GPIO_PIN_4
#define SRCLK_PORT		GPIOB
#define SRCLK_PIN			GPIO_PIN_5

void fndOut(uint8_t data);

#endif /* INC_FND_H_ */
