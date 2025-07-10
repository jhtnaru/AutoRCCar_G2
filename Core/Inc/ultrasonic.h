/*
 * ultrasonic.h
 *
 *  Created on: Jul 10, 2025
 *      Author: user16
 */
#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "main.h"
#include "tim.h"

#define TRIG1_PORT				GPIOB
#define TRIG1_PIN					GPIO_PIN_15

extern uint16_t IC_Value_1[2];
extern uint16_t IC_Value_2[2];
extern uint16_t IC_Value_3[2];

extern uint16_t echoTime_1, echoTime_2, echoTime_3;
extern uint8_t captureFlag_1, captureFlag_2, captureFlag_3;
extern uint8_t distCenter, distLeft, distRight;

void HCSR04_TRIG(void);

#endif /* INC_ULTRASONIC_H_ */
