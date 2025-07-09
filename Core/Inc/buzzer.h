/*
 * buzzer.h
 *
 *  Created on: Jul 10, 2025
 *      Author: user16
 */
#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "main.h"

#define BUZZER_PSC		TIM1->PSC
#define BUZZER_CCR		TIM1->CCR1

extern uint8_t songNote_1[];		// 똑같아요
extern uint8_t songLength_1[];
extern uint8_t songNote_2[];		// 엘리제를 위하여
extern uint8_t songLength_2[];
extern uint8_t songNote_3[];		// 이웃집 토토로 OST 산책?
extern uint8_t songLength_3[];
extern uint8_t songNote_4[];		// 하울의 움직이는 성 OST 인생의 회전목마?
extern uint8_t songLength_4[];

void buzzerStart(uint8_t index, uint16_t volumn);

#endif /* INC_BUZZER_H_ */
