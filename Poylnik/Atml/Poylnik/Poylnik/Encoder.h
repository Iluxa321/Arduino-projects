/*
 * Encoder.h
 *
 * Created: 14.07.2020 13:21:55
 *  Author: Илья
 */ 


#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"

#define BTN_NO_ACTION	0
#define BTN_CLICK		1
#define BTN_HOLD		2

void encoder_init();
uint8_t encoder_state();
uint8_t encoder_button(uint32_t timer);
int32_t encoder_cnt(int32_t cnt, uint8_t dir, uint32_t num);

#endif /* ENCODER_H_ */