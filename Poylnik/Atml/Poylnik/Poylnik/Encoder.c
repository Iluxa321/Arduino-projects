#include "Encoder.h"

char str[10];

#define ENCODER_DDR DDRD
#define ENCODER_PORT PORTD
#define ENCODER_PIN PIND
#define CLK PD5
#define DT PD6
#define SW PD7

#define RIGH 1
#define LEFT 2



void bit_mask(unsigned char mask)
{
	for(char i = 0; i < 8; i++)
	{
		if(mask & 0x80) sendchar(1 + 0x30);
		else sendchar(0 + 0x30);
		mask<<=1;
	}
}

void encoder_init()
{
	ENCODER_DDR &= ~((1<<CLK) | (1<<DT) | (1<<SW));
	ENCODER_PORT |= (1<<CLK) | (1<<DT) | (1<<SW);
}

uint8_t encoder_state()
{
	static unsigned char encoder_st;
	unsigned char current_state = 0;
	if((ENCODER_PIN & (1<<CLK)) != 0) current_state |= (1<<0);
	if((ENCODER_PIN & (1<<DT)) != 0) current_state |= (1<<1);
	if ((encoder_st & 0x03) == current_state) return 0;
	encoder_st = (encoder_st << 2) | current_state;
	if (encoder_st == 210) return LEFT;
	if (encoder_st == 225) return RIGH;
	return 0;
}

uint8_t encoder_button(uint32_t timer){
	static uint32_t button_timer = 0;
	static uint8_t last_state = 0;
	static uint8_t repit = 0;	// исполуется когда кнопка зажата
	uint8_t res = BTN_NO_ACTION;
	uint8_t current_state = !(ENCODER_PIN & (1 << SW));
	if(current_state && !last_state){
		button_timer = timer;
		repit = 0;
	}
	else if(last_state){
		if(!current_state && ((timer - button_timer) > 200) && repit == 0){
			res = BTN_CLICK;
		}
		else if((timer - button_timer) > 1500 && repit == 0){
			button_timer = timer;
			repit = 1;
			res = BTN_HOLD;
		}
		else if(repit == 1 && (timer - button_timer) > 1000){
			button_timer = timer;
			res = BTN_HOLD;
		}
	}
	last_state = current_state;
	return res;
}

int32_t encoder_cnt(int32_t cnt, uint8_t dir, uint32_t num)
{
	if(dir == RIGH) 
		return cnt += num;
	else if(dir == LEFT)
		return cnt -= num;
	else return cnt;
}
