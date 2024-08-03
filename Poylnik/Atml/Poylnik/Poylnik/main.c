#include "main.h"

#define Kp_BASE_ADDR	1
#define Ki_BASE_ADDR	4

volatile uint32_t timer = 0;
char str[20];

union eeprom_data{
	uint8_t data[4];
	uint32_t word;	
}e_data;

/////////////////////////////////////////////////////////////
// Фиксированная точка
/////////////////////////////////////////////////////////////
/*typedef int16_t _iq;
#define _Q            (5)
#define _IQ5(A)       (int16_t)((A) * 32.0)
#define _IQ(A)        _IQ5(A)
#define _IQmpy(A, B)  (int16_t)(((int32_t)A * (int32_t)B) >> _Q)*/

typedef int32_t _iq;
#define _Q            (16)
#define _IQ16(A)       (int32_t)((A) * 65536.0)
#define _IQ(A)        _IQ16(A)
#define _IQmpy(A, B)  (int32_t)(((int64_t)A * (int64_t)B) >> _Q)

#define m_abs(A)	  (A > 0 ? A : -A)
///////////////////////////////////////////////////////////////
#define temp A7                                             // ножка для считывания температуры термопары
#define deg_const     _IQ(5.0 / (1024.0 * 101.0 * 39e-6))   // коэффициент преобразования числа в температуру
#define kT            (310.0/185.0)                         // температурный коэффициент
#define _kT			 _IQ(1.0/kT)
#define R			 _IQ(19.5)								// сопротивление цепи паяльника



long timer1 = 0;
uint16_t disp_t = 150;
uint8_t poylnik_on = 0;

///////////////////////////////////////////////////////////////
// Переменные ПИД регулятора
///////////////////////////////////////////////////////////////
#define limMax    _IQ(5000.0)
#define limMin    _IQ(0.0)
#define step	 _IQ(0.01)				 // шаг обновления  параметров ПИД регулятора 

#define kp  35.5
#define ki  2.5    // 77.16; 58.5; 12.5
#define kd  2.25    // 0.14; 0.25
#define T   40e-3


_iq Kp = _IQ(kp);
_iq Ki = _IQ(ki * T);
_iq Kd = _IQ(kd / T);
uint8_t update_I = 1;


_iq pre_error = _IQ(0.0);
_iq setpoint = _IQ(150.0 * kT);
_iq derivative = _IQ(0.0);
_iq integral = _IQ(0.0);
_iq control = _IQ(0.0);
_iq proportional = _IQ(0.0);
/////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
// Фильтр среднего
/////////////////////////////////////////////////////////////////
typedef struct{
	uint8_t n; // порядок фильтра
	uint8_t i;
	uint32_t *arr;
	uint32_t sum;
}filter_avg;

uint32_t arr[8] = {0};
filter_avg flt = {8, 0, arr, 0};

uint32_t arr2[32] = {0};
filter_avg flt_u = {32, 0, arr2, 0};

uint16_t fileter_handler(filter_avg *f, uint16_t x){
	f->sum -= f->arr[f->i];
	f->arr[f->i] = x;
	f->sum += f->arr[f->i];
	f->i = (f->i + 1) % f->n;
	return f->sum / f->n;
}
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
//	Menu
//////////////////////////////////////////////////////////////////

enum Poilnik_State{
	IDLE = 0,
	WORK,
	SETTING,
	KP_SETTING,
	KI_SETTING
};


/////////////////////////////////////////////////////////////////
// Таймер для формирования задержки
/////////////////////////////////////////////////////////////////
void init_timer1(void){
	TCCR0 |= (1 << CS00);
	TIMSK |= (1 << TOIE0);	
}

uint32_t millis(void){
	return timer;
}
//////////////////////////////////////////////////////////////////


void pwm1_init(void){
	// Инициализвация ШИМ
	DDRB &= ~(1 << DDRB1); 
	// OC1A PWM
	// Fast PWM
	TCCR1B = 0;
	TCCR1A = 0;
	TCNT1 = 0;
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	TCCR1A |= (1 << WGM11);
	TCCR1A |= (1 << COM1A1); // pwm non-inverting mode

	// 8М / 64 / 25 == 5000
	ICR1 = 5000;
	// Задем коэффициент заполенния
	OCR1A = 2500;
	// Прерывание по совпадению с ICR1
	TIMSK |= (1 << TICIE1);
	TCCR1B |= (1 << CS11) | (1 << CS10) ; // 8M/64 = 125000
}



void fix_print(_iq a){
	uint16_t integer = (uint16_t)(a >> _Q);
	uint16_t frac = ((a & 0x0000FFFF) >> (_Q - 10)) * 625 / 64;
	uint8_t tens = frac / 1000;
	uint8_t ones = frac % 1000 / 100;
	
	if((frac % 1000 / 100) != 0){
		ones += 1;
		if(ones == 10){
			ones = 0;
			tens += 1;
			if(tens == 10){
				tens = 0;
				integer += 1;
			}
		}	
	}
	
	sprintf(str, "%d", integer);
	str_lcd(str);
	str_lcd(".");
	sprintf(str, "%d", tens);
	str_lcd(str);
	sprintf(str, "%d",ones);
	str_lcd(str);
	
}

uint16_t get_frac(_iq a){
	a = (((a & 0xFFFF) * 1638400) >> 14);
	return (uint16_t)(a);
}


void menu_handler(uint8_t btn, uint8_t dr, uint32_t timer){
	static enum Poilnik_State state = IDLE;
	static uint32_t my_timer = 0;	// испульзуется для обновления дисплея
	static uint8_t blink = 0;
	_iq current_temp = _IQmpy(_IQ((flt.sum / flt.n)), deg_const);
	current_temp = _IQmpy(current_temp, _kT);
	sprintf(str, "%d", (int16_t)(current_temp >> _Q));
	switch(state){
		case IDLE:{
			if(btn == BTN_CLICK){
				state = WORK;
				my_timer = timer;
				clear_ldc();
				lcd_led_on(); // включаем подсветку
				setpoint = _IQ(150 * kT);
				disp_t = 150;
				DDRB |= (1 << DDRB1); // включаем шим
			}
			else state = IDLE;
			break;
		}
		case WORK:{
			if(btn == BTN_CLICK){
				state = IDLE;
				clear_ldc();
				lcd_led_off(); // включаем подсветку
				DDRB &= ~(1 << DDRB1); // включаем шим
			}
			else if(btn == BTN_HOLD){
				state = SETTING;
			}
			if((timer - my_timer) > 500){
				my_timer = timer;
				
				
				clear_ldc();
				setpos(0,0);
				str_lcd("tT: ");
				str_lcd(str);
				str_lcd(", tY: ");
				sprintf(str, "%d", disp_t);
				str_lcd(str);
			}
			if(dr != 0){
				int16_t tmp = disp_t;
				tmp = (int16_t)encoder_cnt((uint32_t)tmp, dr, 5);
				if((tmp <= 350) && (tmp >= 150)){
					disp_t = tmp;
					setpoint = _IQ(tmp * kT);
				}
			}
			break;
		}
		case SETTING:{
			if(btn == BTN_HOLD){
				state = WORK;
				clear_ldc();
			}
			else if(btn == BTN_CLICK){
				state = KP_SETTING;
			}
			if((timer - my_timer) > 500){
				my_timer = timer;
								
				clear_ldc();
				
				setpos(13,0);
				str_lcd(str);
				setpos(0,0);
				str_lcd("Kp:");
				fix_print(Kp);
				setpos(0,1);
				str_lcd("Ki:");
				fix_print(Ki);
			}
			break;
		}
		case KP_SETTING:{
			if(btn == BTN_CLICK){
				state = KI_SETTING;
			}
			else if(btn == BTN_HOLD){
				state = WORK;
				clear_ldc();
			}
			if((timer - my_timer) > 400){
				my_timer = timer;
				if(blink){
					clear_ldc();
					setpos(13,0);
					str_lcd(str);
					setpos(0,0);
					str_lcd("Kp:");
					setpos(0,1);
					str_lcd("Ki:");
					fix_print(Ki);
				}
				else{
					clear_ldc();
					setpos(13,0);
					str_lcd(str);
					setpos(0,0);
					str_lcd("Kp:");
					fix_print(Kp);
					setpos(0,1);
					str_lcd("Ki:");
					fix_print(Ki);
				}
				blink = !blink;		
			}
			if(dr != 0){
				_iq tmp = encoder_cnt(Kp, dr, step);
				if(tmp > 0){
					cli();	// отключаем прерывание
					Kp = tmp;
					e_data.word = Kp;
					uint8_t base = Kp_BASE_ADDR; // базовый адресс
					for(int i = 0; i < 4; i++){
						EEPROM_write(base + i, e_data.data[i]);
					}
					sei(); // включаем прерывание
				}
			}
			break;
		}
		case KI_SETTING:{
			if(btn == BTN_CLICK){
				state = KP_SETTING;
			}
			else if(btn == BTN_HOLD){
				state = WORK;
				clear_ldc();
			}
			if((timer - my_timer) > 400){
				my_timer = timer;
				if(blink){
					clear_ldc();
					setpos(13,0);
					str_lcd(str);
					setpos(0,0);
					str_lcd("Kp:");
					fix_print(Kp);
					setpos(0,1);
					str_lcd("Ki:");
				}
				else{
					clear_ldc();
					setpos(13,0);
					str_lcd(str);
					setpos(0,0);
					str_lcd("Kp:");
					fix_print(Kp);
					setpos(0,1);
					str_lcd("Ki:");
					fix_print(Ki);
				}
				blink = !blink;
			}
			if(dr != 0){
				_iq tmp = encoder_cnt(Ki, dr, step);
				if(tmp > 0) {
					cli(); // отключаем прерывание
					Ki = tmp;
					e_data.word = Ki;
					uint8_t base = Ki_BASE_ADDR;
					for(int i = 0; i < 4; i++){
						EEPROM_write(base + i, e_data.data[i]);
					}
					sei(); // включаем прерывание
				}
			}
			break;
		}
		
	}
}


int main(void)
{
	ADC_init();	// Настраивам АЦП на работу с каналом PC3
	encoder_init();
    I2C_init();
	LCD_ini();
	lcd_led_off();
	init_timer1();
	pwm1_init();
	_delay_ms(1000);
	uint8_t status = EEPROM_read(0);
	if(status == 0xFF){
		EEPROM_write(0, 1);
		e_data.word = Kp;
		uint8_t base = Kp_BASE_ADDR; // базовый адресс
		for(int i = 0; i < 4; i++){
			EEPROM_write(base + i, e_data.data[i]);
		}
		e_data.word = Ki;
		base = Ki_BASE_ADDR;
		for(int i = 0; i < 4; i++){
			EEPROM_write(base + i, e_data.data[i]);
		}
	}
	else{
		uint8_t base = Kp_BASE_ADDR; // базовый адресс
		for(int i = 0; i < 4; i++){
			e_data.data[i] = EEPROM_read(base + i);
		}
		Kp = e_data.word;
		
		base = Ki_BASE_ADDR;
		for(int i = 0; i < 4; i++){
			e_data.data[i] = EEPROM_read(base + i);
		}
		Ki = e_data.word;
	}
	sei();
    while(1){
		uint8_t button_state = encoder_button(millis());
		uint8_t dir = encoder_state();
		menu_handler(button_state, dir, millis());
    }
}

ISR(TIMER0_OVF_vect){
	static uint8_t i = 0;
	if(i == 31){
		i = 0;
		timer++;
	} 
	else i++;
}

ISR (TIMER1_CAPT_vect){
	_iq current = _IQ(fileter_handler(&flt, analogRead()));
	current = _IQmpy(current, deg_const);
	//fix_print(current);
	
	_iq error = setpoint - current;
	proportional = _IQmpy(error, Kp);

	//fix_print(proportional);
	
	//integral = integral + _IQmpy(((error + pre_error) / 2), Ki);
	//fix_print(integral);
	if(update_I) integral = integral + _IQmpy(((error + pre_error) / 2), Ki);
	/*if((error > 0 && error < _IQ(80.0)) || (error < 0 && error > _IQ(-20.0))){
		integral = integral + _IQmpy(((error + pre_error) / 2), Ki);
		if(m_abs(integral) > (limMax)) integral = limMax;
	}
	else{
		integral = 0;
	}
	*/
	//if(abs(integral) > (limMax / 2)) integral = limMax / 2;
	
	
	//fix_print(integral);
	derivative = _IQmpy((error - pre_error), Kd);
	
	control = proportional + integral + derivative;
	
	if(control > limMax){
		 control = limMax;
		 update_I = 0;
	}
	else if(control < limMin){
		 control = limMin;
		 update_I = 0;
	}
	else{
		update_I = 1;
	}
	//fileter_handler(&flt_u, control >> _Q);
	//fix_print(control);
	pre_error = error;
	OCR1A = (uint16_t)(control >> _Q);
	
	
}