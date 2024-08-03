#include "ADC.h"


void ADC_init()
{
	ADCSRA = 0x00; // очищаем регистор
	ADCSRA |= (1<<ADEN); // включаем АЦП
	ADCSRA |= (1<<ADPS1) | (1<<ADPS0); // предделитель = 8 (1МГц)
	ADCSRA &= ~(1<<ADPS2);
	//ADMUX |= (1<<ADLAR); // считываем только старший регистор (8 бит)
	ADMUX |= (1<<MUX0) | (1<<MUX1) | (1<<REFS0); // порт 3, питание ацп от внешнего 5В
	ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<REFS1));
	ADCSRA |= (1<<ADSC);
}

uint16_t analogRead()
{
	uint16_t tmp = 0;
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	tmp = ADCL;
	tmp = (tmp) | (uint16_t)(ADCH << 8);
	return tmp;
}

