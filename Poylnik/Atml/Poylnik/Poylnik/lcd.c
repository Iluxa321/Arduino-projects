#include "lcd.h"

//----------------------------------------------------------
unsigned char port_lcd = 0; // ячейка для хранения данных порта микросхемы
//----------------------------------------------------------

void send_half_byte(unsigned char c)
{
	c <<= 4;
	e1;     // установка линии E в 1
	_delay_us(50);
	port_lcd &= 0b00001111;
	I2C_Send_Byte_ByADDR(port_lcd | c, 0b01001110);  // стираем информацию на информационных входах
	e0; // установка линии E в 0
	_delay_us(50);
}
//----------------------------------------------------------
void send_byte(unsigned char c, unsigned char mode)
{
	if (mode == 0)
	{
		rs0;
	}
	else
	{
		rs1;
	}
	unsigned char hc = 0;
	hc = c >> 4;
	send_half_byte(hc);
	send_half_byte(c);
}
//----------------------------------------------------------
void setpos(unsigned char x, unsigned char y)
{
	switch(y)
	{
		case 0: send_byte(x|0x80,0); break;
		case 1: send_byte((0x40+x)|0x80,0); break;
		case 2: send_byte((0x14+x)|0x80,0); break;
		case 3: send_byte((0x54+x)|0x80,0); break;
	}
}
//----------------------------------------------------------
void sendchar (unsigned char c)
{
	send_byte(c, 1);
}
//----------------------------------------------------------
void LCD_ini(void)
{
	_delay_ms(15); // ждем 15 мс
	send_half_byte(0b00000011);
	_delay_ms(4);
	send_half_byte(0b00000011);
	_delay_us(100);
	send_half_byte(0b00000010);
	_delay_ms(1);
	send_byte(0b00101000, 0);	 // 4битный режим, используем 2 линии (N=1)
	_delay_ms(1);
	send_byte(0b00001000, 0);	// выключаем дисплей (D=0), курсоры не включаем
	_delay_ms(1);
	send_byte(0b00000001, 0);	// команда очистки дисплея
	_delay_ms(1);
	send_byte(0b00000110, 0);   // заставляем курсор двигатся с слева на право
	_delay_ms(1);
	send_byte(0b00001100, 0);   // включаем дисплей (D=1), курсоры не включаем
	_delay_ms(1);
	setled(); // подсветка 
	setwrite(); // запись
}

void lcd_led_on(void){
	setled(); // включить подсветку 
}

void lcd_led_off(void){
	clearled(); // выключить подсветку
}

//----------------------------------------------------------
void clear_ldc()
{
	send_byte(0b00000001, 0);
	_delay_us(1500);
}
//----------------------------------------------------------
void str_lcd(char str1[])
{
	int n = 0;
	for (n = 0; str1[n]!='\0'; n++ )
	{
		sendchar(str1[n]);
	}
}
//----------------------------------------------------------

