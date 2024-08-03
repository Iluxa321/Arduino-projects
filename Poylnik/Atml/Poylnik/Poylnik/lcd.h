
#ifndef LCD_H_
#define LCD_H_

#include "main.h"

void LCD_ini(void);
void lcd_led_on(void);
void lcd_led_off(void);
void setpos(unsigned char x, unsigned char y);
void str_lcd(char str1[]);
void clear_ldc();
void sendchar (unsigned char c);


//----------------------------------------------------------
#define e1    I2C_Send_Byte_ByADDR(port_lcd|=0x04, 0b01001110) // установка линии E в 1
#define e0    I2C_Send_Byte_ByADDR(port_lcd &= ~0x04, 0b01001110) // установка линии E в 0
#define rs1    I2C_Send_Byte_ByADDR(port_lcd |= 0x01, 0b01001110) // установка линии RS в 1
#define rs0    I2C_Send_Byte_ByADDR(port_lcd &= ~0x01, 0b01001110) // установка линии RS в 0
#define setled()    I2C_Send_Byte_ByADDR(port_lcd |= 0x08, 0b01001110) // включение подсветки
#define clearled()	I2C_Send_Byte_ByADDR(port_lcd &= ~0x08, 0b01001110) // выключение подсветки
#define setwrite()   I2C_Send_Byte_ByADDR(port_lcd &= ~0x02, 0b01001110) // установка записи в память дисплея
//----------------------------------------------------------

#endif 