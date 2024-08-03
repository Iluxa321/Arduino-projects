#include "I2C.h"

void I2C_init()
{
	TWBR = 0x20; // скорость передачи 100 кГц
}

void I2C_Start_Condition(void)
{
	
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while ( !(TWCR & (1<<TWINT)) ); // подождем пока утановится TWIN
}

void I2C_Stop_Condition(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

void I2C_Send_Byte(unsigned char c)
{
	TWDR = c; // запишем байт в регистор данных
	TWCR = (1<<TWINT) |  (1<<TWEN); // вклбчим перелачу данных
	while ( !(TWCR & (1<<TWINT)) ); 
}

void I2C_Send_Byte_ByADDR(unsigned char c, unsigned char addr)
{
	I2C_Start_Condition();
	I2C_Send_Byte(addr);
	I2C_Send_Byte(c);
	I2C_Stop_Condition();	
}