#include "EEPROM.h"

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR & (1<<EEWE)); // ждем окончания операции с записью 
	EEAR = uiAddress; // записываем адрес в регистор 
	EEDR = ucData; // передаем данные в регистор (1 байт)
	EECR |= (1<<EEMWE); // разрешаем запись
	EECR |= (1<<EEWE);  // пишем
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	 while(EECR & (1<<EEWE)); // ждем окончания операции с записью 
	 EEAR = uiAddress; // записываем адрес в регистор 
	 EECR |= (1<<EERE); // читаем
	 return EEDR; 
}


void EEPROM_write_word(unsigned int uiAddress, uint16_t ucData)
{
	EEPROM_write(uiAddress, (unsigned char) ucData);
	unsigned char dt = (ucData>>8);
	EEPROM_write((uiAddress + 1), dt);
}

uint16_t EEPROM_read_word(unsigned int uiAddress)
{
	 uint16_t dt = EEPROM_read((uiAddress + 1)) * 256;
	_delay_ms(1);
	dt += EEPROM_read(uiAddress);
	return dt;
}

void EEPROM_write_string(unsigned int uiAddress, char str1[])
{
	wchar_t n;
	for (n=0; str1[n]!='\0'; n++)
		EEPROM_write(uiAddress+n, str1[n]);
}

const char* EEPROM_read_string(unsigned int uiAddress, unsigned int sz)
{
	unsigned int i;
	char* str1;
	str1 = (char*) realloc(NULL,sz);
	for (i=0; i<sz; i++)
		str1[i] = EEPROM_read((uiAddress + i)); 
	return str1;
}