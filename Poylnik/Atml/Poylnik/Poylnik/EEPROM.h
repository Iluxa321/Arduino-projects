


#ifndef EEPROM_H_
#define EEPROM_H_

#include "main.h"

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void EEPROM_write_word(unsigned int uiAddress,  uint16_t ucData);
uint16_t EEPROM_read_word(unsigned int uiAddress);
void EEPROM_write_string(unsigned int uiAddress, char str1[]);
const char* EEPROM_read_string(unsigned int uiAddress, unsigned int sz);

#endif /* EEPROM_H_ */