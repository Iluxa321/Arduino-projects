


#ifndef I2C_H_
#define I2C_H_

#include "main.h"

void I2C_init();
void I2C_Start_Condition(void);
void I2C_Stop_Condition(void);
void I2C_Send_Byte(unsigned char c);
void I2C_Send_Byte_ByADDR(unsigned char c, unsigned char addr);



#endif /* I2C_H_ */