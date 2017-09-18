#ifndef _ACC_H_
#define _ACC_H_

#include "mbed.h"
#include "BufferedSerial.h"

int ACC_init(BufferedSerial* uart, I2C *i2c);
int ACC_Config(uint8_t reg, uint8_t val);
uint8_t ACC_Read_Status(void);
int ACC_Read_Axis(uint8_t* buffer);

#endif //_ACC_H_
