//ipi2c.h
#include "i2c.h"
#include "generic_typedefs.h"

#ifndef __IPI2C_H
#define __IPI2C_H

//Public functions
void i2cSetup(void);

void i2cStartTx(unsigned char);
void i2cEndTx(unsigned char);
void i2cSendNACK(unsigned char);
unsigned char i2cReceiveByte(unsigned char);
void i2cSendByte(unsigned char, unsigned char);
unsigned int i2cReadString(unsigned char, unsigned, unsigned char* ,unsigned int);

#endif
