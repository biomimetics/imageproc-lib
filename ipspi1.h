/******************************************************************************
* Name: ipspi1.h
* Desc: Header file for basic spi read/write functionality
* Date: 2010-08-10
* Author: stanbaek
*
* UNDER DEVELOPMENT == UNDER DEVELOPMENT == UNDER DEVELOPMENT
*
* TODO: 
*  1. Implement DMA for SPI.
*
* SPI1 is used for AT86RF231
* SPI2 should be used if you run this module on
* MikroElektronika dev board because RB2 is used for LCD.
******************************************************************************/

#include "spi.h"
#include "generic_typedefs.h"

#ifndef __IPSPI_H
#define __IPSPI_H

void ipspi1Config(void);
byte ipspi1GetByte(void);
byte ipspi1PutByte(byte dout);
void ipspi1ChipSelect(byte select);

#endif
