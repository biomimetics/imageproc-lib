/*
 * Copyright (c) 2012, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * I2C driver
 *
 * by Andrew Pullin
 *
 * v.0.1
 *
 * Note: For functions in which the I2C channel is specified, if the value
 * passed for the channel number is anything other than 1, then channel 2
 * will be used by default.
 */

#ifndef __I2C_DRIVER_H
#define __I2C_DRIVER_H


/***********************************************************************
* Function Name : i2cSetup
* Description   : Setup I2C transmission
* Parameters    : None
* Return Value  : None
***********************************************************************/
//void i2cSetup(void);

/***********************************************************************
* Function Name : i2cStartTx
* Description   : Start I2C transmission on a specified I2C channel
* Parameters    : channel is the channel number
* Return Value  : None
***********************************************************************/
void i2cStartTx(unsigned char channel);

/***********************************************************************
* Function Name : i2cEndTx
* Description   : End I2C transmission on a specified I2C channel
* Parameters    : channel is the channel number
* Return Value  : None
***********************************************************************/
void i2cEndTx(unsigned char channel);

/***********************************************************************
* Function Name : i2cSendNACK
* Description   : Send NACK to a specified I2C channel
* Parameters    : channel is the channel number
* Return Value  : None
***********************************************************************/
void i2cSendNACK(unsigned char channel);

/***********************************************************************
* Function Name : i2cReceiveByte
* Description   : Receive a byte from a specified I2C channel
* Parameters    : channel is the channel number
* Return Value  : None
***********************************************************************/
unsigned char i2cReceiveByte(unsigned char channel);

/***********************************************************************
* Function Name : i2cSendByte
* Description   : Send a byte to a specified I2C channel
* Parameters    : channel is the channel number
*                 byte - a byte to send
* Return Value  : None
***********************************************************************/
void i2cSendByte(unsigned char channel, unsigned char byte);

/***********************************************************************
* Function Name : i2cReadString
* Description   : It reads a predetermined data string from the I2C bus.
* Parameters    : channel is the I2C channel to read from
*                 length is the string length to read
*                 data is the storage for received gyro data
*                 data_wait is the timeout value
* Return Value  : Number of bytes read before timeout.
************************************************************************/
unsigned int i2cReadString(unsigned char channel, unsigned length,
                           unsigned char * data, unsigned int data_wait);


#endif // __I2C_DRIVER_H
