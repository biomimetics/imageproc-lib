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
 * Revisions:
 *  Andrew Pullin   2012-5-28   Centralized I2C machinery.
 *
 * Notes:
 *  - This file is derived from the I2C machinery previously in gyro/xl.
 *  - TODO (apullin): Module configuration might get centralized.
 */

#include "i2c_driver.h"
#include "i2c.h"
#include "generic_typedefs.h"
#include "utils.h"
#include <stdio.h>


//////////////   Private declarations  ////////////////
//void i2cConfig(void);


///////////////   Public functions  //////////////////
//This is the PUBLIC setup function;
//void i2cSetup(void){
//    i2cConfig();
//    //Do any other setup....
//}

void i2cStartTx(unsigned char channel){
    if      (channel == 1) { StartI2C1(); while(I2C1CONbits.SEN); }
    else if (channel == 2) { StartI2C2(); while(I2C2CONbits.SEN); }
}

void i2cEndTx(unsigned char channel){
    if      (channel == 1) { StopI2C1(); while(I2C1CONbits.PEN); }
    else if (channel == 2) { StopI2C2(); while(I2C2CONbits.PEN); }
}

void i2cSendNACK(unsigned char channel){
    if      (channel == 1) { NotAckI2C1(); while(I2C1CONbits.ACKEN); }
    else if (channel == 2) { NotAckI2C2(); while(I2C2CONbits.ACKEN); }
}

unsigned char i2cReceiveByte(unsigned char channel) {
    unsigned char temp;
    if      (channel == 1) { temp = MasterReadI2C1(); }
    else if (channel == 2) { temp = MasterReadI2C2(); }
    return temp;
}

void i2cSendByte(unsigned char channel, unsigned char byte) {
    if      (channel == 1) {
        MasterWriteI2C1(byte);
        while(I2C1STATbits.TRSTAT);
        while(I2C1STATbits.ACKSTAT);
    }
    else if (channel == 2) {
        MasterWriteI2C2(byte);
        while(I2C2STATbits.TRSTAT);
        while(I2C2STATbits.ACKSTAT);
    }
}

unsigned int i2cReadString(unsigned char channel, unsigned length,
                           unsigned char * data, unsigned int data_wait) {
    unsigned int res;
    if      (channel == 1) { res = MastergetsI2C1(length, data, data_wait); }
    else if (channel == 2) { res = MastergetsI2C2(length, data, data_wait); }
    return res;
}

///////////////   Private functions  //////////////////
//void i2cConfig(void) {
//    //Configuration is actually done by each module independently.
//    //This may change in the future.
//}
