/*
 * Copyright (c) 2012-2013, Regents of the University of California
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
 * Austria Microsystems AS5048B magnetic encoder I2C Interface
 *
 * by Duncan Haldane
 *
 * v.1.0
 *
 * Revisions:
 *  Duncan Haldane      2012-05-15  Initial release
 *  Andrew Pullin       2012-07-05  Ported to use i2c_driver module
 *  Ronald S. Fearing   2012-12-31  Return fractional value, and put all
 *                                  encoder reading in this file, encoders
 *                                  0-3 instead of 1-4 for simplicity.
 *
 * Notes:
 *  - This module uses the I2C1 port for communicating with the AMS
 *    encoder chips
 */

#include "i2c_driver.h"
#include "i2c.h"
#include "ams-enc.h"
#include "utils.h"
#include "settings.h"


#define LSB2ENCDEG      0.0219
#define ENC_I2C_CHAN    1       //Encoder is on I2C channel 1

unsigned int encAddr[8];

EncObj encPos[NUM_ENC];

#define AMS_ENC_IDLE        0
#define AMS_ENC_WRITE_START 1
#define AMS_ENC_WRITE_ADDR  2
#define AMS_ENC_WRITE_REG   3
#define AMS_ENC_WRITE_STOP  4
#define AMS_ENC_READ_START  5
#define AMS_ENC_READ_ADDR   6
#define AMS_ENC_READ_0      7
#define AMS_ENC_READ_0_ACK  8
#define AMS_ENC_READ_1      9
#define AMS_ENC_READ_1_NACK 10
#define AMS_ENC_READ_STOP   11

#define AMS_ENC_ANGLE_REG   0xFE

#ifndef AMS_ENC_OFFSET_0
#define AMS_ENC_OFFSET_0 0
#endif

#ifndef AMS_ENC_OFFSET_1
#define AMS_ENC_OFFSET_1 0
#endif

volatile unsigned char  state = AMS_ENC_IDLE;
volatile unsigned char  encoder_number = 0;
volatile unsigned int   encoder_new_pos;

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static inline void encoderSetupPeripheral(void);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void amsEncoderSetup(void) {
    // Need delay for encoders to be ready
    delay_ms(100);
    
    // LSB = R/W* .
    // 1. send slave <a2:a1>, a0=W (write reg address)
    // 2. send slave register address <a7:a0>,
    // 3. send slave <a2:a1>, a0=R (read reg data)
    // 4. read slave data <a7:a0>
    encAddr[0] = 0b10000001;        //Encoder 0 rd;wr A1, A2 = low
    encAddr[1] = 0b10000000;        // write

    encAddr[2] = 0b10000011;        //Encoder 1 rd;wr A2 = low, A1 = high
    encAddr[3] = 0b10000010;

    encAddr[4] = 0b10000101;        //Encoder 2 rd;wr A2 = high, A1 = low
    encAddr[5] = 0b10000100;

    encAddr[6] = 0b10000111;        //Encoder 3 rd;wr A2, A1 = high
    encAddr[7] = 0b10000110;

    encoder_number = 0;
    encoder_new_pos = 0;
    state = AMS_ENC_IDLE;

    //setup I2C port I2C1
    encoderSetupPeripheral();

    amsEncoderResetPos();
}

void amsEncoderResetPos(void) {
    // initialize structure
    int i;
    for(i = 0; i< NUM_ENC; i++) {
        //amsEncoderBlockingRead(i);    // get initial values w/o setting oticks
        //encPos[i].offset = encPos[i].pos; // initialize encoder
        encPos[i].oticks = 0;   // set revolution counter to 0
    }
    encPos[0].offset = AMS_ENC_OFFSET_0;
    encPos[1].offset = AMS_ENC_OFFSET_1;
}

/*****************************************************************************
 * Function Name : encoderSetupPeripheral
 * Description   : Setup I2C for encoders
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
static inline void encoderSetupPeripheral(void) { //same setup as ITG3200 for compatibility
    unsigned int I2C1CONvalue, I2C1BRGvalue;
    I2C1CONvalue = I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &
            I2C1_IPMI_DIS & I2C1_7BIT_ADD & I2C1_SLW_DIS &
            I2C1_SM_DIS & I2C1_GCALL_DIS & I2C1_STR_DIS &
            I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
            I2C1_STOP_DIS & I2C1_RESTART_DIS & I2C1_START_DIS &
            MI2C1_INT_PRI_6;

    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 909KHz
    I2C1BRGvalue = 40;
    OpenI2C1(I2C1CONvalue, I2C1BRGvalue);
    SetPriorityIntMI2C1(6);
    _MI2C1IF = 0;
    EnableIntMI2C1;
}

// Blocking encoder read. This will fail if the encoders are in a weird state
void amsEncoderBlockingRead(unsigned char num) {
    unsigned char enc_data[2];

    CRITICAL_SECTION_START
    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, encAddr[2*num+1]);    //Write address
    i2cSendByte(ENC_I2C_CHAN, AMS_ENC_ANGLE_REG);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, encAddr[2*num]);      //Read address
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);
    IdleI2C1();
    _MI2C1IF = 0;
    CRITICAL_SECTION_END

    amsEncoderUpdatePos(num,((enc_data[1] << 6)+(enc_data[0] & 0x3F)));
}

//kick off asynchrounous read of both encoders
unsigned char amsEncoderStartAsyncRead(void) {
    if(state == AMS_ENC_IDLE) {
        encoder_number = 0;
        encoder_new_pos = 0;
        state = AMS_ENC_WRITE_START;
        I2C1CONbits.SEN = 1;
        return 1;
    } else {
        return 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void) {
    LED_3 = 1;

    switch(state) {
        case AMS_ENC_WRITE_START:
            I2C1TRN = encAddr[2*encoder_number+1];
            state = AMS_ENC_WRITE_ADDR;
            break;
        case AMS_ENC_WRITE_ADDR:
            I2C1TRN = AMS_ENC_ANGLE_REG;
            state = AMS_ENC_WRITE_REG;
            break;
        case AMS_ENC_WRITE_REG:
            I2C1CONbits.RSEN = 1;
            state = AMS_ENC_READ_START;
            break;
        case AMS_ENC_READ_START:
            I2C1TRN = encAddr[2*encoder_number];
            state = AMS_ENC_READ_ADDR;
            break;
        case AMS_ENC_READ_ADDR:
            I2C1CONbits.RCEN = 1;
            state = AMS_ENC_READ_0;
            break;
        case AMS_ENC_READ_0:
            encoder_new_pos = I2C1RCV <<6;
            I2C1CONbits.ACKDT = 0;
            I2C1CONbits.ACKEN = 1;
            state = AMS_ENC_READ_0_ACK;
            break;
        case AMS_ENC_READ_0_ACK:
            I2C1CONbits.RCEN = 1;
            state = AMS_ENC_READ_1;
            break;
        case AMS_ENC_READ_1:
            encoder_new_pos |= (I2C1RCV &  0x3F);
            I2C1CONbits.ACKDT = 1;
            I2C1CONbits.ACKEN = 1;
            state = AMS_ENC_READ_1_NACK;
            break;
        case AMS_ENC_READ_1_NACK:
            I2C1CONbits.PEN = 1;
            state = AMS_ENC_READ_STOP;
            break;
        case AMS_ENC_READ_STOP:
            amsEncoderUpdatePos(encoder_number,encoder_new_pos);

            if(++encoder_number < NUM_ENC) {
                I2C1CONbits.SEN = 1;
                state = AMS_ENC_WRITE_START;
            } else {
                state = AMS_ENC_IDLE;
            }
            break;
        default:
            state = AMS_ENC_IDLE;
            break;
    }
    LED_3 = 0;
    _MI2C1IF = 0;
}

inline void amsEncoderUpdatePos(unsigned char encoder_number, unsigned int new_pos) {
    unsigned int prev_pos = encPos[encoder_number].pos;
    if (new_pos > prev_pos) {
        if( (new_pos-prev_pos) > MAX_HALL/2 ) {//Subtract one Rev count if diff > 180
            encPos[encoder_number].oticks--;
        }
    } else {
        if( (prev_pos-new_pos) > MAX_HALL/2 ) {//Add one Rev count if -diff > 180
            encPos[encoder_number].oticks++;
        }
    }
    encPos[encoder_number].pos = new_pos;
}

float amsEncoderGetFloatPos(unsigned char num) {
    float pos;
    pos = encPos[num].pos* LSB2ENCDEG; //calculate Float
    return pos;
}
