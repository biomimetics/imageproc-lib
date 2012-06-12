/*
 * Copyright (c) 2010, Regents of the University of California
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
 * Analog Devices ADXL345 3-axis MEMS accelerometer IC Interface
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2010-06-05    Initial release
 *
 * Notes:
 *  - Uses an I2C port for communicating with the accelerometer chip
 *  - MCU resources requied for this module:
 *      I2C bus (I2C1 for ImageProc2) - SCL1 & SDA1
 *      External INT (INT3 for ImageProc2)
 */


#include "ports.h"      // for external interrupt
#include "i2c_driver.h"
#include "i2c.h"        // kept only for peripheral setup
#include "xl.h"
#include "utils.h"

#define XL_ADDR_RD             0xA7
#define XL_ADDR_WR             0xA6
#define XL_DEFAULT_SCALE       0.03832  // = 9.81/256
#define XL_I2C_CHAN            1


/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/
// data storage for receiving data
static union {
    unsigned char chr_data[6];
    int int_data[3];
} XlData;


static union {
    unsigned char chr_data[24];
    float f_data[6];
} CalibParam;


/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static void xlWrite(unsigned char regaddr, unsigned char data);
static inline void xlSetupPeripheral(void);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void xlSetup(void) {

    xlSetupPeripheral();

    ConfigINT3(RISING_EDGE_INT & EXT_INT_DISABLE & EXT_INT_PRI_3);

    delay_ms(25);   // power up delay, may not need...

    // Do not use offset registers, but use calib_param_
    xlWrite(0x1e, 0x00); // x-axis offset is zero
    xlWrite(0x1f, 0x00); // y-axis offset is zero
    xlWrite(0x20, 0x00); // z-axis offset is zero

    xlWrite(0x21, 0x00); // disable the single/double tap functions
    xlWrite(0x22, 0x00); // disable the double tap function
    xlWrite(0x23, 0x00); // disable the double tap function
    xlWrite(0x27, 0x00); // disable the activity/inactivity
    xlWrite(0x2a, 0x00); // disable single/double tap fuctions

    xlWrite(0x2c, 0x0d); // normal power mode & 800Hz output rate
    xlWrite(0x2d, 0x08); // normal mode

    //interrupt enable/disable
    xlWrite(0x2e, 0x80); // DATA_READY is enabled. others are disabled
    xlWrite(0x2f, 0x00); // all interrputs are sent to INT1 Pin

    // data format
    // enable the self-test force
    // active high for interrupts
    // full resolution mode
    // right justified mode
    // +-8g range
    xlWrite(0x31, 0b00001010);

    xlWrite(0x38, 0x00); // FIFO control is bypassed

    // set default scale factors
    CalibParam.f_data[0] = XL_DEFAULT_SCALE;
    CalibParam.f_data[1] = XL_DEFAULT_SCALE;
    CalibParam.f_data[2] = XL_DEFAULT_SCALE;

}


void xlSetIntEn(unsigned char flag) {
    _INT3IE = flag;
}


void xlSetRange(unsigned char range) {
    if (range == 2) {
        xlWrite(0x31, 0b00001000);
    } else if (range == 4) {
        xlWrite(0x31, 0b00001001);
    } else if (range == 8) {
        xlWrite(0x31, 0b00001010);
    } else if (range == 16) {
        xlWrite(0x31, 0b00001011);
    }
    // else do not change anything
}


void xlSetOutputRate(unsigned char power_mode, unsigned char rate) {
    if (power_mode == 0) {
        xlWrite(0x2c, (0x0f & rate));
    } else {
        xlWrite(0x2c, ((0x0f & rate) | 0x10));
    }
}


void xlSleep(void) {
    xlWrite(0x2d, 0x00);    // stanby mode, clear Measure bit
}


void xlWake(void) {
    xlWrite(0x2d, 0x08);    // normal measurement mode, set Measure bit
}

void xlSetCalibParam(unsigned char* param) {
    unsigned char i;

    for(i = 0; i < 24; ++i) {
        CalibParam.chr_data[i] = param[i];
    }
}

unsigned char* xlGetCalibParam(void) {
    return CalibParam.chr_data;
}


void xlLoadCalibParam(void) {
    // TODO: Implement !!!!!
   // work harder !!
}


void xlSaveCalibParam(void){
    // TODO: Implement !!!!!
   // work harder !!
}

unsigned char xlGetID(void) {
    unsigned char c;

    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN, XL_ADDR_WR);
    i2cSendByte(XL_I2C_CHAN, 0x00);
    i2cEndTx(XL_I2C_CHAN);
    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN, XL_ADDR_RD);
    c = i2cReceiveByte(XL_I2C_CHAN);
    i2cEndTx(XL_I2C_CHAN);

    return c;
}

void xlGetFloatXYZ(float* data){
    data[0] = XlData.int_data[0]*CalibParam.f_data[0] + CalibParam.f_data[3];
    data[1] = XlData.int_data[1]*CalibParam.f_data[1] + CalibParam.f_data[4];
    data[2] = XlData.int_data[2]*CalibParam.f_data[2] + CalibParam.f_data[5];
}

int* xlGetIntXYZ(void){
    return XlData.int_data;
}

unsigned char* xlToString(void) {
    return XlData.chr_data;
}

void xlDumpData(unsigned char* buffer) {
    int i;
    for (i = 0; i < 6; i++) {
        buffer[i] = XlData.chr_data[i];
    }
}

unsigned char* xlReadXYZ(void)  {
    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN, XL_ADDR_WR);
    i2cSendByte(XL_I2C_CHAN, 0x32);
    i2cEndTx(XL_I2C_CHAN);
    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN, XL_ADDR_RD);
    i2cReadString(XL_I2C_CHAN, 6, XlData.chr_data, 1000);
    i2cEndTx(XL_I2C_CHAN);

    return XlData.chr_data;
}

void xlGetXYZ(unsigned char *data)  {
    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN,XL_ADDR_WR);
    i2cSendByte(XL_I2C_CHAN,0x32);
    i2cEndTx(XL_I2C_CHAN);
    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN, XL_ADDR_RD);
    i2cReadString(XL_I2C_CHAN, 6, data, 1000);
    i2cEndTx(XL_I2C_CHAN);
}


/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/**************************************************
 * Interrupt hander for Accelerometer
 * ************************************************/
void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void) {

    _INT3IF = 0;    // Clear the interrupt flag
}

/*****************************************************************************
* Function Name : xlWrite
* Description   : Write a data to a register
* Parameters    : regaddr - address of register
*                 data - value to be written to the register
* Return Value  : None
*****************************************************************************/
static void xlWrite(unsigned char regaddr, unsigned char data ){
    i2cStartTx(XL_I2C_CHAN);
    i2cSendByte(XL_I2C_CHAN, XL_ADDR_WR);
    i2cSendByte(XL_I2C_CHAN, regaddr);
    i2cSendByte(XL_I2C_CHAN, data);
    i2cEndTx(XL_I2C_CHAN);
}

/******************************************************************************
* Function Name : xlSetupPeripheral
* Description   : This routine sets up I2C bus for this module
* Parameters    : None
* Return Value  : None
*******************************************************************************/
static inline void xlSetupPeripheral(void) {
    unsigned int I2C1CONvalue, I2C1BRGvalue;
    I2C1CONvalue = I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &
                   I2C1_IPMI_DIS & I2C1_7BIT_ADD & I2C1_SLW_DIS &
                   I2C1_SM_DIS & I2C1_GCALL_DIS & I2C1_STR_DIS &
                   I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
                   I2C1_STOP_DIS & I2C1_RESTART_DIS & I2C1_START_DIS;

    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 400KHz
    //Maximum ADC data rate of 800 Hz
    I2C1BRGvalue = 95;
    OpenI2C1(I2C1CONvalue, I2C1BRGvalue);
    IdleI2C1();
}
