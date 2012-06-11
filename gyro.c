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
 * InvenSense ITG-3200 3-axis MEMS gyro IC Interface
 *
 * by Stanley S. Baek
 *
 * v.1.0
 *
 * Revisions:
 *  Stanley S. Baek      2010-05-30    Initial release
 *
 * Notes:
 *  - This module uses an I2C port for communicating with the gyroscope chip
 */

#include "ports.h"      // for external interrupt
#include "i2c.h"
#include "gyro.h"
#include "utils.h"

#define GYRO_ADDR_RD        0b11010001
#define GYRO_ADDR_WR        0b11010000

#define LSB2DEG             0.0695652174        // 14.375 LSB/(deg/s)
#define LSB2RAD             0.00121414209


/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/
// data storage for receiving data
static union gyrodata {
    unsigned char chr_data[8];
    int int_data[4];
} GyroData;


// calibration parameters for gyroscope
static union {
    float fdata[3];
    unsigned char cdata[12];
} GyroOffset;

int offsets[3];


/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static void gyroWrite(unsigned char regaddr, unsigned char data );
static void gyroHandleISR(void);
static inline unsigned int gyroReadString(unsigned length, unsigned char * data,
                                   unsigned int data_wait);
static inline void gyroSendByte( unsigned char byte );
static inline unsigned char gyroReceiveByte(void);
static inline void gyroSendNACK(void);
static inline void gyroStartTx(void);
static inline void gyroEndTx(void);
static inline void gyroSetupPeripheral(void);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/


void gyroSetup(void) {

    // setup I2C port
    gyroSetupPeripheral();

    // external interrupt configuration.
    // it is NOT USED at this moment.
    ConfigINT1(RISING_EDGE_INT & EXT_INT_DISABLE & EXT_INT_PRI_3);

    delay_ms(25);   // power up delay, may not need...
    //gyroWrite(0x16, 0x1A);  // 2000 deg/sec, 1 kHz Sampling rate, 98Hz LPF
    gyroWrite(0x16, 0x19);  // 2000 deg/sec, 1 kHz Sampling rate, 196Hz LPF
    gyroWrite(0x17, 0x00);  // interrupt disabled
    gyroWrite(0x3e, 0x03);
    delay_ms(1);   // PLL Settling time

    gyroRunCalib(300);  // quick calibration. better to run this with > 1000.
}

void gyroSetSampleRate(unsigned char rate) {
    gyroWrite(0x16, rate);
}

void gyroSetIntEn(unsigned char flag) {
    _INT1IE = flag;
}

void gyroSleep(void) {
    gyroWrite(0x3e, 0x78);
}

void gyroWake(void) {
    gyroWrite(0x3e, 0x03);
}

unsigned char* gyroGetCalibParam(void) {
    return GyroOffset.cdata;
}

void gyroGetOffsets(int* data){
    data[0] = offsets[0];
    data[1] = offsets[1];
    data[2] = offsets[2];
}

void gyroRunCalib(unsigned int count){

    unsigned int i;
    long x, y, z;
    x = 0;
    y = 0;
    z = 0;

    delay_ms(6); //From datasheet, standard settling time

    // throw away first 200 data. Sometimes they are bad at the beginning.
    for (i = 0; i < 200; ++i) {
        gyroReadXYZ();
        delay_us(100);
    }

    for (i = 0; i < count; i++) {
        gyroReadXYZ();
        x += GyroData.int_data[1];
        y += GyroData.int_data[2];
        z += GyroData.int_data[3];
        delay_us(200);
    }

    offsets[0] = x/count;
    offsets[1] = y/count;
    offsets[2] = z/count;

    GyroOffset.fdata[0] = 1.0*x/count;
    GyroOffset.fdata[1] = 1.0*y/count;
    GyroOffset.fdata[2] = 1.0*z/count;
}

float gyroGetFloatTemp(void) {
    int x;
    x = gyroGetIntTemp();
    return (35 + (x+13200)/280.0);
}

int gyroGetIntTemp(void) {
    return GyroData.int_data[0];
}

void gyroReadTemp(void) {
    unsigned char temp_data[2];

    gyroStartTx();
    gyroSendByte(GYRO_ADDR_WR);
    gyroSendByte(0x1b);
    gyroEndTx();
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_RD);
    gyroReadString(2, temp_data, 1000);
    gyroEndTx();

    GyroData.chr_data[0] = temp_data[1];
    GyroData.chr_data[1] = temp_data[0];
}


void gyroGetRadXYZ(float* data) {
    unsigned char i;
    for (i = 0; i < 3; ++i) {
        data[i] = (GyroData.int_data[i+1] - GyroOffset.fdata[i])*LSB2RAD;
    }
}

float gyroGetRadX(void) {
    return (GyroData.int_data[1] - GyroOffset.fdata[0])*LSB2RAD;
}

float gyroGetRadY(void) {
    return (GyroData.int_data[2] - GyroOffset.fdata[1])*LSB2RAD;
}

float gyroGetRadZ(void) {
    return (GyroData.int_data[3] - GyroOffset.fdata[2])*LSB2RAD;
}

void gyroGetDegXYZ(float* data) {
    unsigned char i;
    for (i = 0; i < 3; ++i) {
        data[i] = (GyroData.int_data[i+1] - GyroOffset.fdata[i])*LSB2DEG;
    }
}

unsigned char* gyroToString(void) {
    return GyroData.chr_data + 2;
}

void gyroDumpData(unsigned char* buffer) {

    int i;
    for (i = 0; i < 6; i++) {
        buffer[i] = GyroData.chr_data[2+i];
    }
}


unsigned char* gyroReadXYZ(void) {
    unsigned char gyro_data[6];
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_WR);
    gyroSendByte(0x1d);
    gyroEndTx();
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_RD);
    gyroReadString(6, gyro_data, 1000);
    gyroEndTx();

    GyroData.chr_data[2] = gyro_data[1];
    GyroData.chr_data[3] = gyro_data[0];
    GyroData.chr_data[4] = gyro_data[3];
    GyroData.chr_data[5] = gyro_data[2];
    GyroData.chr_data[6] = gyro_data[5];
    GyroData.chr_data[7] = gyro_data[4];

    return GyroData.chr_data + 2;
}

void gyroGetXYZ(unsigned char *data) {

    unsigned char gyro_data[6];
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_WR);
    gyroSendByte(0x1d);
    gyroEndTx();
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_RD);
    gyroReadString(6, gyro_data, 1000);
    gyroEndTx();

    data[0] = gyro_data[1];
    data[1] = gyro_data[0];
    data[2] = gyro_data[3];
    data[3] = gyro_data[2];
    data[4] = gyro_data[5];
    data[5] = gyro_data[4];

}


/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/******************************************************************************
 * Interrupt handler for Gyroscope
 * ****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    gyroHandleISR();
    _INT1IF = 0;    // Clear the interrupt flag
}

/*****************************************************************************
* Function Name : gyroHandleISR
* Description   : Need to implement this function if interrupt is used.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void gyroHandleISR(void) {
    return;
}


/*****************************************************************************
* Function Name : gyroWrite
* Description   : Write a data to a register
* Parameters    : regaddr - address of register
*                 data - value to be written to the register
* Return Value  : None
*****************************************************************************/
static void gyroWrite(unsigned char regaddr, unsigned char data ){
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_WR);
    gyroSendByte(regaddr);
    gyroSendByte(data);
    gyroEndTx();
}

/*****************************************************************************
* Function Name : gyroReadString
* Description   : It reads predetermined data string length from the I2C bus.
* Parameters    : length is the string length to read
*                 data is the storage for received gyro data
*                 data_wait is the timeout value
* Return Value  : Number of bytes read before timeout.
*****************************************************************************/
static inline unsigned int gyroReadString(unsigned length, unsigned char * data,
                                   unsigned int data_wait) {
    return MastergetsI2C2(length, data, data_wait);
}


/*****************************************************************************
* Function Name : gyroSendByte
* Description   : Send a byte to gyroscope
* Parameters    : byte - a byte to send
* Return Value  : None
*****************************************************************************/
static inline void gyroSendByte( unsigned char byte ) {
    MasterWriteI2C2(byte);
    while(I2C2STATbits.TRSTAT);
    while(I2C2STATbits.ACKSTAT);
}

/*****************************************************************************
* Function Name : gyroReceiveByte
* Description   : Receive a byte from gyroscope
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static inline unsigned char gyroReceiveByte(void) {
    return MasterReadI2C2();
}

/*****************************************************************************
* Function Name : gyroSendNACK
* Description   : Send NACK to gyroscope
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static inline void gyroSendNACK(void){
    NotAckI2C2();
    while(I2C2CONbits.ACKEN);
}

/*****************************************************************************
* Function Name : gyroStartTx
* Description   : Start I2C transmission
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static inline void gyroStartTx(void){
    StartI2C2();
    while(I2C2CONbits.SEN);
}

/*****************************************************************************
* Function Name : gyroEndTx
* Description   : End I2C transmission
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static inline void gyroEndTx(void){
    StopI2C2();
    while(I2C2CONbits.PEN);
}

/*****************************************************************************
* Function Name : gyroSetupPeripheral
* Description   : Setup I2C for gyroscope
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static inline void gyroSetupPeripheral(void) {
    unsigned int I2C2CONvalue, I2C2BRGvalue;
    I2C2CONvalue = I2C2_ON & I2C2_IDLE_CON & I2C2_CLK_HLD &
                   I2C2_IPMI_DIS & I2C2_7BIT_ADD & I2C2_SLW_DIS &
                   I2C2_SM_DIS & I2C2_GCALL_DIS & I2C2_STR_DIS &
                   I2C2_NACK & I2C2_ACK_DIS & I2C2_RCV_DIS &
                   I2C2_STOP_DIS & I2C2_RESTART_DIS & I2C2_START_DIS;

    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 400KHz
    I2C2BRGvalue = 95;
    OpenI2C2(I2C2CONvalue, I2C2BRGvalue);
    IdleI2C2();
}
