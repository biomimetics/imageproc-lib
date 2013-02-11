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
 *  Stanley S. Baek     2010-05-30      Initial release
 *  Humphrey Hu         2012-06-27      Refactored, added dead zone
 *
 * Notes:
 *  - This module uses an I2C port for communicating with the gyroscope chip
 */

#include "ports.h"      // for external interrupt
#include "i2c.h"
#include "gyro.h"
#include "utils.h"
#include "string.h"

// Register parameters
#define GYRO_ADDR_RD        (0b11010001)
#define GYRO_ADDR_WR        (0b11010000)

// Register addresses
#define REG_WHO_AM_I            (0x00)
#define REG_SMPLRT_DIV          (0x15)
#define REG_DLPF_FS             (0x16)
#define REG_INT_CFG             (0x17)
#define REG_INT_STATUS          (0x1A)
#define REG_TEMP_OUT_H          (0x1B)
#define REG_TEMP_OUT_L          (0x1C)
#define REG_GYRO_XOUT_H         (0x1D)
#define REG_GYRO_XOUT_L         (0x1E)
#define REG_GYRO_YOUT_H         (0x1F)
#define REG_GYRO_YOUT_L         (0x20)
#define REG_GYRO_ZOUT_H         (0x21)
#define REG_GYRO_ZOUT_L         (0x22)
#define REG_PWR_MGM             (0x3E)

#define LSB2DEG             (0.0695652174)  // 14.375 LSB/(deg/s)
#define LSB2RAD             (0.00121414209)

// Other parameters
#define DEFAULT_DEAD_ZONE   (3)             // Initial dead zone (0 disabled)
#define INITIAL_CALIB_NUM   (200)           // Initial calibration samples
#define I2C_TIMEOUT_BYTES   (200)           // Number of bytes for I2C timeout

/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/
static unsigned char is_ready = 0;

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

static int offsets[3]; // Integer gyro offsets
static int dead_zone; // Dead zone cutoff

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
static int applyDeadZone(int val);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void gyroSetup(void) {

    // setup I2C port
    gyroSetupPeripheral();
    
    // external interrupt configuration.
    // it is NOT USED at this moment.
    ConfigINT1(RISING_EDGE_INT & EXT_INT_DISABLE & EXT_INT_PRI_3);

    dead_zone = DEFAULT_DEAD_ZONE;
    
    delay_ms(25);   // power up delay, may not need...
    gyroReset();
    delay_ms(10);
    //gyroWrite(REG_DLPF_FS, 0x1A);  // 2000 deg/sec, 1 kHz Sampling rate, 98Hz LPF
    gyroWrite(REG_DLPF_FS, 0x19);  // 2000 deg/sec, 1 kHz Sampling rate, 196Hz LPF
    gyroWrite(REG_INT_CFG, 0x00);  // interrupt disabled
    gyroWrite(REG_PWR_MGM, 0x03);  // Set power management?
    delay_ms(1);   // PLL Settling time
    
    is_ready = 1;
    
    delay_ms(6); //From datasheet, standard settling time    
    gyroRunCalib(INITIAL_CALIB_NUM);  // quick calibration
    
}

void gyroSetDeadZone(int cutoff) {

    if(cutoff < 0) { cutoff = -cutoff; }    // Correct negative inputs
    dead_zone = cutoff;

}

void gyroSetSampleRate(unsigned char rate) {
    gyroWrite(REG_DLPF_FS, rate);
}

void gyroSetIntEn(unsigned char flag) {
    _INT1IE = flag;
}

// TODO: Check these register values for sleep/wake
void gyroSleep(void) {
    gyroWrite(REG_PWR_MGM, 0x78);
}

void gyroReset(void){
    gyroWrite(REG_PWR_MGM, 0x80); //Write H_RESET bit
}

void gyroWake(void) {
    gyroWrite(REG_PWR_MGM, 0x03);
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
    long x_acc, y_acc, z_acc;
    
    x_acc = 0;
    y_acc = 0;
    z_acc = 0;

    CRITICAL_SECTION_START
            
    // throw away first 200 data. Sometimes they are bad at the beginning.
    for (i = 0; i < 300; ++i) {
        gyroReadXYZ();
        delay_us(100);
    }

    for (i = 0; i < count; i++) {
        gyroReadXYZ();
        x_acc += GyroData.int_data[1];
        y_acc += GyroData.int_data[2];
        z_acc += GyroData.int_data[3];
        delay_ms(1); // Sample at around 1kHz
    }

    CRITICAL_SECTION_END

    offsets[0] = x_acc/count;
    offsets[1] = y_acc/count;
    offsets[2] = z_acc/count;

    GyroOffset.fdata[0] = 1.0*x_acc/count;
    GyroOffset.fdata[1] = 1.0*y_acc/count;
    GyroOffset.fdata[2] = 1.0*z_acc/count;

}

float gyroGetFloatTemp(void) {
    
    return (35 + (gyroGetIntTemp() + 13200) / 280.0);
    
}

int gyroGetIntTemp(void) {

    return GyroData.int_data[0];

}

void gyroReadTemp(void) {

    unsigned char temp_data[2];

    gyroStartTx();
    gyroSendByte(GYRO_ADDR_WR);
    gyroSendByte(REG_TEMP_OUT_H);
    gyroEndTx();
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_RD);
    gyroReadString(2, temp_data, I2C_TIMEOUT_BYTES);
    gyroEndTx();

    GyroData.chr_data[0] = temp_data[1];
    GyroData.chr_data[1] = temp_data[0];
    
}

void gyroGetIntXYZ(int* data) {

    data[0] = gyroGetIntX();
    data[1] = gyroGetIntY();
    data[2] = gyroGetIntZ();
    
}

int gyroGetIntX(void) {

    return applyDeadZone(GyroData.int_data[1] - offsets[0]);

}

int gyroGetIntY(void) {

    return applyDeadZone(GyroData.int_data[2] - offsets[1]);

        }

int gyroGetIntZ(void) {

    return applyDeadZone(GyroData.int_data[3] - offsets[2]);

    }

void gyroGetRadXYZ(float* data) {

    data[0] = gyroGetRadX();
    data[1] = gyroGetRadY();
    data[2] = gyroGetRadZ();
}

float gyroGetRadX(void) {

    return LSB2RAD*gyroGetIntX();
    
}

float gyroGetRadY(void) {

    return LSB2RAD*gyroGetIntY();
    
}

float gyroGetRadZ(void) {

    return LSB2RAD*gyroGetIntZ();
    
}

void gyroGetDegXYZ(float* data) {

    data[0] = gyroGetDegX();
    data[1] = gyroGetDegY();
    data[2] = gyroGetDegZ();
    
}

float gyroGetDegX(void) {

    return LSB2DEG*gyroGetIntX();

        }

float gyroGetDegY(void) {

    return LSB2DEG*gyroGetIntY();

    }

float gyroGetDegZ(void) {

    return LSB2DEG*gyroGetIntZ();

}

unsigned char* gyroToString(void) {

    return GyroData.chr_data + 2;

}

void gyroDumpData(unsigned char* buffer) {

    memcpy(buffer, GyroData.chr_data + 2, 6);    
    
}


unsigned char* gyroReadXYZ(void) {
    
    unsigned char gyro_data[6];
    
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_WR);
    gyroSendByte(0x1d);
    gyroEndTx();
    gyroStartTx();
    gyroSendByte(GYRO_ADDR_RD);
    gyroReadString(6, gyro_data, I2C_TIMEOUT_BYTES);
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

    gyroReadXYZ();
    gyroDumpData(data);    

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

// Dead zone a value
static int applyDeadZone(int val) {

    if(val < dead_zone && val > -dead_zone) { val = 0; }
    return val;
    
}
