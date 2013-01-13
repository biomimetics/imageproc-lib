/*
* Copyright (c) 2012, Regents of the University of California
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of the University of California, Berkeley nor the names
* of its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
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
* InvenSense MPU-6000 6-axis MEMS Driver
*
* by Richard J. Sheperd
* based on MPU-6050 Driver by Humphrey Hu
*
* v alpha
*
* Revisions:
* Duncan Haldane 11/1/2012 Made it work
* R. Fearing 12/31/2012 - fixed some lingering cs2 issues
*
* Notes:
* - This module uses an SPI port for communicating with the chip
*/

#include "mpu6000.h"
#include "ports.h"
#include "spi.h"
#include "spi_controller.h"
#include "utils.h"

#include <string.h>

#define SPI_CON1bits (SPI2CON1bits)
#define SPI_CON2 (SPI2CON2)
#define SPI_STATbits (SPI2STATbits)



// Read/Write Access
#define READ (128)
#define WRITE (0)

// Default configuration values
#define DEF_FCY (8000) // Gyro clock output
#define XL_MAX_FCY (1000) // Accelerometer output rate
#define DEFAULT_RATEDIV (7) // For sample rate of 1000

// More parameters
#define GYRO_SCALE_BASE (0.000133231241) // Lowest gyro range (250 degrees/sec)
#define XL_SCALE_BASE (5.98550415E-4) // Lowest accelerometer range (+-2 g)
#define TEMP_SCALE (0.00294117647)

#define DEFAULT_TEMP_OFFSET (521)
#define UPDATE_SIZE (14)
#define UPDATE_TIMEOUT (1000)

/*-----------------------------------------------------------------------------
* Static Variables
-----------------------------------------------------------------------------*/
// Internal data buffer defined now in mpu6000.h
/* static struct {
    int xl_data[3];
    int gyro_data[3];
    int temp;
} mpu_data; */

mpuObj mpu_data; // define data structure

static struct {
    int xl_offset[3]; // Not yet implemented!
    int gyro_offset[3]; // Not yet implemented!
    int temp_offset;	// Not yet implemented!
    float xl_scale;	// For m/s^2
    float gyro_scale;	// For rad/s
    float temp_scale;	// For celsius
} mpu_params;

/*-----------------------------------------------------------------------------
* Declaration of static functions
-----------------------------------------------------------------------------*/
static void writeReg(unsigned char regaddr, unsigned char data );
static unsigned char readReg(unsigned char regaddr);
static inline void setupSPI(void);

//TODO: Implement
// static inline unsigned int readString(unsigned int length, unsigned char * data,
// unsigned int data_wait);
//TODO: Implement
static inline unsigned int writeString(unsigned int length, unsigned char* data);	
//TODO: Implement
static inline void mpuSendByte( unsigned char byte );
//TODO: Implement
static inline unsigned char mpuReceiveByte(void);


/*-----------------------------------------------------------------------------
* Public functions
-----------------------------------------------------------------------------*/

// Note to self: FIFO State change requires power cycle!

void mpuSetup(void) {
    
   // setup SPI port
   setupSPI();
   unsigned char reg;
    
   writeReg(MPU_REG_PMGT1, 0x83); //Reset IMU
   delay_ms(100);
   writeReg(MPU_REG_PMGT1, 0x03); // Set clock to PLL Z Gyro
   writeReg(MPU_REG_USERCON, 0b00010000); // Disable I2C, FIFO Operations
   
   reg = readReg(MPU_REG_WHOAMI);	// Some sort of check here? =============== HH
    
    writeReg(MPU_REG_RATEDIV, DEFAULT_RATEDIV);	// Set rate divider
writeReg(MPU_REG_CONFIG, 0b00000001);	// DLPF configured to 188Hz Bandwidth [Fastest filtered mode]
    
    writeReg(MPU_REG_GYROCONFIG, 0b00011000);	// Set gyro scale 2000 DPS (8x)
    mpu_params.gyro_scale = GYRO_SCALE_BASE*8;
    
    writeReg(MPU_REG_XLCONFIG, 0b00010000);	// Set xl scale 8g (4x)
    mpu_params.xl_scale = XL_SCALE_BASE*4;	
    
    mpu_params.temp_scale = TEMP_SCALE;
    mpu_params.temp_offset = DEFAULT_TEMP_OFFSET;
    
    writeReg(MPU_REG_INTENABLE, 0);	// Disable interrupts
    //writeReg(MPU_REG_CONFIG, 0); // Set frame sync and DLPF off
    writeReg(MPU_REG_PMGT2, 0b00000000);	// Activate all sensors

 // mpuRunCalib(1000);
}

// TODO: Implement!
void mpuRunCalib(unsigned int count){

    unsigned int i;
    long gx, gy, gz;

    for(i = 0; i < count; i++) {
 // mpuUpdate();
        gx += mpu_data.gyro_data[0];
        gy += mpu_data.gyro_data[1];
        gz += mpu_data.gyro_data[2];
    }
    mpu_params.gyro_offset[0] = -gx/count;
    mpu_params.gyro_offset[1] = -gy/count;
    mpu_params.gyro_offset[2] = -gz/count;

}

// TODO: Implement
void mpuSetSleep(unsigned char mode) {

    return;

}

void mpuGetGyro(int* buff) {
    buff[0] = mpu_data.gyro_data[0] + mpu_params.gyro_offset[0];
    buff[1] = mpu_data.gyro_data[1] + mpu_params.gyro_offset[1];
    buff[2] = mpu_data.gyro_data[2] + mpu_params.gyro_offset[2];
}

float mpuGetGyroScale(void) {
    return mpu_params.gyro_scale;
}

void mpuGetXl(int* buff) {
    buff[0] = mpu_data.xl_data[0] + mpu_params.xl_offset[0];
    buff[1] = mpu_data.xl_data[1] + mpu_params.xl_offset[1];
    buff[2] = mpu_data.xl_data[2] + mpu_params.xl_offset[2];
}

float mpuGetXlScale(void) {
    return mpu_params.xl_scale;
}

void mpuGetTemp(int* buff) {
    *buff = mpu_data.temp + mpu_params.temp_offset;
}

float mpuGetTempScale(void) {
    return mpu_params.temp_scale;
}

/* This is DMA version - probably conflicts with DMA for dfmem, so use direct IO instead 
void mpuUpdate(void) {
    unsigned char buff[UPDATE_SIZE], rev[UPDATE_SIZE], i;	
    unsigned int read_bytes;

    spic2cs2BeginTransaction();
    spic2Transmit(MPU_REG_XLBASE);	//send address of first registry of IMU read back
    read_bytes = spic2MassTransmit(UPDATE_SIZE, NULL, 9*UPDATE_SIZE);
    spic2cs2EndTransaction();

    spic2ReadBuffer(read_bytes, buff);

// Order is XL[6] TEMP[2] GYRO[6]
    // Reverse data
    for(i = 0; i < UPDATE_SIZE; i++) {
        rev[i] = buff[UPDATE_SIZE - i - 1];
    }
    
    // Order is now GYRO[6] TEMP[2] XL[6]
    // Copy into buffers
    memcpy(mpu_data.gyro_data, rev, 6);
    memcpy(&mpu_data.temp, rev + 6, 2);
    memcpy(mpu_data.xl_data, rev + 8, 6);

}
*/

/**************************************************************/
/* simplified version to avoid needing dual DMA on SPI2 (memory + gyro) */
// need to use burst mode to ensure all readings are synchronized
void mpuUpdate(void) 
{  unsigned char c, buff[UPDATE_SIZE], rev[UPDATE_SIZE], i = 0;
   
	c= readReg(MPU_REG_XLBASE);
 	spic2cs2BeginTransaction(); 

 	spic2Transmit(MPU_REG_XLBASE | READ);
 	for ( i = 0; i < 14; i++)
	{  buff[i] = spic2Receive(); }
	spic2cs2EndTransaction();

	// Order is XL[6] TEMP[2] GYRO[6]
	// Reverse data
	for(i = 0; i < UPDATE_SIZE; i++) {
		rev[i] = buff[UPDATE_SIZE - i - 1];
	}
// Order is now GYRO[6] TEMP[2] XL[6]
// Copy into buffers

	memcpy(mpu_data.gyro_data, rev, 6);
	memcpy(&mpu_data.temp, rev + 6, 2);
	memcpy(mpu_data.xl_data, rev + 8, 6);

}




/*-----------------------------------------------------------------------------
* ----------------------------------------------------------------------------
* The functions below are intended for internal use, i.e., private methods.
* Users are recommended to use functions defined above.
* ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*****************************************************************************
* Function Name : writeReg
* Description : Write a data to a register
* Parameters : regaddr - address of register
* data - value to be written to the register
* Return Value : None
*****************************************************************************/
static void writeReg(unsigned char regaddr, unsigned char data ){
    spic2cs2BeginTransaction(); // Its usually useful to return the value read too ==========
    spic2Transmit(regaddr);
    spic2Transmit(data);
    spic2cs2EndTransaction();
}

/*****************************************************************************
* Function Name : readReg
* Description : Read a register
* Parameters : regaddr - address of register
* Return Value : register contents
*****************************************************************************/
static unsigned char readReg(unsigned char regaddr) {
    unsigned char c;
    
    spic2cs2BeginTransaction();
    spic2Transmit(regaddr | READ);
    c = spic2Receive();
    spic2cs2EndTransaction();
    
    return c;
}



/*****************************************************************************
* Function Name : mpuWriteString
* Description : Writes a buffer of data to the SPI bus
* Parameters : length is the string length to write
* data is a pointer to buffer containing data to write
* Return Value : Error codes: -3 if collision, 0 if successful
*****************************************************************************/
static inline unsigned int writeString(unsigned int length, unsigned char * data) {
return 0;//putsSPI2(length, data); Modified by doug in desperate attempt to get code to work.
}


/*****************************************************************************
* Function Name : sendByte
* Description : Send a byte to mpuscope
* Parameters : byte - a byte to send
* Return Value : None
*****************************************************************************/
static inline void mpuSendByte( unsigned char byte ) {

spic2Transmit(byte);

}

/*****************************************************************************
* Function Name : receiveByte
* Description : Receive a byte from mpuscope
* Parameters : None
* Return Value : None
*****************************************************************************/
static inline unsigned char mpuReceiveByte(void) {
    return NULL;
}

/*****************************************************************************
* Function Name : setupSPI
* Description : Setup SPI for mpuscope
* Parameters : None
* Return Value : None
*****************************************************************************/
static inline void setupSPI(void) {

// SPI2CON1 Register Settings
    SPI_CON1bits.MSTEN = 1; // Master mode Enabled
    SPI_CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
    SPI_CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI_CON1bits.MODE16 = 0; // Communication is byte-wide (8 bits)
    SPI_CON1bits.SMP = 0; // Input data is sampled at middle of data output time
    SPI_CON1bits.SSEN = 0; // SSx pin is used
    SPI_CON1bits.CKE = 1; // Serial output data changes on transition
                        // from active clock trx_state to idle clock trx_state
    SPI_CON1bits.CKP = 0; // Idle trx_state for clock is a low level;
                            // active trx_state is a high level

    // Set up SCK frequency of 13.333Mhz for 40 MIPS
	// 1 MHz limit for MPU 6000
    //SPI_CON1bits.SPRE = 0b100; // Secondary prescale    3:1
    //SPI_CON1bits.PPRE = 0b11; // Primary prescale       1:1
    SPI_CON1bits.SPRE = 0b000; // Secondary prescale    8:1
    SPI_CON1bits.PPRE = 0b10; // Primary prescale       4:1
	
	// SPI2CON2 Register Settings
	SPI_CON2 = 0x0000;
	
	// SPI2STAT Register Settings
    SPI_STATbits.SPISIDL = 1; // Discontinue module when device enters idle mode
    SPI_STATbits.SPIROV = 0; // Clear Overflow
    SPI_STATbits.SPIEN = 1; // Enable SPI module
}
