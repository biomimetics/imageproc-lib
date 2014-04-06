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
 * InvenSense MPU-6000 6-axis MEMS Driver
 *
 * by Richard J. Sheperd
 *
 * v.alpha
 *
 * Revisions:
 *  Richard J. Sheperd              Initial release
 *  Duncan Haldane      2012-11-01  Made it work
 *  Ronald S. Fearing   2012-12-31  Fixed some lingering cs2 issues
 *
 * Notes:
 * - Based on MPU-6050 Driver by Humphrey Hu
 * - This module uses an SPI port for communicating with the chip
 */

#include "mpu6000.h"
#include "ports.h"
#include "spi.h"
#include "spi_controller.h"
#include "utils.h"
#include <string.h>

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
    int temp_offset;    // Not yet implemented!
    float xl_scale; // For m/s^2
    float gyro_scale;   // For rad/s
    float temp_scale;   // For celsius
} mpu_params;


/*-----------------------------------------------------------------------------
 * Declaration of static functions
 -----------------------------------------------------------------------------*/
static void writeReg(unsigned char regaddr, unsigned char data );
static unsigned char readReg(unsigned char regaddr);
static inline void setupSPI(char initConfig);
static void mpuFinishUpdate(unsigned int cause);
static void waitDmaFinish(void);


/*-----------------------------------------------------------------------------
 * Public functions
 -----------------------------------------------------------------------------*/

// Note to self: FIFO State change requires power cycle!

void mpuSetup(void) {

  // setup SPI port
  setupSPI(MPU_CS);  // Setup SPI for register configuration
  unsigned char reg;

  writeReg(MPU_REG_PMGT1, 0x83);              //Reset IMU
  delay_ms(100);
  writeReg(MPU_REG_PMGT1, 0x03);              // Set clock to PLL Z Gyro
  writeReg(MPU_REG_USERCON, 0b00010000);      // Disable I2C, FIFO Operations

  reg = readReg(MPU_REG_WHOAMI);              // Some sort of check here?

  writeReg(MPU_REG_RATEDIV, DEFAULT_RATEDIV); // Set rate divider
  // writeReg(MPU_REG_CONFIG, 0b00000001);     // DLPF configured to 188Hz BW
                                              // [Fastest filtered mode]

  writeReg(MPU_REG_GYROCONFIG, 0b00011000);   // Set gyro scale 2000 DPS (8x)
  mpu_params.gyro_scale = GYRO_SCALE_BASE*8;

  writeReg(MPU_REG_XLCONFIG, 0b00010000);     // Set xl scale 8g (4x)
  mpu_params.xl_scale = XL_SCALE_BASE*4;

  mpu_params.temp_scale = TEMP_SCALE;
  mpu_params.temp_offset = DEFAULT_TEMP_OFFSET;

  writeReg(MPU_REG_INTENABLE, 0);             // Disable interrupts
  writeReg(MPU_REG_CONFIG, 0);              // Set frame sync and DLPF off
  writeReg(MPU_REG_PMGT2, 0b00000000);        // Activate all sensors

  setupSPI(0);  // Setup SPI for highspeed data readback. Can do 20MHz, set for 13

  mpuRunCalib(1000, 1000);
}

void mpuRunCalib(unsigned int discard, unsigned int count) {

    unsigned int i;
    long gx = 0, gy = 0, gz = 0;

    // Discard initial samples
    for (i = 0; i < discard; i++) {
      mpuBeginUpdate();
      waitDmaFinish();
    }

    // Calibrate gyro offset
    for(i = 0; i < count; i++) {
        mpuBeginUpdate();
        waitDmaFinish();
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
    waitDmaFinish();
    buff[0] = mpu_data.gyro_data[0] + mpu_params.gyro_offset[0];
    buff[1] = mpu_data.gyro_data[1] + mpu_params.gyro_offset[1];
    buff[2] = mpu_data.gyro_data[2] + mpu_params.gyro_offset[2];
}

float mpuGetGyroScale(void) {
    return mpu_params.gyro_scale;
}

void mpuGetXl(int* buff) {
    waitDmaFinish();
    buff[0] = mpu_data.xl_data[0] + mpu_params.xl_offset[0];
    buff[1] = mpu_data.xl_data[1] + mpu_params.xl_offset[1];
    buff[2] = mpu_data.xl_data[2] + mpu_params.xl_offset[2];
}

float mpuGetXlScale(void) {
    return mpu_params.xl_scale;
}

void mpuGetTemp(int* buff) {
    waitDmaFinish();
    *buff = mpu_data.temp + mpu_params.temp_offset;
}

float mpuGetTempScale(void) {
    return mpu_params.temp_scale;
}

///**************************************************************/
///* simplified version to avoid needing dual DMA on SPI2 (memory + gyro) */
//// need to use burst mode to ensure all readings are synchronized
//void mpuUpdate(void)
//{  unsigned char c, buff[UPDATE_SIZE], rev[UPDATE_SIZE], i = 0;
//
//    c= readReg(MPU_REG_XLBASE);
//    spic2cs2BeginTransaction();
//
//    spic2Transmit(MPU_REG_XLBASE | READ);
//    for ( i = 0; i < 14; i++)
//    {  buff[i] = spic2Receive(); }
//    spic2cs2EndTransaction();
//
//    // Order is XL[6] TEMP[2] GYRO[6]
//    // Reverse data
//    for(i = 0; i < UPDATE_SIZE; i++) {
//        rev[i] = buff[UPDATE_SIZE - i - 1];
//    }
//// Order is now GYRO[6] TEMP[2] XL[6]
//// Copy into buffers
//
//    memcpy(mpu_data.gyro_data, rev, 6);
//    memcpy(&mpu_data.temp, rev + 6, 2);
//    memcpy(mpu_data.xl_data, rev + 8, 6);
//
//}

void waitDmaFinish(void) {
  spic2BeginTransaction(MPU_CS);
  spic2EndTransaction();
}

void mpuBeginUpdate(void) {
  spic2BeginTransaction(MPU_CS);
  spic2Transmit(MPU_REG_XLBASE | READ);
  //TODO(rqou): better timeout?
  spic2MassTransmit(UPDATE_SIZE, NULL, 1000);
}

static void mpuFinishUpdate(unsigned int cause) {
  //TODO(rqou): don't ignore cause
  unsigned char buff[UPDATE_SIZE], i, temp;

  spic2ReadBuffer(UPDATE_SIZE, buff);
  spic2EndTransaction();

  // Order is XL[6] TEMP[2] GYRO[6]
  //reverse endianness
  for(i = 0; i < UPDATE_SIZE; i += 2) {
      temp = buff[i];
      buff[i] = buff[i+1];
      buff[i+1] = temp;
  }

  // Copy into buffers
  memcpy(mpu_data.gyro_data, buff + 8, 6);
  memcpy(&mpu_data.temp, buff + 6, 2);
  memcpy(mpu_data.xl_data, buff, 6);

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
static void writeReg(unsigned char regaddr, unsigned char data )
{
    spic2BeginTransaction(MPU_CS);
    spic2Transmit(regaddr);
    spic2Transmit(data);
    spic2EndTransaction();
}

/*****************************************************************************
* Function Name : readReg
* Description : Read a register
* Parameters : regaddr - address of register
* Return Value : register contents
*****************************************************************************/
static unsigned char readReg(unsigned char regaddr) {
  unsigned char c;

  spic2BeginTransaction(MPU_CS);
  spic2Transmit(regaddr | READ);
  c = spic2Receive();
  spic2EndTransaction();

  return c;
}

/*****************************************************************************
* Function Name : setupSPI
* Description : Setup SPI for mpuscope
* Parameters : None
* Return Value : None
*****************************************************************************/
static inline void setupSPI(char initConfig)
{
    if (initConfig)
    {
      spicSetupChannel2(MPU_CS,
                        ENABLE_SCK_PIN &
                        ENABLE_SDO_PIN &
                        SPI_MODE16_OFF &
                        SPI_SMP_OFF &
                        SPI_CKE_ON &
                        SLAVE_ENABLE_OFF &
                        CLK_POL_ACTIVE_HIGH &
                        MASTER_ENABLE_ON &
                        PRI_PRESCAL_64_1 &
                        SEC_PRESCAL_1_1);
    } else {    
      spicSetupChannel2(MPU_CS,
                      ENABLE_SCK_PIN &
                      ENABLE_SDO_PIN &
                      SPI_MODE16_OFF &
                      SPI_SMP_OFF &
                      SPI_CKE_ON &
                      SLAVE_ENABLE_OFF &
                      CLK_POL_ACTIVE_HIGH &
                      MASTER_ENABLE_ON &
                      PRI_PRESCAL_1_1 &
                      SEC_PRESCAL_4_1);
    }

    spic2SetCallback(MPU_CS, &mpuFinishUpdate);
}
