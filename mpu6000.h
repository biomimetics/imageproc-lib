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
 */

#ifndef __MPU6000_H
#define __MPU6000_H


// Registers
#define MPU_REG_RATEDIV (25)
#define MPU_REG_CONFIG (26)
#define MPU_REG_GYROCONFIG (27)
#define MPU_REG_XLCONFIG (28)
#define MPU_REG_FIFOEN (35)
#define MPU_REG_I2CMASTCON (36)
#define MPU_REG_I2CMASTSTAT (54)
#define MPU_REG_INTENABLE (56)
#define MPU_REG_INTSTAT (57)
#define MPU_REG_XLBASE (59)
#define MPU_XLLEN (6)
#define MPU_REG_TEMPBASE (65)
#define MPU_TEMPLEN (2)
#define MPU_REG_GYROBASE (67)
#define MPU_GYROLEN (6)
#define MPU_REG_USERCON (106)
#define MPU_REG_PMGT1 (107)
#define MPU_REG_PMGT2 (108)
#define MPU_REG_FIFOCNTH (114) // Not sure if high or low
#define MPU_REG_FIFOCNTL (115)
#define MPU_REG_FIFORW (116)
#define MPU_REG_WHOAMI (117)


// Internal data buffer
 typedef struct {
    int xl_data[3];
    int gyro_data[3];
    int temp;   // temperature
} mpuObj;


// Setup device, chip select set in BSP header
void mpuSetup(void);

// Run calibration routine
void mpuRunCalib(unsigned int discard, unsigned int count);

// Set sleep mode
void mpuSetSleep(unsigned char mode);

// 3 ints
void mpuGetGyro(int* buff);
// 3 ints
void mpuGetXl(int* buff);
// 1 int
void mpuGetTemp(int* buff);

float mpuGetGyroScale(void);
float mpuGetXlScale(void);
float mpuGetTempScale(void);

// Read data from MPU
// This begins an asynchronous update.
void mpuBeginUpdate(void);


#endif // __MPU6000_H
