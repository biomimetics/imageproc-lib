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
 * InvenSense MPU-6050 6-axis MEMS Driver
 *
 * by Humphrey Hu
 * based on ITG-3200 Driver by Stanley S. Baek
 *
 * Notes:
 *  - Uses an I2C port for communicating with the mpuscope chip
 *
 * Usage:
 *  #include  "mpu.h"
 *
 *  float mpuData[3];
 *  unsigned char * mpuStrData;
 *
 *  // initialize mpu module
 *  mpuSetup();
 * 
 *  // run calibration with 1000 samples
 *  mpuRunCalib(1000)
 *
 *  // read out data from mpuscope and save in an internal buffer
 *  mpuReadXYZ();
 *
 *  // convert data into floating point values in deg/s
 *  mpuGetDegXYZ(mpuData);
 *  // mpuGetRadXYZ(mpuData);  // in radian/s  
 *
 *  // read the data in raw string format
 *  // mpuStrData[0] = lower byte of x-axis data
 *  // mpuStrData[1] = higher byte of x-axis data
 *  // mpuStrData[2] = lower byte of y-axis data
 *  // ...
 *  // mpuStrData[5] = higher byte of z-axis data
 *
 *  mpuStrData = mpuGetsXYZ();
 */

#ifndef __MPU_H
#define __MPU_H

// Setup device
void mpuSetup(void);

// Run calibration routine
void mpuRunCalib(unsigned int count);

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
void mpuUpdate(void);

#endif



