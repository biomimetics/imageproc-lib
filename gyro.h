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
 *  - Uses an I2C port for communicating with the gyroscope chip
 *
 * Usage:
 *  #include  "gyro.h"
 *
 *  float gyroData[3];
 *  unsigned char * gyroStrData;
 *
 *  // initialize gyro module
 *  gyroSetup();
 *
 *  // run calibration with 1000 gyro data
 *  gyroRunCalib(1000)
 *
 *  // read out data from gyroscope and save in an internal buffer
 *  gyroReadXYZ();
 *
 *  // convert data into floating point values in deg/s
 *  gyroGetDegXYZ(gyroData);
 *  // gyroGetRadXYZ(gyroData);  // in radian/s
 *
 *  // read the data in raw string format
 *  // gyroStrData[0] = lower byte of x-axis data
 *  // gyroStrData[1] = higher byte of x-axis data
 *  // gyroStrData[2] = lower byte of y-axis data
 *  // ...
 *  // gyroStrData[5] = higher byte of z-axis data
 *
 *  gyroStrData = gyroGetsXYZ();
 */

#ifndef __GYRO_H
#define __GYRO_H

/*****************************************************************************
* Function Name : gyroSetup
* Description   : Initialize gyroscope
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void gyroSetup(void);

/*****************************************************************************
* Function Name : gyroSetSampleRate
* Description   : Change sampling rate and LPF bandwidth.  Please refer to
*                 ITG-3200 datasheet page 24 for valid values.
* Parameters    : rate.
* Return Value  : None
*****************************************************************************/
void gyroSetSampleRate(unsigned char rate);

/******************************************************************************
* Function Name : gyroSetIntEn
* Description   : Enable or disable External Interrupt 1.
* Parameters    : 1 (TRUE) to enable and 0 (FALSE) to disable
* Return Value  : None
******************************************************************************/
void gyroSetIntEn(unsigned char flag);

/*****************************************************************************
* Function Name : gyroSleep
* Description   : Put gyroscope into sleep mode
*                 It is recommended to use this function to save power if a
*                 user is not using gyroscope\.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void gyroSleep(void);

/*****************************************************************************
* Function Name : gyroWake
* Description   : Put gyroscope into normal operating mode
* Parameters    : None
* Return Value  : None
******************************************************************************/
void gyroWake(void);

/*****************************************************************************
* Function Name : gyroGetCalibParam
* Description   : Get the calibration parameters for gyroscope.
* Parameters    : None
* Return Value  : The array of 12 characters representing 3 floating point
*                 calibration offset values.
******************************************************************************/
unsigned char* gyroGetCalibParam(void);

/*****************************************************************************
* Function Name : gyroGetOffsets
* Description   : Get the calibration offsets for gyroscope.
* Parameters    : An array of 3 integers where to save the offsets.
* Return Value  : None
*******************************************************************************/
void gyroGetOffsets(int* data);

/*****************************************************************************
* Function Name : gyroRunCalib
* Description   : Calibrate gyroscope
* Parameters    : An unsigned integer value for the number of data to read
*                 from gyroscope for calibration. It takes count*1ms to finish
*                 calibration.
* Return Value  : None
*****************************************************************************/
void gyroRunCalib(unsigned int count);

/*****************************************************************************
* Function Name : gyroGetFloatTemp
* Description   : Get temperature in float point format. The unit is degrees
*                 Celsius.
* Parameters    : None
* Return Value  : temperature in floating point format
*****************************************************************************/
float gyroGetFloatTemp(void);

/*****************************************************************************
* Function Name : gyroGetIntTemp
* Description   : Get temperature in raw integer format.
* Parameters    : None
* Return Value  : temperature in integer format
*****************************************************************************/
int gyroGetIntTemp(void);

/*****************************************************************************
* Function Name : gyroReadTemp
* Description   : Read temperature from gyroscopeand save into internal buffer
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void gyroReadTemp(void);

/*****************************************************************************
* Function Name : gyroGetRadXYZ
* Description   : Get X, Y, and Z gyro data in floating point format.
*                 The unit is radian/s.
* Parameters    : Array of 3 floating point numbers to store gyro data.
* Return Value  : None
*****************************************************************************/
void gyroGetRadXYZ(float* data);

/*****************************************************************************
* Function Name : gyroGetRadX
* Description   : Get the x-axis gyro data in floating point format.
*                 The unit is radian/s.
* Parameters    : None
* Return Value  : The x-axis gyro data.
*****************************************************************************/
float gyroGetRadX(void);

/*****************************************************************************
* Function Name : gyroGetRadY
* Description   : Get the y-axis gyro data in floating point format.
*                 The unit is radian/s.
* Parameters    : None
* Return Value  : The y-axis gyro data.
*****************************************************************************/
float gyroGetRadY(void);

/*****************************************************************************
* Function Name : gyroGetRadZ
* Description   : Get the z-axis gyro data in floating point format.
*                 The unit is radian/s.
* Parameters    : None
* Return Value  : The z-axis gyro data.
*****************************************************************************/
float gyroGetRadZ(void);

/*****************************************************************************
* Function Name : gyroGetDegXYZ
* Description   : Get X, Y, and Z gyro data in floating point format.
*                 The unit is deg/s.
* Parameters    : Array of 3 floating point numbers to store gyro data.
* Return Value  : None
* To Do         : Need to implement better sensor calibration and adjust the
*                 raw data....
*****************************************************************************/
void gyroGetDegXYZ(float* data);

/*****************************************************************************
* Function Name : gyroToString
* Description   : Get X, Y, and Z gyro data in raw string format.
* Parameters    : None
* Return Value  : pointer of the internal buffer holding 6 characters
*                 representing the 3-axis gyro data.
*****************************************************************************/
unsigned char* gyroToString(void);

/*****************************************************************************
* Function Name : gyroDumpData
* Description   : Get X, Y, and Z gyro data in raw string format.
* Parameters    : buffer is an char array holding 6 characters
*                 representing the 3-axis gyro data.
* Return Value  : None
*****************************************************************************/
void gyroDumpData(unsigned char* buffer);

/*****************************************************************************
* Function Name : gyroReadXYZ
* Description   : Read X, Y, and Z gyro outputs and save into internal
*                 buffer and return X, Y, and Z gyro data in raw string format.
*                 It will take about 250us for this function.
* Parameters    : None
* Return Value  : pointer of the internal buffer holding 6 characters
*                 representing the 3-axis gyro data.
*****************************************************************************/
unsigned char* gyroReadXYZ(void);

/*****************************************************************************
* Function Name : gyroGetXYZ
* Description   : Read X, Y, and Z gyro outputs and save into the array passed
*                 into this function. It will take about 250us for this
*                 function.
* Parameters    : An array of 6 characters to store 3-axis gyro data.
* Return Value  : None
*****************************************************************************/
void gyroGetXYZ(unsigned char *data);

#endif // __GYRO_H
