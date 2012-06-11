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
 *  - The default setup is +-8g and 800Hz output rate.
 *  - Uses an I2C port for communicating with the accelerometer chip.
 *  - MCU resources requied for this module:
 *      I2C bus (I2C1 for ImageProc2) - SCL1 & SDA1
 *      External INT (INT3)
 *
 * Usage:
 *  #include  "xl.h"
 *
 *  float xlData[3];
 *  unsigned char * xlStrData;
 *
 *  // initialize xl module with default range, +-8g, 800Hz output rate
 *  xlSetup()
 *
 *  // change the operation range to +-2g
 *  xlSetRange(2);
 *
 *  // read out data from accelerometer and save in an internal buffer
 *  xlReadXYZ();
 *
 *  // convert data into floating point values in m/s^2
 *  xlGetfXYZ(xlData);
 *
 *  // read the data in raw string format
 *  // xlStrData[0] = lower byte of x-axis data
 *  // xlStrData[1] = higher byte of x-axis data...
 *  xlStrData = xlGetsXYZ();
 *
 */

#ifndef __XL_H
#define __XL_H


/*****************************************************************************
* Function Name : xlSetup
* Description   : Initialize accelerometer
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void xlSetup(void);

/******************************************************************************
* Function Name : xlSetIntEn
* Description   : Enable or disable External Interrupt 3. It is disabled by
*                 default. At this moment, there is no ISR for this module.
*                 If we think we need any interrupt, we will implement
*                 xlHandleISR() function.
* Parameters    : 1 (TRUE) to enable and 0 (FALSE) to disable
* Return Value  : None
******************************************************************************/
void xlSetIntEn(unsigned char flag);

/*****************************************************************************
* Function Name : xSetRange
* Description   : ChangePut accelerometer into sleep mode
*                 It is recommended to use this function to save power if a
*                 user is not using acceleromter.
* Parameters    : Range can be 2, 4, 8, or 16 for +-2g, +-4g, +-8g, +-16g,
*                 respectively. If a range value is other than those listed,
*                 the current range will not be changed. The default range is
*                 +-8g. The sensor resolution will be changed accordingly to
*                 maintain the full resolution.
* Return Value  : None
*****************************************************************************/
void xlSetRange(unsigned char range);

/*****************************************************************************
* Function Name : xlSetOutputRate
* Description   : Set the output data rate. Please check the datasheet for
*                 ADXL345 page 13 for the valid rates.
* Parameters    : power_mode is 0 for normal power consumption mode and 1 for
*                 low power consumption mode.
*                 rate is the output data rate in the range of 0x00 - 0x0f.
* Return Value  : None
*****************************************************************************/
void xlSetOutputRate(unsigned char power_mode, unsigned char rate);

/*****************************************************************************
* Function Name : xlSleep
* Description   : Put accelerometer into sleep mode
*                 It is recommended to use this function to save power if a
*                 user is not using acceleromter.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void xlSleep(void);

/*****************************************************************************
* Function Name : xlWake
* Description   : Put accelerometer into normal operating mode
* Parameters    : None
* Return Value  : None
=*****************************************************************************/
void xlWake(void);

/*****************************************************************************
* Function Name : xlSetCalibParam
* Description   : Set the current calibration parameters with param
* Parameters    : calibration parameters in string format.
* Return Value  : None
*****************************************************************************/
void xlSetCalibParam(unsigned char* param);

/*****************************************************************************
* Function Name : xlGetCalibParam
* Description   : Return the current calibration parameters
* Parameters    : None
* Return Value  : parameter data in string
*****************************************************************************/
unsigned char* xlGetCalibParam(void);

/*****************************************************************************
* Function Name : xlLoadCalibParam
* Description   : Load the current calibration parameters from Flash Memory.
*                 If the first scale factor is zero, use XL_DEFAULT_SCALE
*                 value for the first three parameters and use "0" for the
*                 last three parameters.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void xlLoadCalibParam(void);

/*****************************************************************************
* Function Name : xlSaveCalibParam
* Description   : Save the current calibration parameters in Flash Memory
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void xlSaveCalibParam(void);

/*****************************************************************************
* Function Name : xlGetID
* Description   : Get accelerometer identification number
* Parameters    : None
* Return Value  : Accelerometer ID
*****************************************************************************/
unsigned char xlGetID(void);

/*****************************************************************************
* Function Name : xlGetFloatXYZ
* Description   : Get X, Y, and Z data in m/s^2 in floating point format.
*                 This is the only function that returns calibrated sensor data
*                 if the calibration parameters are set.
* Parameters    : Array of 3 floating point numbers to store accelerometer data.
* Return Value  : None
*****************************************************************************/
void xlGetFloatXYZ(float* data);

/*****************************************************************************
* Function Name : xlGetIntXYZ
* Description   : Get X, Y, and Z raw data in integer format. Users need to
*                 convert with a scale factor. For a fresh data, xlReadXYZ
*                 must be called before this function call.
* Parameters    : None
* Return Value  : pointer for internal buffer holding 3 integers representing
*                 the 3-axis accelerometer data.
*****************************************************************************/
int* xlGetIntXYZ(void);

/*****************************************************************************
* Function Name : xlToString
* Description   : Get X, Y, and Z raw data in string format. Users need to
*                 convert with a scale factor.  For a fresh data, xlReadXYZ
*                 must be called before this function call.
* Parameters    : None
* Return Value  : The pointer of the internal buffer holding 6 characters
*                 representing the 3-axis accelerometer data.
*****************************************************************************/
unsigned char* xlToString(void);


/*****************************************************************************
* Function Name : xlDumpData
* Description   : Get X, Y, and Z xl data in raw string format.
* Parameters    : buffer is an char array holding 6 characters
*                 representing the 3-axis xl data.
* Return Value  : None
*****************************************************************************/
void xlDumpData(unsigned char* buffer);


/*****************************************************************************
* Function Name : xlReadXYZ
* Description   : This function reads X, Y, and Z xl outputs and save into
*                 an internal buffer and returns X, Y, and Z raw data in string
*                 format. It will take about 250us for this function.
* Parameters    : None
* Return Value  : The pointer of the internal buffer holding 6 characters
*                 representing the 3-axis accelerometer data.
*****************************************************************************/
unsigned char*  xlReadXYZ(void);

/*****************************************************************************
* Function Name : xlGetXYZ
* Description   : Read X, Y, and Z gyro outputs and save into the array passed
*                 into this function. xlReadXYZ does not have to be called for
*                 this function call. It will take about 250us for this
*                 function.
* Parameters    : An array of 6 characters to store 3-axis xl data.
* Return Value  : None
*****************************************************************************/
void xlGetXYZ(unsigned char *data);


#endif // __XL_H
