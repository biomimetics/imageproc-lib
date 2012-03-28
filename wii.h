/*
 * Copyright (c) 2009-2010, Regents of the University of California
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
 * Communication with Wiimote IR Detector (I2C)
 *
 * by Stanley S. Baek
 *
 * v.beta
 */

#ifndef __WII_H
#define __WII_H


#define WII_PIXEL2RAD       3.14159265/180*40/1024   // 1024 pixels for 40 degrees

#define WII_INVALID_BLOB    0xFF

// Flashing pattern of IR emitters
// Although 4 bits are used now, it could be expanded to 8 bits.
typedef enum {WII_PATTERN_0000, WII_PATTERN_0001, WII_PATTERN_0011, WII_PATTERN_0101, 
        WII_PATTERN_0111, WII_PATTERN_1111} WiiFlashPaterns;

typedef struct {
    int x;
    int y;
    unsigned char size;
    WiiFlashPaterns pattern;
} WiiBlob; 


/******************************************************************************
* Function Name : wiiSetupBasic
* Description   : Initialize wii camera with basic features. 
* Parameters    : None
* Return Value  : None
******************************************************************************/
void wiiSetupBasic(void);


/*****************************************************************************
* Function Name : wiiSetupAdvance
* Description   : Initialize wii camera with advanced features.
* Parameters    : Sensitivity: 1 = highest sensitivity, 5 = lowest sensitivity.
*                 If 0 is passed for sensitivity, wiiSetupBasic will be called.
*                 Refer to the wiimote documentation for details.
* Return Value  : None
*****************************************************************************/
void wiiSetupAdvance(unsigned char sensitivity, unsigned char mode);


/*****************************************************************************
* Function Name : wiiGetData
* Description   : Get wii blob data in integer format.
*                 This function is a combination of wiiReadData and 
*                 wiiConvertData.
* Parameters    : WiiBlob array of 4 to contain blob data.
* Return Value  : None
*****************************************************************************/
void wiiGetData(WiiBlob* blobs);

/*****************************************************************************
* Function Name : wiiConvertData
* Description   : Get wii blob data in integer format.
*                 Users may need to call wiiReadData before this function.
* Parameters    : WiiBlob array of 4 to contain blob data.
* Return Value  : None
*****************************************************************************/
void wiiConvertData(WiiBlob* blobs);


/*****************************************************************************
* Function Name : wiiToString
* Description   : Get wii data in raw string format.
* Parameters    : None
* Return Value  : pointer for internal buffer 
*****************************************************************************/
unsigned char* wiiToString(void);


/*****************************************************************************
* Function Name : wiiDumpData
* Description   : Get wii data in raw string format.
* Parameters    : buffer is a char array holding 12 characters 
*                 representing 4 blobs of wii data.
* Return Value  : None
*****************************************************************************/
void wiiDumpData(unsigned char* buffer);


/*****************************************************************************
* Function Name : wiiReadData
* Description   : Read blob locations and blob sizes. It will take about 400us
*                 to finish this function.
* Parameters    : None
* Return Value  : pointer for internal buffer 
*****************************************************************************/
unsigned char* wiiReadData(void);

/*****************************************************************************
* Function Name : wiiFindPattern
* Description   : Find the blinking pattern of IR LEDs
* Parameters    : An array of 4 WiiBlob structures to contain blob data.
* Return Value  : The index of the target. The valid index is between 0 and 3.
*                 The return value of -1 indicates that target has not been
*                 found.
*****************************************************************************/
char wiiFindTarget(WiiBlob* blobs);

#endif // __WII_H

