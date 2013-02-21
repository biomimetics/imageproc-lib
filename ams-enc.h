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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE`
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Austria Microsystems AS5048B magnetic encoder I2C Interface
 *
 * by Duncan Haldane
 *
 * v.1.0
 *
 * Usage:
 *  #include "enc.h"
 *
 *  // initialize encoder module
 *  encSetup();
 *
 *  //get position of right encoder write to ENCPOS struct
 *  void encGetRPos()
 */

#ifndef __AMS_ENC_H
#define __AMS_ENC_H


#define MAX_HALL 0x4000 // maximum Hall sensor value
#define NUM_ENC 2

//Leg position struct
typedef struct {
    unsigned int pos; // raw reading from sensor 14 bits
    long oticks;  // revolution counter
    unsigned int calibPos;  // 0 to 2pi, converted to 16 bits
    unsigned int offset; // initial reading on setup - relative zero position
} EncObj;

extern EncObj encPos[NUM_ENC];

/*****************************************************************************
 * Function Name : amsHallSetup
 * Description   : initialize I2C, and for initial calibration, set offset
 *                 to current position
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void amsHallSetup(void);

/*****************************************************************************
 * Function Name : amsGetPos/encSumPos
 * Description   : Count encoder[num] rollovers, write to struct encPos
 * Parameters    : None
 * Return Value  : None
 * .pos: 0 <= .pos <= 0x3fff,  0 to 2 pi range.
 * see hall_velocity_ip2.5.ppt
 *****************************************************************************/
void amsGetPos(unsigned char num);

/*****************************************************************************
 * Function Name : encSetup
 * Description   : Initialize encoder
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void encSetup(void);

/*****************************************************************************
 * Function Name : encGetPos
 * Description   : Read the angular position of the right encoder, write to
 *                 struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void encGetPos(unsigned char num);

int amsStartPosRead(void);

/*****************************************************************************
 * Function Name : encSumPos
 * Description   : Count encoder[num] rollovers, write to struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void encSumPos(unsigned char num);

/*****************************************************************************
 * Function Name : encGetFloatPos
 * Description   : Read the angular position of encoder[num] return as float
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
float encGetFloatPos(unsigned char num);


#endif // __AMS_ENC_H
