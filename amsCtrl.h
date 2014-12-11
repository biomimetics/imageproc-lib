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
 * ams PID Module
 *
 * by Duncan Haldane
 *
 * v.0.2
 */

#ifndef __AMS_CTRL_H
#define __AMS_CTRL_H


#ifdef PID_SOFTWARE

    #define AMS_DEFAULT_KP  200
    #define AMS_DEFAULT_KI  5
    #define AMS_DEFAULT_KD  0
    #define AMS_DEFAULT_KAW 5
    #define AMS_DEFAULT_KFF  0
    #define SOFT_GAIN_SCALER 512

#elif defined PID_HARDWARE

    #define AMS_DEFAULT_KP  3000
    #define AMS_DEFAULT_KI  10
    #define AMS_DEFAULT_KD  0
    #define AMS_DEFAULT_KAW 0
    #define AMS_DEFAULT_KFF  0
    #define AMS_GAIN_SCALE  8

#endif

#define nPIDS  2

//Setup PID for ams encoders
void amsPIDSetup(void);

void amsCtrlSetInput(unsigned char num, int state);

void amsCtrlPIDUpdate(unsigned char num, int state);

void amsCtrlSetGains(unsigned char num, int Kp, int Ki, int Kd, int Kaw, int ff);


#endif // __AMS_CTRL_H
