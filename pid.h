/*
 * Copyright (c) 2012, Regents of the University of California
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
 * Generalized integer PID module
 *
 * by Andrew Pullin
 *
 * v.0.1
 */

#ifndef __PID_H
#define __PID_H

//Select DSP core PID
//#define PID_HARDWARE

//DSP dependent include
#ifdef PID_HARDWARE
#include <dsp.h>
#endif

#define PID_ON  1
#define PID_OFF 0

//Structures and enums
//PID Continer structure

typedef struct {
    int input;
    long dState, iState, preSat, p, i, d;
    int Kp, Ki, Kd, Kaw, y_old, output;
    unsigned char N;
    char onoff; //boolean
    long error;
    unsigned long run_time;
    unsigned long start_time;
    int inputOffset;
    int Kff;
    int maxVal, minVal;
    int satValPos, satValNeg;
#ifdef PID_HARDWARE
    tPID dspPID;
#endif
} pidObj;

//Functions
void pidUpdate(pidObj *pid, int y);
void pidInitPIDObj(pidObj *pid, int Kp, int Ki, int Kd, int Kaw, int ff);
void pidSetInput(pidObj *pid, int feedback);
void pidSetGains(pidObj *pid, int Kp, int Ki, int Kd, int Kaw, int ff);
void pidOnOff(pidObj *pid, unsigned char state);

#endif // __PID_H
