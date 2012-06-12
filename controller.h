/*
 * Copyright (c) 2009 - 2010, Regents of the University of California
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
 * Control block module
 *
 * by Stanley S. Baek
 *
 * v.beta
 */

#ifndef __CONTROLLER_H
#define __CONTROLLER_H


#include "dfilter.h"

typedef struct {
    char running;
    float ref;
    float offset;
    float ts;   // sampling interval
    float kp;   // proportional control gain 
    float ki;   // integral control gain in discrete time (= cont. time gain * ts)
    float kd;   // derivative control gain in discrete time (= cont. time gain / ts)
    float beta; // reference weight for proportional control
    float gamma; // reference weight for derivative control
    float umax;
    float umin;
    float iold;
    float derrold;
} CtrlPidParamStruct;

typedef CtrlPidParamStruct* CtrlPidParam;

float ctrlGetRef(CtrlPidParam pid);
void ctrlSetRef(CtrlPidParam pid, float ref);
float ctrlRunPid(CtrlPidParam pid, float y, DigitalFilter lpf); 
CtrlPidParam ctrlCreatePidParams(float ts);
void ctrlSetPidParams(CtrlPidParam pid, float ref, float kp, float ki, float kd);
void ctrlSetPidOffset(CtrlPidParam pid, float offset);
float ctrlGetPidOffset(CtrlPidParam pid);
void ctrlSetRefWeigts(CtrlPidParam pid, float beta, float gamma);
void ctrlSetSaturation(CtrlPidParam pid, float max, float min);
unsigned char ctrlIsRunning(CtrlPidParam pid);
void ctrlStart(CtrlPidParam pid);
void ctrlStop(CtrlPidParam pid);


#endif  // __CONTROLLER_H
