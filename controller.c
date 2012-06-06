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
 *
 * Revisions:
 *  Stanley S. Baek      2009-10-30    Initial release 
 */

#include "controller.h"
#include "dfilter.h"
#include <stdlib.h>


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

float ctrlGetRef(CtrlPidParam pid) {
    return pid->ref;
}

void ctrlSetRef(CtrlPidParam pid, float ref) {
    pid->ref = ref;
}



float ctrlRunPid(CtrlPidParam pid, float y, DigitalFilter lpf) {

    if (pid->running == 0) return 0;

    float p, d, u;
    float derr = pid->gamma*pid->ref - y;  // error for derivative control

    p = pid->kp*(pid->beta*pid->ref - y);    // proportional control gain

    if (lpf == NULL) {
        d = pid->kd*(derr - pid->derrold);
    } else {
        d = pid->kd*dfilterApply(lpf, derr - pid->derrold);
    }

    u = pid->offset + p + pid->iold + d;

    if (u > pid->umax) { // do not update pid->iold for ki = 0
        u = pid->umax;
    } else if (u < pid->umin) {  // do not update pid->iold for ki = 0
        u = pid->umin;
    } else {    // update pid->iold with ki
        pid->iold += pid->ki*(pid->ref - y);
    }

    pid->derrold = derr;
    
    return u;

}

CtrlPidParam ctrlCreatePidParams(float ts) {

    CtrlPidParam pid = (CtrlPidParam)malloc(sizeof(CtrlPidParamStruct));
    pid->ref = 0;
    pid->running = 0;
    pid->ts = ts;
    pid->iold = 0;
    pid->derrold = 0;  
    pid->offset = 0;
    
    return pid;
}

void ctrlSetPidParams(CtrlPidParam pid, float ref, float kp, float ki, float kd) {

    pid->ref = ref;
    pid->kp = kp;
    pid->kd = kd/pid->ts;
    pid->ki = ki*pid->ts;
}


void ctrlSetPidOffset(CtrlPidParam pid, float offset) {
    pid->offset = offset;
}

float ctrlGetPidOffset(CtrlPidParam pid) {
    return pid->offset;
}


void ctrlSetRefWeigts(CtrlPidParam pid, float beta, float gamma) {
    pid->beta = beta;
    pid->gamma = gamma;
}

void ctrlSetSaturation(CtrlPidParam pid, float max, float min) {
    pid->umax = max;
    pid->umin = min;
}

unsigned char ctrlIsRunning(CtrlPidParam pid) {
    return pid->running;
}

void ctrlStart(CtrlPidParam pid) {
    pid->running = 1;
}

void ctrlStop(CtrlPidParam pid) {
    pid->running = 0;
}
