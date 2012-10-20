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
 *
 * Revisions:
 *  Andrew Pullin    2012-06-03    Initial release
 *
 * Notes:
 *  - For DSP hardware PID, the user MUST set the abcCoeffs and controlHists
 *    pointers for the dspPID variable. Those arrays are declared in special
 *    X and Y section memory, and cannot be allocated dynamically.
 */

#include "pid.h"
#include "pid_hw.h"
#include <stdlib.h> // for malloc


#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

//This is an option to force the PID outputs back to zero when there is no input.
//This was an attempt to stop bugs w/ motor twitching, or controller wandering.
//It may not be needed anymore.
#define PID_ZEROING_ENABLE 1

//Default gains
#ifdef PID_SOFTWARE
#define SOFT_GAIN_SCALER 512
#elif defined PID_HARDWARE
#include <dsp.h>
#define MOTOR_PID_SCALER 32
#endif

#define FF_SCALER 1024;

//////////////////////////

void pidUpdate(pidObj *pid, int feedback) {
    pid->error = pid->input - feedback;
#ifdef PID_SOFTWARE
    pid->p = (long) pid->Kp * pid->error;
    pid->i = (long) pid->Ki * pid->iState;
    //Filtered derivative action applied directly to measurement
    pid->d = ((long) pid->Kd * (long) pid->d * (long) SOFT_GAIN_SCALER) /
             ((long) pid->Kd + (long) pid->Kp * (long) pid->N) -
             ((long) pid->Kd * (long) pid->Kp * (long) pid->N *
             ((long) feedback - (long) pid->y_old)) / ((long) pid->Kd +
             (long) pid->Kp * (long) pid->N);

    pid->preSat = ((pid->p + pid->i + pid->d) * (long) pid->maxVal) /
            ((long) SOFT_GAIN_SCALER * (long)(pid->inputOffset));

    pid->iState += (long)(pid->error) + (long) (pid->Kaw) *
                   ((long) (pid->output) - (long) (pid->preSat)) /
                   ((long) SOFT_GAIN_SCALER);
    pid->y_old = feedback;

#elif defined PID_HARDWARE
    pid->dspPID.controlReference = pid->input;
    pid->preSat = pidHWRun(&(pid->dspPID), feedback); //Do PID calculate via DSP lib
#endif

    //Feedforward term
    long fftemp = (long)(pid->Kff)*(long)(pid->input);
    fftemp = fftemp / (long)FF_SCALER;
    pid->preSat += fftemp;

    if (pid->preSat  >  pid->satValPos) {
        pid->output = pid->satValPos;
    }
    else if (pid->preSat  <  pid->satValNeg) {
        pid->output = pid->satValNeg;
    }
    else {
        pid->output = pid->preSat;
    }
}

void pidInitPIDObj(pidObj* pid, int Kp, int Ki, int Kd, int Kaw, int Kff) {
    pid->input = 0;
    pid->dState = 0;
    pid->iState = 0;
    pid->output = 0;
    pid->y_old = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    //This is just a guess for the derivative filter time constant
    pid->N = 5;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->Kff = Kff;
    pid->onoff = PID_OFF;
    pid->error = 0;
#ifdef PID_HARDWARE
    pidHWSetFracCoeffs(&(pid->dspPID), pid->Kp, pid->Ki, pid->Kd);
    if((pid->dspPID.abcCoefficients != NULL) &&
            (pid->dspPID.controlHistory != NULL) ){
        PIDInit(&(pid->dspPID));
    }
#endif
}

void pidSetInput(pidObj* pid, int input_val) {
    pid->input = input_val;
    //zero out running PID values
    pid->iState = 0;
    pid->dState = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    //Seed the IIR filter; (TODO)
    pid->y_old = input_val;
}

void pidSetGains(pidObj* pid, int Kp, int Ki, int Kd, int Kaw, int Kff) {
    //Gains to our pidObj are always set
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->Kff = Kff;

    //If we are using the DSP core PID, we need to recalculate gain coeffs
#ifdef PID_HARDWARE
    //Gains are retrieved from the PID container object,
    //and need special setup for the DSP type PID calculation
    pidHWSetFracCoeffs(&(pid->dspPID), pid->Kp, pid->Ki, pid->Kd);
#endif
}

void pidOnOff(pidObj *pid, unsigned char state){
    pid->onoff = state;
}
