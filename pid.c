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
 * Generalized integer PID module
 *
 * by Andrew Pullin
 *
 * v.beta
 *
 * Revisions:
 *  Andrew Pullin    2012-06-03    Initial release
 *
 * Notes:
 * For DSP hardware PID, the user MUST set the abcCoeffs and controlHists
 * pointers for the dspPID variable. Those arrays are decalred in special
 * X and Y section memory, and cannot be allocated dynamically.
 */

#include "pid_hw.h"
#include "pid.h"
#include "timer.h"
#include "math.h"
#include<dsp.h>
#include <stdlib.h> // for malloc
#include "leg_ctrl.h" //ONLY for getT1_ticks, to be fixed later!


//Choose between software PID and DSP core hardware PID
#define PID_HARDWARE

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

//This is an option to force the PID outputs back to zero when there is no input.
//This was an attempt to stop bugs w/ motor twitching, or controller wandering.
//It may not be needed anymore.
#define PID_ZEROING_ENABLE 1

//////////////////////////

void pidSetup() {
    //Nothing to do here
}

void pidUpdate(pidObj *pid, int feedback) {
#ifdef PID_SOFTWARE
    pid->error = pid->input - feedback;
    pid->p = (long) pid->Kp * pid->error;
    pid->i = (long) pid->Ki * pid->iState;
    //Filtered derivative action applied directly to measurement
    pid->d = ((long) pid->Kd * (long) pid->d * (long) GAIN_SCALER) / ((long) pid->Kd + (long) pid->Kp * (long) pid->N) -
            ((long) pid->Kd * (long) pid->Kp * (long) pid->N * ((long) y - (long) pid->y_old)) /
            ((long) pid->Kd + (long) pid->Kp * (long) pid->N);

    pid->preSat = pid->feedforward + ((pid->p + pid->i + pid->d) * (long) pid->maxVal) / ((long) GAIN_SCALER * (long) (pid->inputOffset));
    //pid->preSat = ((pid->p + pid->i + pid->d) * (long)FULLTHROT) / ((long)GAIN_SCALER * (long)ADC_Offset);
    //pid->preSat = (pid->p * (long)FULLTHROT) / ((long)GAIN_SCALER * (long)ADC_MAX);

    if (pid->preSat > pid->satVal) {
        pid->output = pid->satVal;
    } else {
        pid->output = pid->preSat;
    }

    //pid->iState += (long)(pid->error) + (long)(pid->Kaw) * ((long)(pid->output) - (long)(pid->preSat));
    pid->iState += (long) (pid->error) + (long) (pid->Kaw) * ((long) (pid->output) - (long) (pid->preSat)) / ((long) GAIN_SCALER);
    pid->y_old = y;
#elif defined PID_HARDWARE
    int temp;
    pid->dspPID.controlReference = pid->input;
    temp = pidHWRun(&(pid->dspPID), feedback);   //Do PID calculate via DSP lib

    if (pid->dspPID.controlOutput  >  pid->satValPos) {
        temp = pid->satValPos;
    }
    else if (pid->dspPID.controlOutput  <  pid->satValNeg) {
        temp = pid->satValNeg;
    }

    pid->output = temp;
#endif
}

void pidInitPIDObj(pidObj* pid, int Kp, int Ki, int Kd, int Kaw, int ff) {
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
    pid->feedforward = ff;
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
    pid->start_time = getT1_ticks();
    //zero out running PID values
    pid->iState = 0;
    pid->dState = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    //Seed the IIR filter; (TODO)
    pid->y_old = input_val;
}

void pidSetGains(pidObj* pid, int Kp, int Ki, int Kd, int Kaw, int ff) {
    //Gains to our pidObj are always set
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->feedforward = ff;

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
