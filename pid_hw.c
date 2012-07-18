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
 * Hardware PID module
 *
 * by Kevin Peterson
 *
 * v.0.1
 *
 * Revisions:
 *  Kevin Peterson      2012-04-05    Initial release
 *
 * Notes:
 *  - !!!!!!! PID gains must be between -1 and 1 !!!!!!!
 *  - This file is a wrapper for the built in dsp PID controller on the dsPIC
 *  - The memory model in the build options must be set to Large Data Model for
 *    this code to properly compile. Also, under "Project Properties/XC16
 *    (Global Options)/xc16-ld", add -ldsp-elf in the "Additional options" field
 */

#include "pid_hw.h"
#include <dsp.h>
#include <libq.h>


void pidHWCreate(tPID* controller, fractional* coeffs, fractional* hist) {
    // Set up pointers to the derived coefficents and controller histories
    controller->abcCoefficients = coeffs;
    controller->controlHistory = hist;

    // Initialize the PID controller
    PIDInit(controller);
}

void pidHWSetFloatCoeffs(tPID* controller, float Kp, float Ki, float Kd) {

    fractional kCoeffs[3];

    kCoeffs[0] = Q15(Kp);
    kCoeffs[1] = Q15(Ki);
    kCoeffs[2] = Q15(Kd);

    PIDCoeffCalc(&kCoeffs[0], controller);
}

void pidHWSetFracCoeffs(tPID* controller, fractional Kp, fractional Ki,
                                                         fractional Kd) {
    fractional kCoeffs[3];

    kCoeffs[0] = Kp;
    kCoeffs[1] = Ki;
    kCoeffs[2] = Kd;

    PIDCoeffCalc(kCoeffs, controller);
}

void pidHWSetReference(tPID* controller, fractional reference) {
    controller->controlReference = reference;
}

fractional pidHWRun(tPID* controller, fractional feedback) {

    controller->measuredOutput = feedback;
    PID(controller);

    return controller->controlOutput;
}
