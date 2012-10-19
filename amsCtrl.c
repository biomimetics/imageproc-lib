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
 * ams PID Module
 *
 * by Duncan Haldane 10/18/2012
 *
 * v.0.1
 */


#include "ams-enc.h"
#include "pid.h"
#include "amsCtrl.h"

#define nPIDS = 2

//Create PID object
longpidObj amsPID[nPIDS];

//Hardware PID 
fractional ams_abcCoeffs[nPIDS][3] __attribute__((section(".xbss, bss, xmemory")));
fractional ams_controlHists[nPIDS][3] __attribute__((section(".ybss, bss, ymemory")));

void amsPIDSetup(void){
	unsigned char i;
	
	for(i=0; i < nPIDS; i++){
	#ifdef PID_HARDWARE
		amsPID[i].dspPID.abcCoefficients = ams_abcCoeffs;
		amsPID[i].dspPID.controlHistory = ams_controlHists;
	#endif
		pidInitPIDObj(&amsPID[i], AMS_DEFAULT_KP, AMS_DEFAULT_KI,
				AMS_DEFAULT_KD, AMS_DEFAULT_KAW, AMS_DEFAULT_KFF);
		amsPID[i].satValPos = 65536;		//Saturation values
		amsPID[i].satValNeg = -65536;
		amsPID[i].maxVal = 65536;
		amsPID[i].minVal = -65536;

		amsPID[i].onoff = PID_ON;
	}
}

void amsCtrlSetInput(unsigned char num, long state){
	pidSetInput(&amsPID[num], state);
}

void amsCtrlSetGains(unsigned char num, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pidSetGains(&amsPID[num], Kp, Ki, Kd, Kaw, ff);
}


	