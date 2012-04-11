/**
 * Copyright (c) 2010-2012, Regents of the University of California
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
 * Orientation Estimation Module (Quaternion and Binary Angle Representation)
 *
 *  by Humphrey Hu 
 *	v.beta
 *
 */

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "telemetry.h"
#include "bams.h"

typedef struct {
    float yaw;
    float pitch;
    float roll;
    unsigned long timestamp;
} PoseEstimateStruct;

typedef PoseEstimateStruct *PoseEstimate;

/*****************************************************************************
* Function Name : attSetup
* Description   : Sets up module for operation
* Parameters    : Estimator period in seconds
* Return Value  : None                                                     
*****************************************************************************/
void attSetup(float ts);
void attReset(void);

void attZero(void);

float attGetPitch(void);
float attGetRoll(void);
float attGetYaw(void);

bams16_t attGetPitchBAMS(void);
bams16_t attGetRollBAMS(void);
bams16_t attGetYawBAMS(void);

void attGetPose(PoseEstimate pose);

unsigned char attIsRunning(void);
void attSetRunning(unsigned char flag);

void attEstimatePose(void);

void attUpdateTelemetryB(TelemetryB);

#endif  // __ATTITUDE_H


