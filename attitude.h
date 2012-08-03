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
 *	v.0.4
 *
 */

#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "bams.h"
#include "quat.h"

typedef struct {
    float yaw;
    float pitch;
    float roll;
    unsigned long timestamp;
} PoseEstimateStruct;

typedef PoseEstimateStruct *PoseEstimate;

/**
 * Initialize module for operation.
 * @param ts - Estimation period in seconds
 */
void attSetup(float ts);

/**
 * Reset module to initial state and estimate to default.
 */
void attReset(void);

/**
 * Use accelerometer to estimate pitch and roll. Requires XL values be
 * read from the IC already.
 */
void attZero(void);


/**
 * See if module is running or not.
 * @return 0 if not running, 1 if running
 */
unsigned char attIsRunning(void);

/**
 * Set the module running state.
 * @param flag - State to set to
 */
void attSetRunning(unsigned char flag);
void attStart(void);
void attStop(void);

/**
 * Fetch the pitch/roll/yaw angle in radians.
 * @return Pitch/roll/yaw angle in radians
 */
float attGetPitch(void);
float attGetRoll(void);
float attGetYaw(void);

/**
 * Fetch the pitch/roll/yaw angle in binary angle units.
 * @return Pitch/roll/yaw angle in BAMS
 */
bams16_t attGetPitchBAMS(void);
bams16_t attGetRollBAMS(void);
bams16_t attGetYawBAMS(void);

/**
 * Get internal pose quaternion
 */
void attGetQuat(Quaternion *quat);

/**
 * Fetch pitch, roll, and yaw in radians.
 * @param pose - Estimate struct to write into
 */
void attGetPose(PoseEstimate pose);

/**
 * Compute pose estimate. This method should be called at a fixed
 * frequency as specified in module initialization.
 */
void attEstimatePose(void);

#endif  // __ATTITUDE_H

