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
 *  v.beta
 *
 *
 * Revision History:
 *  Humphrey Hu         2011-10-08      Initial release
 *  Humphrey Hu         2011-12-06      Code refactor
 *  Humphrey Hu         2012-02-16      Updated interface to use objects
 */

#include "stopwatch.h"
#include "attitude.h"
#include "quat.h"
#include "xl.h"
#include "gyro.h"
#include <math.h>
#include "bams.h"
#include <stdlib.h>
#include "utils.h"

#define QUAT_POLE_LIMIT         (0.499)
#define PI                  (3.14159265)
#define PI_2                (1.57079633)
#define GRAVITY             (9.80665)   // Gravitational acceleration
#define GRAVITY_SQUARED         (96.1703842)

#define SCALE_CALIB_SAMPLES     (100)

// =========== Static Variables ===============================================

// State variables
// Orientation is represented internally as a quaternion
static Quaternion pose_quat;
static unsigned char is_ready, is_running;

// Attitude estimate terms
static bams16_t phi;
static bams16_t theta;
static bams16_t psi;
static unsigned long timestamp;

//static float xl_scale;

// Calculation parameters
static float sample_period = 0;

// =========== Function Prototypes ============================================

static void calculateEulerAngles(void);
//static void compensateDrift(void);
//static void measureXLScale(unsigned int num_samples);

// =========== Public Functions ===============================================

void attSetup(float ts) {

    is_ready = 0;
    is_running = 0;

    sample_period = ts;
    //measureXLScale(SCALE_CALIB_SAMPLES);
    xlReadXYZ();
    attZero();
    attReset();
    swatchReset();
    swatchTic();

    is_ready = 1;

}

void attReset(void) {

    if(!is_ready) { return; }

    pose_quat.w = 1.0;
    pose_quat.x = 0.0;
    pose_quat.y = 0.0;
    pose_quat.z = 0.0;

    phi = 0.0;
    theta = 0.0;
    psi = 0.0;

}

bams16_t attGetPitchBAMS(void) {
    return theta;
}

bams16_t attGetRollBAMS(void) {
    return phi;
}

bams16_t attGetYawBAMS(void) {
    return psi;
}

// TODO: Implement flip-buffer to avoid timestamp mismatch
void attGetPose(PoseEstimate pose) {
    pose->yaw = bams16ToFloatRad(psi);
    pose->pitch = bams16ToFloatRad(theta);
    pose->roll = bams16ToFloatRad(phi);
    pose->timestamp = timestamp;
}

unsigned char attIsRunning(void) {
    return is_running;
}

void attSetRunning(unsigned char flag) {
    is_running = flag;
}

void attZero(void) {

    float gxy, sina_2, xl[3], temp;
    bams16_t a_2;

    xlGetFloatXYZ(xl);

    // Convert frames so that z axis is oriented upwards, x is forward, y is side
    xl[2] = -xl[2];
    temp = xl[0];
    xl[0] = -xl[1];
    xl[1] = temp;

    gxy = sqrtf(xl[0]*xl[0] + xl[1]*xl[1]);
    a_2 = (BAMS16_PI_2 + bams16Atan2(xl[2], gxy))/2;
    sina_2 = bams16SinFine(a_2);

    pose_quat.w = bams16CosFine(a_2)*gxy;
    pose_quat.x = sina_2*(-xl[1]);
    pose_quat.y = sina_2*(xl[0]);
    pose_quat.z = 0.0;
    quatNormalize(&pose_quat);

}

// 12000 cycles?
void attEstimatePose(void) {

    Quaternion displacement_quat;
    float rate[3], norm, sina_2;
    bams32_t a_2;

    if(!is_ready) { return; }
    if(!is_running) { return; }

    gyroGetRadXYZ(rate); // Get last read gyro values
    timestamp = swatchToc(); // Record timestamp

    // Calculate magnitude and disiplacement
    norm = sqrtf(rate[0]*rate[0] + rate[1]*rate[1] + rate[2]*rate[2]);

    // Special case when no movement occurs due to simplification below
    if(norm == 0.0) {

        displacement_quat.w = 1.0;
        displacement_quat.x = 0.0;
        displacement_quat.y = 0.0;
        displacement_quat.z = 0.0;

    } else {

        // Generate displacement rotation quaternion
        // Normally this is w = cos(a/2), but we can delay normalizing
        // by multiplying all terms by norm
        a_2 = floatToBams32Rad(norm*sample_period)/2;
        sina_2 = bams32SinFine(a_2);

        displacement_quat.w = bams32CosFine(a_2)*norm;
        displacement_quat.x = sina_2*rate[0];
        displacement_quat.y = sina_2*rate[1];
        displacement_quat.z = sina_2*rate[2];
        quatNormalize(&displacement_quat);
    }

    // Apply displacement to pose
    quatMult(&pose_quat, &displacement_quat, &pose_quat);

    // Normalize pose quaternion to account for unnormalized displacement quaternion
    quatNormalize(&pose_quat);
    calculateEulerAngles();

}

static void calculateEulerAngles(void) {

    float temp1, temp2;

    // Convert back to Euler angles
    temp1 = pose_quat.w*pose_quat.y - pose_quat.z*pose_quat.x;
    if(temp1 > QUAT_POLE_LIMIT) {
        psi = 2*bams16Atan2(pose_quat.w, pose_quat.x);
        theta = -BAMS16_PI_2;
    } else if(temp1 < -QUAT_POLE_LIMIT) {
        psi = -2*bams16Atan2(pose_quat.w, pose_quat.x);
        theta = BAMS16_PI_2;
    } else {
        theta = -bams16Asin(2.0*temp1);
        temp1 = 2.0*(pose_quat.w*pose_quat.x + pose_quat.y*pose_quat.z);
        temp2 = 1.0 - 2.0*(pose_quat.x*pose_quat.x + pose_quat.y*pose_quat.y);
        phi = bams16Atan2(temp1, temp2);
        temp1 = 2.0*(pose_quat.w*pose_quat.z + pose_quat.x*pose_quat.y);
        temp2 = 1.0 - 2.0*(pose_quat.y*pose_quat.y + pose_quat.z*pose_quat.z);
        psi = bams16Atan2(temp1, temp2);
    }

}

// Quick accelerometer hack to help with estimation drift
//static void compensateDrift(void) {
//
//    float gxy, sina_2, sNorm, xl[3], temp, confidence;
//    float w_est, w_grav;
//    bams16_t a_2;
//    Quaternion grav_quat;
//
//    xlGetFloatXYZ(xl);
//
//    // Convert frames so that z axis is oriented upwards, x is forward, y is side
//    xl[2] = -xl[2];
//    temp = xl[0];
//    xl[0] = -xl[1];
//    xl[1] = temp;
//
//    sNorm = (xl[0]*xl[0] + xl[1]*xl[1] + xl[2]*xl[2])*xl_scale;
//
//    // High confidence when norm matches gravity
//    if(sNorm > GRAVITY_SQUARED) {
//        confidence = 1.0 - (sNorm - GRAVITY_SQUARED);
//    } else {
//        confidence = 1.0 - (GRAVITY_SQUARED - sNorm);
//    }
//    if(sNorm < 0.0) { sNorm = 0.0; }
//
//    gxy = sqrtf(xl[0]*xl[0] + xl[1]*xl[1]);
//    a_2 = (BAMS16_PI_2 + bams16Atan2(xl[2], gxy))/2;
//    sina_2 = bams16SinFine(a_2);
//
//    grav_quat.w = bams16CosFine(a_2)*gxy;
//    grav_quat.x = sina_2*(-xl[1]);
//    grav_quat.y = sina_2*(xl[0]);
//    grav_quat.z = 0.0;
//    quatNormalize(&grav_quat);
//
//    w_est = 1.0 - confidence;
//    w_grav = confidence;
//
//    pose_quat.w = pose_quat.w*w_est + grav_quat.w*w_grav;
//    pose_quat.x = pose_quat.x*w_est + grav_quat.x*w_grav;
//    pose_quat.y = pose_quat.y*w_est + grav_quat.y*w_grav;
//    pose_quat.z = pose_quat.z*w_est + grav_quat.z*w_grav;
//    quatNormalize(&pose_quat);
//
//}

//static void measureXLScale(unsigned int num_samples) {
//
//    float xl[3], sum[3], sNorm;
//    unsigned int i;
//
//    for(i = 0; i < num_samples; i++) {
//        xlReadXYZ();
//        xlGetFloatXYZ(xl);
//        sum[0] += xl[0];
//        sum[1] += xl[1];
//        sum[2] += xl[2];
//    }
//    sum[0] = sum[0]/num_samples;
//    sum[1] = sum[1]/num_samples;
//    sum[2] = sum[2]/num_samples;
//
//    sNorm = sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2];
//
//    xl_scale = (GRAVITY_SQUARED)/sNorm;
//
//}

