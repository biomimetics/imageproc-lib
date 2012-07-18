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
* Quaternion Library
*
* by Humphrey Hu
*
* v. beta 
*/

#ifndef __QUATERNION_H
#define __QUATERNION_H

// Quaternion object
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

/**
 * Copy a quaternion
 * @param dst - Pointer to destination
 * @param src - Pointer to source
 */
void quatCopy(Quaternion *dst, Quaternion *src);

/**
 * Multiply two quaternions (q1*q2)
 * @param q1 - Pointer to operand 1
 * @param q2 - Pointer to operand 2
 * @param result - Pointer to result storage
 * @note 1700 cycles
 */
void quatMult(Quaternion *q1, Quaternion *q2, Quaternion *result);

/**
 * Normalize a quaternion
 * @param q - Pointer to quaternion to normalize
 * @note 1200 cycles
 */
void quatNormalize(Quaternion *q);

/**
 * Conjugate a quaternion
 * @param q - Pointer to operand
 * @param result - Pointer to result storage
 */
void quatConj(Quaternion *q, Quaternion *result);

/**
 * Rotate a vector by a quaternion
 * @param q1 - Pointer to rotation quaternion
 * @param v - Pointer to vector to rotate (w component should be 0)
 * @param result - Pointer to result storage
 */
void quatRotate(Quaternion *q1, Quaternion *v, Quaternion *result);

#endif
