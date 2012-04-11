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
 * Binary Angle Measurement System (BAMS) Implementation
 *
 * by Humphrey Hu
 *
 * v.alpha
 *
 * Revisions:
 *  Humphrey Hu			 2011-10-23	   Initial implementation
 * 
 */

#ifndef __BAMS_H
#define __BAMS_H

/*	
 *	Two's complement (binary angles) are convenient because they 	
 *	take apply integer math wrap-around to angle representation.
 *	Also, being fixed-point, addition and subtraction of angles is
 *	extremely fast and condusive to fast lookup tables.
 *
 *	16-bit binary angles map a range of [-pi, pi) to 
 *	[0x8000, 0x7FFF]. Note that this range does not include
 *	positive pi, such that there is only one value of pi.
 *	Though they are interpreted as signed integers, 
 *	they are equivalent to unsigned angles.
 *
 *	32-bit binary angles map a range of [-pi, pi) to 
 *	[0x80000000, 0x7FFFFFFF]. Most 32-bit functions have not been
 *	implemented yet.
 */
 
#define BAMS16_PI			(0x8000)
#define BAMS16_PI_2			(0x4000)
#define BAMS16_PI_4			(0x2000)

#define BAMS32_PI                       (0x80000000)
#define BAMS32_PI_2                     (0x40000000)
#define BAMS32_PI_4                     (0x20000000)

typedef int bams16_t;
typedef long bams32_t ;

// 16-bit Conversion Functions
float bams16ToFloatRad(bams16_t);
float bams16ToFloatDeg(bams16_t);
bams16_t floatToBams16Rad(float);
bams16_t floatToBams16Deg(float);
bams32_t bams16ToBams32(bams16_t b);

// 32-bit Conversion Functions
float bams32ToFloatRad(bams32_t);
float bams32ToFloatDeg(bams32_t);
bams32_t floatToBams32Rad(float);
bams32_t floatToBams32Deg(float);
bams16_t bams32ToBams16(bams32_t b);

// 16-bit Forward Trignonometric Functions
// Standard implementation offers ~2-3 digits of precision
float bams16Sin(bams16_t);		// 30 Cycles
float bams16Cos(bams16_t);		// 30 Cycles
float bams16Tan(bams16_t);		// 161 Cycles

// Fine implementation offers ~4-5 digits of precision
float bams16SinFine(bams16_t);	// 370 Cycles
float bams16CosFine(bams16_t);	// 370 Cycles
float bams16TanFine(bams16_t);	// 790 Cycles

float bams32Sin(bams32_t b);
float bams32Cos(bams32_t b);
float bams32Tan(bams32_t b);

float bams32SinFine(bams32_t b);
float bams32CosFine(bams32_t b);
float bams32TanFine(bams32_t b);

// 16-bit Reverse Trignonometric Functions
// Standard implementation offers ~1-2 digits of precision
bams16_t bams16Asin(float);		// 330 Cycles
bams16_t bams16Acos(float);		// 330 Cycles
bams16_t bams16Atan2(float, float); // 1000 Cycles

// Fine implementation offers ~3-4 digits of precision
bams16_t bams16AsinFine(float);	// 1100 Cycles
bams16_t bams16AcosFine(float);	// 1100 Cycles

#endif
