/**
 * Copyright (c) 2008-2012, Regents of the University of California
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
 * System Time Module
 *
 * by Stanley S. Baek and Humphrey Hu
 *
 * v.0.1
 *
 * Usage:
 *   #include "sclock.h"
 *   #include "utils.h"
 *
 *   unsigned long time_elapsed;
 *
 *   // initialize system time module
 *   sclockSetup();
 *
 *   // delay for 2 sec
 *   delay_ms(2000);
 *
 *   time_elapsed = sclockGetGlobalMillis();
 *   // time_elapsed should hold a value of ~2,000.
 */

#ifndef __SCLOCK_H
#define __SCLOCK_H


// Handles initialization of required timers and resets time to 0.
void sclockSetup(void);

// Requests number of ticks since the clock was started, added to a
// specified offset, if any.
//
// 625 ticks add up to a millisecond elapsed.
//
// Returns : global ticks
unsigned long sclockGetGlobalTicks(void);

// Requests number of milliseconds since the clock was started, added to a
// specified offset, if any.
//
// Returns : global milliseconds
unsigned long sclockGetGlobalMillis(void);

// Requests number of ticks since the clock was started.
//
// 625 ticks add up to a millisecond elapsed.
//
// Returns : local ticks
unsigned long sclockGetLocalTicks(void);

// Requests number of milliseconds since the clock was started.
//
// Returns : local milliseconds
unsigned long sclockGetLocalMillis(void);

// Requests the clock offset in ticks.
//
// Returns : offset ticks
unsigned long sclockGetOffsetTicks(void);

// Requests the clock offset in milliseconds.
//
// Returns : offset milliseconds
unsigned long sclockGetOffsetMillis(void);

// Sets the clock offset in ticks.
//
// Parameters : offset ticks
void sclockSetOffsetTicks(unsigned long offset);

// Sets the clock offset in milliseconds.
//
// Parameters : offset milliseconds
void sclockSetOffsetMillis(unsigned long offset);

// Requests how many ticks comprise a millisecond
//
// Returns: millisecond-to-ticks factor
unsigned int sclockGetMillisFactor(void);


#endif //  __SCLOCK_H
