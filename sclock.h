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

// Handles initialization of required peripherals and resets time to 0.
void sclockSetup(void);

unsigned long sclockGetGlobalTicks(void);
unsigned long sclockGetGlobalMillis(void);

unsigned long sclockGetLocalTicks(void);
unsigned long sclockGetLocalMillis(void);

unsigned long sclockGetOffsetTicks(void);
unsigned long sclockGetOffsetMillis(void);

void sclockSetOffsetTicks(unsigned long offset);
void sclockSetOffsetMillis(unsigned long offset);

#endif //  __SCLOCK_H
