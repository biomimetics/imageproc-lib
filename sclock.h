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
 * by Humphrey Hu
 * based on "stopwatch.c" by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek     2010-06-16      Initial release of "stopwatch.c"
 *  Humphrey Hu         2012-02-20      Initial release
 *  Humphrey Hu         2012-04-26      Renamed file to "sclock"
 * Notes:
 *  - MCU resources requied for this module:
 *      Timer8 & Timer9 are used for a 32-bit timer. 
 *  - Timer is set to 625 ticks per millisecond
 *
 * Usage:
 */

#ifndef __SYS_CLOCK_H
#define __SYS_CLOCK_H

void sclockSetup(void);
void sclockReset(void);

unsigned long sclockGetGlobalTicks(void);
unsigned long sclockGetGlobalMillis(void);

unsigned long sclockGetLocalTicks(void);
unsigned long sclockGetLocalMillis(void);

unsigned long sclockGetOffsetTicks(void);
unsigned long sclockGetOffsetMillis(void);

void sclockSetOffsetTicks(unsigned long offset); 
void sclockSetOffsetMillis(unsigned long offset);

#endif //  __STOPWATCH_H
