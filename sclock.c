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
 * Revisions:
 *  Stanley S. Baek     2010-06-16      Initial release
 *  Humphrey Hu         2012-02-20      Restructuring module as sclock.
 *
 * Notes:
 *  - This module requires Timer 8 and 9 for setting up a 32-bit timer.
 */

#include "timer.h"
#include "sclock.h"

#define TMR_MSW         (TMR9HLD)
#define TMR_LSW         (TMR8)
#define MILLIS_FACTOR   (625)

// Time is represented by a 32-bit number
typedef union {
    unsigned long time;
    struct {
        unsigned int lsw;
        unsigned int msw;
    } half;
} Time;

// =========== Private Variables ==============================================

static Time sclock_offset;

// =========== Private Function Prototypes ====================================

static void sclockSetupPeripheral(void);
static void sclockReset(void);

// =========== Public Functions ===============================================

void sclockSetup(void) {

    sclockSetupPeripheral();
    sclockReset();

}

unsigned long sclockGetGlobalTicks(void) {

    Time time;
    time.half.lsw = TMR_LSW;
    time.half.msw = TMR_MSW;
    return time.time + sclock_offset.time;

}

unsigned long sclockGetGlobalMillis(void) {

    return sclockGetGlobalTicks()/MILLIS_FACTOR;

}

unsigned long sclockGetLocalTicks(void) {

    Time time;
    time.half.lsw = TMR_LSW;
    time.half.msw = TMR_MSW;
    return time.time;

}

unsigned long sclockGetLocalMillis(void) {

    return sclockGetLocalTicks()/MILLIS_FACTOR;

}

unsigned long sclockGetOffsetTicks(void) {

    return sclock_offset.time;

}

unsigned long sclockGetOffsetMillis(void) {

    return sclockGetOffsetTicks()/MILLIS_FACTOR;

}

void sclockSetOffsetTicks(unsigned long offset) {

    sclock_offset.time = offset;

}

void sclockSetOffsetMillis(unsigned long offset) {

    sclockSetOffsetTicks(offset*MILLIS_FACTOR);

}

unsigned int sclockGetMillisFactor(void) {

    return MILLIS_FACTOR;

}


// =========== Private Functions ==============================================

/**
 * Timer ticks 625 times per millisecond with 64:1 prescale
 */
static void sclockSetupPeripheral(void) {

    unsigned int T8CONvalue, T8PERvalue;
    T8CONvalue =    T8_OFF &
                    T8_IDLE_CON &
                    T8_GATE_OFF &
                    T8_PS_1_64 &
                    T8_32BIT_MODE_ON &
                    T8_SOURCE_INT;
    T8PERvalue = 40;    // this value doesn't really mean anything here.
    OpenTimer8(T8CONvalue, T8PERvalue);
    T8CONbits.TON = 1;

}

static void sclockReset(void) {

    // do not change the order of the following two lines.
    TMR_MSW = 0;
    TMR_LSW = 0;
    sclock_offset.time = 0;

}
