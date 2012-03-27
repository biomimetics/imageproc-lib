/*
 * Copyright (c) 2008-2010, Regents of the University of California
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
 * Real time clock
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2010-06-16     Initial release
 *                      
 * Notes:
 *  - MCU resources requied for this module:
 *      Timer8 & Timer9 are used for a 32-bit timer.
 *  - This module may need to be renamed to rtclock (real time clock).
 *
 */

#include "timer.h"
#include "stopwatch.h"

#define TMR_MSW         TMR9HLD
#define TMR_LSW         TMR8
#define TIME_FACTOR     5   // resolution is 0.2us for prescale of 8

typedef union {
    unsigned long time;
    struct {
        unsigned int lsw;
        unsigned int msw;
    } half;
} Clock;


/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/

// to save the time when swatchTic() is called.
static Clock tictoc_;

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

static void swatchSetupPeripheral(void);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void swatchSetup(void) {
    swatchSetupPeripheral();
    swatchReset();
}

void swatchReset(void) {
    // do not change the order of the following two lines.
    TMR_MSW = 0;
    TMR_LSW = 0;
}

unsigned long swatchTic(void) {
    // do not change the order of the following two lines.
    tictoc_.half.lsw = TMR_LSW;
    tictoc_.half.msw = TMR_MSW;
    return (tictoc_.time)/TIME_FACTOR;
}

unsigned long swatchToc(void) {
    Clock toc;
    // do not change the order of the following two lines.
    toc.half.lsw = TMR_LSW;
    toc.half.msw = TMR_MSW;
    return (toc.time - tictoc_.time)/TIME_FACTOR;
}


void swatchDelayUs(unsigned long delay) {
    Clock swatch;
    unsigned long exit_time;

    swatch.half.lsw = TMR_LSW;
    swatch.half.msw = TMR_MSW;
    exit_time = swatch.time + delay*TIME_FACTOR;
    
    while(swatch.time < exit_time) {
        swatch.half.lsw = TMR_LSW;
        swatch.half.msw = TMR_MSW;
    }
}


void swatchDelayMs(unsigned long delay) {
    swatchDelayUs(delay*1000);
}



/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., static functions.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/


/*****************************************************************************
* Timer8 & Timer9 are used for swatch module.
* No Interrupt is associated with Timer8.
* T8PER does not mean anything.
* Timer increase by one every 0.2us  
* Timer8 & Timer9 should be set up as 32-bit timer with prescale of 8
* Then, the 32-bit timer will increase its value by one every 0.2us.
* Interrupt for Timers is not used.
*****************************************************************************/
static void swatchSetupPeripheral(void) {
    unsigned int T8CONvalue, T8PERvalue;
    T8CONvalue = T8_OFF & T8_IDLE_CON & T8_GATE_OFF &
                 T8_PS_1_8 & T8_32BIT_MODE_ON & T8_SOURCE_INT;
    // prescale 1:8
    // Period is set so that period = 1us (1MHz), MIPS = 40MHz
    // value = Fcy/(prescale*Ftimer)
    T8PERvalue = 40;    // this value doesn't really mean anything here.
    OpenTimer8(T8CONvalue, T8PERvalue);	
    T8CONbits.TON = 1;
}




