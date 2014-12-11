/*
 * Copyright (c) 2007-2012, Regents of the University of California
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
 * Battery supervisor module
 *
 * by Fernando L. Garcia Bermudez and Stanley S. Baek
 *
 * v.0.2
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez     2007-8-8    Initial release
 *  Aaron M. Hoover                 2009-4-2    Moved code to interrupts.c/h
 *  Stanley S. Baek                 2010-7-5    Created this module
 *  Humphrey Hu                     2012-07-12  Added event callback
 *
 * Notes:
 *  - Uses an external interrupt (INT0 for ImageProc1, INT2 for ImageProc2).
 */

#include "battery.h"
#include "ports.h"
#include "pwm.h"
#include "utils.h"
#include <stdlib.h>

// =========== Static Variables ================================================
BatteryEventISR event_callback;

// =========== Function Stubs ==================================================
static void batHandleISR(void);
static void batDefaultCallback(void);

// =========== Public Methods ==================================================
void batSetup(void) {

#if defined(__IMAGEPROC1)
    ConfigINT0(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_7);
#elif defined(__IMAGEPROC2)
    ConfigINT2(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_7);
#endif

}

void batSetCallback(BatteryEventISR isr) {

    event_callback = isr;

}

// =========== Private Methods =================================================
static void batHandleISR(void) {

    if(event_callback == NULL) {
        batDefaultCallback();
    } else {
        event_callback();
    }

}

static void batDefaultCallback(void) {

    unsigned char i;

    CRITICAL_SECTION_START; // Disable interrupts

    LED_1 = 1;
    LED_2 = 1;
    #if defined(__IMAGEPROC2)
        LED_3 = 1;
    #endif

    // TODO (fgb) : Useful, but rarely used. Maybe move to another callback?
    #if defined(__LOWBATT_STOPS_MOTORS)
        // Stop any running motors
        for (i=1; i<=4; i++) { SetDCMCPWM(i, 0, 0); }
        CloseMCPWM();
    #endif

    // Slowly blink all LEDs 5 times (1 second interval)
    for (i=0; i<5; ++i) {
        LED_1 = ~LED_1;
        LED_2 = ~LED_2;
        #if defined(__IMAGEPROC2)
            LED_3 = ~LED_3;
        #endif
        delay_ms(1000);
    }

    CRITICAL_SECTION_END; // Re-enable interrupts

}

#if defined(__IMAGEPROC1)
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {

    batHandleISR();
    _INT0IF = 0;    // Clear the interrupt flag

}
#elif defined(__IMAGEPROC24) || defined(__IMAGEPROC25)
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {

    batHandleISR();
    _INT2IF = 0;    // Clear the interrupt flag

}
#endif
