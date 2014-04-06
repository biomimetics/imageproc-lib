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
 * Utility Functions
 *
 * by Fernando L. Garcia Bermudez, Aaron M. Hoover & Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez     2008-6-17   Added delay function prototypes
 *  Stanley S. Baek                 2009        Defined true/false & comparisons
 *  Aaron M. Hoover                 2010        Added interrupt disabling macros
 *  Andrrew Pullin                  2014        Changed to a BSP-include system
 *
 */

#ifndef __UTILS_H
#define __UTILS_H


#include <xc.h>

/*
 * This is now going to be done inside a board-specific bsp .h file
// Abstracting LEDs for easier control of their status
#if defined(__IMAGEPROC1)

    #define LED_1   _LATF0
    #define LED_2   _LATF1

#elif defined(__IMAGEPROC24) || defined(__IMAGEPROC25) || defined(__MIKRO)

    #define LED_1   _LATB12
    #define LED_2   _LATB13
    #define LED_3   _LATB14

#elif defined(__BASESTATION) || defined(__BASESTATION2)

    #define LED_0   _LATD0
    #define LED_1   _LATD1
    #define LED_2   _LATD2
    #define LED_3   _LATD3

#elif defined(__EXP16DEV)

    #define LED_0   _LATA0
    #define LED_1   _LATA1
    #define LED_2   _LATA2
    #define LED_3   _LATA3
    #define LED_4   _LATA4
    #define LED_5   _LATA5
    #define LED_6   _LATA6
    #define LED_7   _LATA7

#endif
 */

#define ON              1
#define OFF             0

/// Board Support Header
#if defined(__IMAGEPROC1)
    #include "bsp-ip1.h"
#elif defined(__IMAGEPROC24)
    #include "bsp-ip24.h"
#elif defined(__IMAGEPROC25)
    #include "bsp-ip25.h"
#elif defined(__BASESTATION)
    #include "bsp-basestation.h"
#elif defined(__BASESTATION2)
    #include "bsp-basestation2.h"
#elif defined(__MIKRO)
    #include "bsp-mikro.h"            //Not expected to be used
#elif defined(__EXP16DEV)
    #include "bsp-exp16dev.h"         //Not expected to be used
#endif

//Disable interrupts by wrapping code with this macro
//Example usage:
//
//      CRITICAL_SECTION_START
//          ...
//          <some important code like writing to shared data>
//          ...
//      CRITICAL_SECTION_END
#ifndef CRITICAL_SECTION_START
    #define CRITICAL_SECTION_START  char saved_ipl; SET_AND_SAVE_CPU_IPL(saved_ipl, 7);
    #define CRITICAL_SECTION_END RESTORE_CPU_IPL(saved_ipl);
#endif

// Define False/FALSE & True/TRUE for boolean
typedef enum {False, True} Boolean;

#ifndef FALSE
    #define FALSE   0
    #define TRUE    1
#endif


/*-----------------------------------------------------------------------------
 *          Comparison subroutines
-----------------------------------------------------------------------------*/

#ifndef max
    #define max(a,b)    ((a)>(b))?(a):(b)
#endif

#ifndef min
    #define min(a,b)    ((a)<(b))?(a):(b)
#endif

/*-----------------------------------------------------------------------------
 *          Delay subroutines
-----------------------------------------------------------------------------*/

// Wastes a number of microseconds
//
// Even though time is an int, it is limited by its assembly implementation to
// the value 8192. Thus, this function can delay up to ~8.2ms.
//
// Parameter : time = microseconds to waste
extern void delay_us (unsigned int time);

// Wastes a number of milliseconds
//
// This function is slightly less accurate than the microsecond delay routine
// and thus it is recommended to use the former within its limits.
//
// Parameter : time = milliseconds to waste
extern void delay_ms (unsigned int time);


/*-----------------------------------------------------------------------------
 *          Commonly used unions
-----------------------------------------------------------------------------*/

typedef union union_int
{
    int i;
    unsigned char c[2];
} intT;



#endif // __UTILS_H
