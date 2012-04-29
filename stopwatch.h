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
 * Usage:
 *   #include  "stopwatch.h"
 *
 *   unsigned long time_elapsed;
 *
 *   // initialize stopwatch module
 *   swatchSetup();
 *
 *   // reset the timer with 0
 *   swatchReset();
 *
 *   swatchTic();
 *   // delay for 1 sec
 *   swatchDelayUs(1000000);
 *
 *   // delay for 1 sec
 *   swatchDelayMs(1000);
 *
 *   time_elapsed = swatchToc();
 *   // time_elapsed will hold a value of ~2,000,000.
 *
 * TODO: may need to change the name of this module to rtclock (real time clock)
 */

#ifndef __STOPWATCH_H
#define __STOPWATCH_H

/******************************************************************************
* Function Name : swatchSetup
* Description   : Initialize stopwatch module.
* Parameters    : None
* Return Value  : None
******************************************************************************/
void swatchSetup(void);

/******************************************************************************
* Function Name : swatchSetup
* Description   : Reset the timer value with 0.
* Parameters    : None
* Return Value  : None
******************************************************************************/
void swatchReset(void);

/******************************************************************************
* Function Name : swatchTic
* Description   : You know how tic toc works...... :D
* Parameters    : None
* Return Value  : The current time value
******************************************************************************/
unsigned long swatchTic(void);

/******************************************************************************
* Function Name : swatchToc
* Description   : You know how tic toc works...... :D
* Parameters    : None
* Return Value  : Time elapsed since swatchTic() was called
******************************************************************************/
unsigned long swatchToc(void);

/*****************************************************************************
* Function Name : swatchDelayUs
* Description   : Delay
* Parameters    : dalay in microseconds.
*                 The delay value should not exceed 2^32/5 = 858,993,459
* Return Value  : None
* Note: delay_us defined in utils.h is also a good alternative.
*****************************************************************************/
void swatchDelayUs(unsigned long delay);

/*****************************************************************************
* Function Name : swatchDelayMs
* Description   : Delay
* Parameters    : dalay in miliseconds.
*                 The delay value should not exceed 2^32/5000 = 858,993
* Return Value  : None
* Note: delay_ms defined in utils.h is also a good alternative.
*****************************************************************************/
void swatchDelayMs(unsigned long delay);

#endif //  __STOPWATCH_H
