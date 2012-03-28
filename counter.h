/*
 * Copyright (c) 2011, Regents of the University of California
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
 * Protected Counter Module
 *
 * by Humphrey Hu
 *
 * v 1.0
 */
 
#ifndef __COUNTER_H
#define __COUNTER_H

// TODO: Extend to multiple word lengths
typedef struct {
   // Config info (debug purposes)
   unsigned char valid;
   // State info
   unsigned int value;
} CounterStruct;

typedef CounterStruct* Counter;

/*****************************************************************************
 * Function Name : cntrCreate
 * Description   : Create a Counter object
 * Parameters    : None
 * Return Value  : Counter object reference
 *****************************************************************************/
Counter cntrCreate(void);

/*****************************************************************************
 * Function Name : cntrDelete
 * Description   : Delete a Counter object, freeing up its resources
 * Parameters    : Counter object reference
 * Return Value  : None
 *****************************************************************************/
void cntrDelete(Counter counter);

/*****************************************************************************
 * Function Name : cntrRead
 * Description   : Read a Counter object's value
 * Parameters    : Counter object reference
 * Return Value  : Counter object's current value
 *****************************************************************************/
unsigned int cntrRead(Counter counter);

// TODO: Increment by arbitrary values
/*****************************************************************************
 * Function Name : cntrIncrement
 * Description   : Increment a Counter object's value by one
 * Parameters    : Counter object reference
 * Return Value  : None
 *****************************************************************************/
void cntrIncrement(Counter counter);

/*****************************************************************************
 * Function Name : cntrAdd
 * Description   : Increment a Counter object's value by argument
 * Parameters    : Counter object reference, integer to add to counter
 * Return Value  : None
 *****************************************************************************/ 
void cntrAdd(Counter counter, unsigned int val);

/*****************************************************************************
 * Function Name : cntrSet
 * Description   : Set Counter object's value to argument
 * Parameters    : Counter object reference, value to set to
 * Return Value  : None
 *****************************************************************************/
void cntrSet(Counter counter, unsigned int val);

#endif	// __COUNTER_H
