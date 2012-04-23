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
 *
 * Revisions:
 *  Humphrey Hu 2011-09-03    Initial implementation
 *                      
 * Note:
 *	- Uses 16-bit unsigned integer counters
 */
 
// ==== REFERENCES ==========================================
#include "counter.h"
#include <stdlib.h>
#include "utils.h"

// ==== FUNCTION BODIES =====================================
Counter cntrCreate(void) {
   
   Counter counter = (Counter) malloc(sizeof(CounterStruct));
   counter->valid = 1;
   counter->value = 0;
   return counter;
   
}

void cntrDelete(Counter counter) {

   if(counter != NULL) {
   	free(counter);
   }

}

unsigned int cntrRead(Counter counter) {

   unsigned int val;
   CRITICAL_SECTION_START;
   val = counter->value;
   CRITICAL_SECTION_END;
   return val;
   
}

void cntrIncrement(Counter counter) {

   CRITICAL_SECTION_START;
   counter->value++;
   CRITICAL_SECTION_END;

}

void cntrAdd(Counter counter, unsigned int val) {

   CRITICAL_SECTION_START;
   counter->value += val;
   CRITICAL_SECTION_END;

}

void cntrSet(Counter counter, unsigned int value) {

   CRITICAL_SECTION_START;
   counter->value = value;
   CRITICAL_SECTION_END;

}
