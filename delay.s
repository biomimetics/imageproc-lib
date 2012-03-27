; 
; Copyright (c) 2007-2010, Regents of the University of California
; All rights reserved.
; 
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
; 
; - Redistributions of source code must retain the above copyright notice,
;   this list of conditions and the following disclaimer.
; - Redistributions in binary form must reproduce the above copyright notice,
;   this list of conditions and the following disclaimer in the documentation
;   and/or other materials provided with the distribution.
; - Neither the name of the University of California, Berkeley nor the names
;   of its contributors may be used to endorse or promote products derived
;   from this software without specific prior written permission.
; 
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
; 
; 
; Delay subroutines
; 
; by Fernando L. Garcia Bermudez
; 
; v.release
; 
; Revisions:
;  Fernando L. Garcia Bermudez     2007-9-14    Mixed assembly-C implementation
;                                  2010-7-23    Full assembly implementation
; 

#include p33Fxxxx.inc

; Symbol definitions
.equiv  time, w0
.equiv  temp, w2


;------------------------------------------------------------------------------
;           Public functions
;------------------------------------------------------------------------------

.global _delay_us
.global _delay_ms

; Need to waste ~40 inst/us (for 40MIPS) and since assembly loop takes 5
; instructions, it should be repeated 8 times to elapse a us. Since temp is a
; 16-bit word, the maximum delay possible is ~8.2ms (time=8192).
_delay_us:

            mul.uu  time, #8, temp
            sub     #2, temp        ; #2 obtained testing btg delays
ELAPSED:    nop
            nop
            dec     temp, temp
            bra     NZ, ELAPSED

            return

; The inner loop, modelled after the _delay_us routine, is repeated "time"
; times wasting 1ms per iteration (1000us * 8 - 2 = 7998).
_delay_ms:

msELAPSED:  mov     #7998, temp
usELAPSED:  nop
            nop
            dec     temp, temp
            bra     NZ, usELAPSED
            dec     time, time
            bra     NZ, msELAPSED

            return


.end
