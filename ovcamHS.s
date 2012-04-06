;
; Copyright (c) 2007-2011, Regents of the University of California
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
; OmniVision OV7660 Camera (ovcam) Interface
;
; by Fernando L. Garcia Bermudez
;
; v.beta
;
; Revisions:
;  Fernando L. Garcia Bermudez     2007-11-30    Initial release
;  Humphrey Hu                     2011-10-14    Added capture of a specified
;                                                number of pixels.
;

#include p33Fxxxx.inc


; Symbol definitions
.equiv  tmp,           w0
.equiv  arg1,           w1
.equiv  row_ptr,        w2
.equiv  row_len,        w3
.equiv  frame_ptr,      w8
.equiv  row_counter,    w9

.equiv  VSYNC_PORT,     PORTD
.equiv  VSYNC_PIN,      8
.equiv  HREF_PORT,      PORTF
.equiv  HREF_PIN,       0
.equiv  PIXEL,          PORTD


;------------------------------------------------------------------------------
;           Public functions
;------------------------------------------------------------------------------

.global _ovcamGetRow
.global _ovcamGetPixels
;.global _ovcamGetFrame

; After waiting for the start of a new row, it captures the first pixel with
; slightly different timing than all subsequent ones, saving these to
; consecutive addresses starting at the one passed as a pointer to the current
; row's first element.
_ovcamGetRow:

; Save row pointer in row_ptr
            mov     tmp, row_ptr

; Wait until a new row starts
wROWe:      btst    HREF_PORT, #HREF_PIN
            bra     NZ, wROWe
wROWs:      btst    HREF_PORT, #HREF_PIN
            bra     Z, wROWs

; Capture image row
            mov     PIXEL, tmp      ; init
            mov.b   tmp, [row_ptr++]
            nop
            nop
            nop
            nop
getROW:     mov     PIXEL, tmp      ; rest
            mov.b   tmp, [row_ptr++]
            nop
            nop
            nop
            btst    HREF_PORT, #HREF_PIN
            bra     NZ, getROW

            return

_ovcamGetPixels:

; Save row pointer in row_ptr
            mov     tmp, row_ptr
            mov     arg1, row_len
; Wait until a new row starts
wPIXe:      btst    HREF_PORT, #HREF_PIN
            bra     NZ, wPIXe
wPIXs:      btst    HREF_PORT, #HREF_PIN
            bra     Z, wPIXs


; Capture arg # of pixels
; N = Number of cycles per loop
; N = 8*A for Ax subsample (1 out of A pixels captured)

            mov     PIXEL, tmp      ; init
            mov.b   tmp, [row_ptr++]
            repeat  #2 ;N - 6
            nop

getPIX:     mov     PIXEL, tmp      ; rest
            mov.b   tmp, [row_ptr++]
            repeat  #1  ;N - 7
            nop
            dec     row_len, row_len
            bra     NZ, getPIX

            return


; TODO (fgb): Throws an address error, need to make sure how to iterate through
;             the rows of the array, which is probably causing the problem.
; TODO (fgb): Image sizes should be passed as arguments
;_ovcamGetFrame:
;
;; Save frame pointer in frame_ptr
;            mov     tmp, frame_ptr
;
;; Wait until a new frame starts
;wFRAMEe:    btst    VSYNC_PORT, #VSYNC_PIN
;            bra     NZ, wROWe
;wFRAMEs:    btst    VSYNC_PORT, #VSYNC_PIN
;            bra     Z, wROWs
;
;; Capture image frame
;            mov     #100, row_counter
;
;getFRAME:   mov     frame_ptr, tmp
;            call    _ovcamGetRow
;            add     #160, frame_ptr
;            dec     row_counter, row_counter
;            bra     NZ, getFRAME
;
;            return


.end
