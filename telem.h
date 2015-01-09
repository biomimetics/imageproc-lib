/*
 * Copyright (c) 2010-2014, Regents of the University of California
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
 * Generalized interface for telemetry recording and readback.
 *
 * by Andrew Pullin
 *
 * v0.1
 *
 * Revisions:
 *  Andrew Pullin       2014       Initial publish to imageproc-lib
 *
 * Usage:
 * There are three requed macros to be defined, in settings.h:
 *       #define TELEM_TYPE myTelemStruct_t
 *       #define TELEM_INCLUDE "my_telem.h"
 *       #define TELEMPACKFUNC(x) myTelemGetData(x)
 *
 * The header file referenced by TELEM_INCLUDE must be defined.
 * See https://github.com/biomimetics/roach/blob/master/lib/vr_telem.h for an
 * example of how to set up this file.
 * Similarly, the matching C file that implements TELEMPACKFUNC(x) must exist.
 * See https://github.com/biomimetics/roach/blob/master/lib/vr_telem.c for an
 * example implementation matching he header above.
 */


#ifndef __TELEM_H
#define __TELEM_H

#include "settings.h" //Required to set telemetry type
#include TELEM_INCLUDE

#ifndef TELEM_TYPE
#error "A telemtry type is not defined."
#endif

#include <stdint.h>

//Telemetry packet structure
typedef struct {
    uint32_t sampleIndex;
    uint32_t timestamp;
    TELEM_TYPE telemData;
} telemStruct_t;

#define TELEM_STREAM_OFF  0
#define TELEM_STREAM_ON   1

// Prototypes
void telemSetup(); //To be called in main
void telemReadbackSamples(unsigned long, unsigned int src_addr);
void telemSendDataDelay(telemStruct_t* sample, int delaytime_ms, unsigned int src_addr);
void telemSaveData(telemStruct_t *data);
void telemSetSamplesToSave(unsigned long n);
void telemErase(unsigned long);
void telemSetSkip(unsigned int skipnum);
void telemSetStartTime(void);
void telemGetSample(unsigned long sampNum, unsigned int sampLen, unsigned char *data);
void telemSaveNow();

#endif  // __TELEM_H
