/*
 * Copyright (c) 2011-2012, Regents of the University of California
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
 * Digital filter module
 *
 * by Stanley S. Baek
 *
 * v.alpha
 *
 * Revisions:
 *  Stanley S. Baek             2011-01-28  Initial release
 *  Fernando L. Garcia Bermudez 2012-04-24  Changed module name to dfilter, to
 *                                          prevent collisions with Microchip's
 *                                          dsp module.
 *  Humphrey Hu                 2012-07-13  Switched to static allocation
 */

#include <stdlib.h>
#include "dfilter.h"

//#define NaN(f) ( ((((char *)&f)[3] & 0x7f) == 0x7f ) && (((char *)&f)[2] & 0x80) )

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
float dfilterApply(DigitalFilter f, float x)
{
    float y;
    unsigned int i;
        
    f->xold[0] = x;
    f->yold[0] = 0.0;
    y = 0;
        
    for (i = 0; i <= f->order; i++) {
        y += f->xcoef[i]*f->xold[i] + f->ycoef[i]*f->yold[i];
    }
    
    memmove(&f->xold[1], &f->xold[0], sizeof(float)*(f->order));
    memmove(&f->yold[1], &f->yold[0], sizeof(float)*(f->order));
    
    f->yold[1] = y;

    return y;
}


DigitalFilter dfilterCreate(unsigned char order, FilterType type,
                float* xcoeffs, float* ycoeffs)
{
    return NULL;
}

void dfilterInit(DigitalFilter f, unsigned char order, FilterType type,
                    float* xcoeffs, float* ycoeffs) {

    memset(f, 0x00, sizeof(DigitalFilterStruct));
    
    if(order > MAX_FILTER_ORDER) { return; }
    f->order = order;
    f->type = type;
    memcpy(&f->xcoef, xcoeffs, sizeof(float)*(order + 1));
    memcpy(&f->ycoef, ycoeffs, sizeof(float)*(order + 1));
    
}


float* dfilterGetOutputValues(DigitalFilter f)
{
    return f->yold;
}

float* dfilterGetInputValues(DigitalFilter f)
{
    return f->xold;
}

float dfilterGetLatestOutputValue(DigitalFilter f)
{
    return (f == NULL) ? 0.0 : f->yold[1];
}

float dfilterGetLatestInputValue(DigitalFilter f)
{
    return (f == NULL) ? 0.0 : f->xold[1];
}

// TODO: Deprecate
unsigned char dfilterGetIndex(DigitalFilter f)
{
    //return f->index;
    return 0;
}

void dfilterDelete(DigitalFilter f)
{
    return;
}
