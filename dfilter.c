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
 */

#include <stdlib.h>
#include "dfilter.h"


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

float dfilterApply(DigitalFilter f, float x)
{
    int i;
    //unsigned char idx = f->index;
    float y = f->xcoef[0] * x;

    for (i = 1; i <= f->order; ++i) {
        y += f->xcoef[i]*f->xold[f->index] - f->ycoef[i]*f->yold[f->index];
        f->index = (f->index == f->order-1)? 0 : f->index + 1;
    }

    f->index = (f->index == 0)? f->order - 1 : f->index - 1;
    f->xold[f->index] = x;
    f->yold[f->index] = y;

    return y;
}


DigitalFilter dfilterCreate(unsigned char order, FilterType type,
                float* xcoeffs, float* ycoeffs)
{
    int i;

    if (order == 0) return NULL;

    DigitalFilter f = (DigitalFilter)malloc(sizeof(DigitalFilterStruct));
    f->order = order;
    f->type = type;
    f->xcoef = (float*)malloc((order+1)*sizeof(float));
    f->ycoef = (float*)malloc((order+1)*sizeof(float));
    f->xold = (float*)malloc(order*sizeof(float));
    f->yold = (float*)malloc(order*sizeof(float));
    f->index = 0;

    for(i = 0; i < order; ++i) {
        f->xold[i] = 0;
        f->yold[i] = 0;
        f->xcoef[i] = xcoeffs[i];
        f->ycoef[i] = ycoeffs[i];
    }

    f->xcoef[order] = xcoeffs[order];
    f->ycoef[order] = ycoeffs[order];

    return f;
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
    return (f == NULL)? 0.0 : f->yold[f->index];
}

float dfilterGetLatestInputValue(DigitalFilter f)
{
    return (f == NULL)? 0.0 : f->xold[f->index];
}

unsigned char dfilterGetIndex(DigitalFilter f)
{
    return f->index;
}

void dfilterDelete(DigitalFilter f)
{
    free(f->xcoef);
    free(f->ycoef);
    free(f->xold);
    free(f->yold);
    free(f);
}
