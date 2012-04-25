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
 */

#ifndef __DFILTER_H
#define __DFILTER_H


// At this moment, only floating point is available.
// It would be nice to have different types such as double precision
// and fixed point numbers.
typedef enum {FILTER_TYPE_FLOAT, FILTER_TYPE_DOUBLE} FilterType;

/*
 * xcoef -> b0 : b1 : b2 : ... : bn
 * ycoef -> a0 : a1 : a2 : ... : an; a0 is always 1, but never used anyway.
 * xold -> ... : x[k-n-1]: x[k-n] : x[k-1] : x[k-2] : x[k-3] : ...
 * yold -> ... : y[k-n-1]: y[k-n] : y[k-1] : y[k-2] : y[k-3] : ...
 * index -> the location of x[k-1] in the xold array.
 * When a new input x[k] at the time step k comes into the filter,
 * the output y is calculated by
 * y[k] = b0*x[k] + b1*x[k-1] + ... + bn*x[k-n] - a1*y[k-1] - ... - an*y[k-n].
 * Then we need to replace y[k-n] and x[k-n] with y[k] and x[k], respectively.
 * The new index value should be updated to locate x[k-n] (or x[k] after replacing)
 */
typedef struct {
    unsigned char order;     // order = n
    FilterType type;        // float, double, or fixed point (short or long)
    float *xcoef;  // # of coeffs = n+1
    float *ycoef;  // # of coeffs = n
    float *yold;    // n prev y values
    float *xold;    // n prev x values
    unsigned char index;
} DigitalFilterStruct;

typedef DigitalFilterStruct* DigitalFilter;

// just put 0 or FILTER_TYPE_FLOAT for the type argument.
DigitalFilter dfilterCreate(unsigned char order, FilterType type,
                float* xcoeffs, float* ycoeffs);

float dfilterApply(DigitalFilter f, float x);

float* dfilterGetOutputValues(DigitalFilter f);

float* dfilterGetInputValues(DigitalFilter f);

/*****************************************************************************
* Function Name : dfilterGetLatestInputValues
* Description   : Get the latest output value entered from the filter.
* Parameters    : digital filter
* Return Value  : The latest output value from the filter
*****************************************************************************/
float dfilterGetLatestOutputValue(DigitalFilter f);


/*****************************************************************************
* Function Name : dfilterGetLatestInputValues
* Description   : Get the latest input value entered to the filter.
* Parameters    : digital filter
* Return Value  : The latest input value to the filter
*****************************************************************************/
float dfilterGetLatestInputValue(DigitalFilter f);


/*****************************************************************************
* Function Name : dfilterGetIndex
* Description   : Get the index value of a digital filter.
* Parameters    : digital filter
* Return Value  : The index value of the filter
*****************************************************************************/
unsigned char dfilterGetIndex(DigitalFilter f);


/*****************************************************************************
* Function Name : dfilterDeleteFilter
* Description   : Delete the filter to free the memory allocated for the filter.
* Parameters    : The filter to be deleted.
* Return Value  : None
*****************************************************************************/
void dfilterDelete(DigitalFilter f);


#endif  // __DFILTER_H
