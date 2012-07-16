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
* v.0.2
*/

#ifndef __DFILTER_H
#define __DFILTER_H

// At this moment, only floating point is available.
// It would be nice to have different types such as double precision
// and fixed point numbers.
typedef enum { 
    FILTER_TYPE_FLOAT = 0, 
    FILTER_TYPE_DOUBLE
} FilterType;

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
*/

#define MAX_FILTER_ORDER        (5)
// TODO: Add ready bit? Magic number?
typedef struct {
    unsigned char order;     // order = n
    FilterType type;        // float, double, or fixed point (short or long)
    float xcoef[MAX_FILTER_ORDER + 1];  // # of coeffs = n+1
    float ycoef[MAX_FILTER_ORDER + 1];  // # of coeffs = n
    float yold[MAX_FILTER_ORDER + 1];       // n prev y values
    float xold[MAX_FILTER_ORDER + 1];       // n prev x values
} DigitalFilterStruct;

typedef DigitalFilterStruct* DigitalFilter;

DigitalFilter __attribute__ ((deprecated)) dfilterCreate(unsigned char order, FilterType type,
float* xcoeffs, float* ycoeffs);

/**
* Initialize a filter
* @param f - Pointer to filter object
* @param order - Order of filter
* @param type - Filter representation type
* @param xcoeffs - Input coefficients array
* @param ycoeffs - Output coefficients array
*/
void dfilterInit(DigitalFilter f, unsigned char order, FilterType type,
float* xcoeffs, float* ycoeffs);

float dfilterApply(DigitalFilter f, float x);

float* dfilterGetOutputValues(DigitalFilter f);

float* dfilterGetInputValues(DigitalFilter f);

/**
* Get previous output from filter
* @param f - Pointer to filter object
* @return - Previous output
*/
float dfilterGetLatestOutputValue(DigitalFilter f);

/**
* Get previous input to filter
* @param f - Pointer to filter object
* @return - Previous input
*/
float dfilterGetLatestInputValue(DigitalFilter f);

unsigned char __attribute__ ((deprecated)) dfilterGetIndex(DigitalFilter f);
void __attribute__ ((deprecated)) dfilterDelete(DigitalFilter f);


#endif  // __DFILTER_H
