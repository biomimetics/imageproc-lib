/*
 * Copyright (c) 2008-2011, Regents of the University of California
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
 * Header for the OmniVision OV7660 Camera (ovcam) Interface
 *
 * by Fernando L. Garcia Bermudez
 *
 * v.beta
 *
 * Usage:
 *  #include "ovcam.h"
 *
 *  // Initialize camera
 *  ovcamSetup();
 *
 *  unsigned int imcols=160, imrows=100, rowcnt;
 *  unsigned char image[imrows][imcols];
 *
 *  // Throw out frame fraction
 *  ovcamWaitForNewFrame();
 *
 *  // Capture rows into array
 *  for (rowcnt = 0; rowcnt < imrows; rowcnt++)
 *  {
 *      ovcamGetRow(image[rowcnt]);
 *  }
 *
 *  // The array now contains a 160x100 image
 *
 */

#ifndef __OVCAM_H
#define __OVCAM_H


#include <xc.h>


// OVCAM Pins
#if defined(__IMAGEPROC1)

    #define OVCAM_REG2_EN   _LATC14
    #define OVCAM_PWDN      _LATC13
    #define OVCAM_VSYNC     _RD8
    #define OVCAM_HREF      _RD9

#elif (defined(__IMAGEPROC2) || defined(__MIKRO))

    #define OVCAM_PWDN      _LATC14
    #define OVCAM_VSYNC     _RC13
    #define OVCAM_HREF      _RF0

#endif


// Configures the camera through I2C(SCCB).
void ovcamSetup (void);
// !!! Should rely on function above instead of the following one !!!
void ovcamSetupOV7660 (void);
static inline void __attribute__ ((deprecated)) SCCB_SetupOV7660 (void) {
        ovcamSetupOV7660(); }

// Modifies the content of a configuration register.
//
// Parameters : subaddr = register address,
//              data = modified contents of the register.
void ovcamWriteRegister (unsigned char subaddr, unsigned char data);
static inline void __attribute__ ((deprecated)) SCCB_Write (unsigned char
        subaddr, unsigned char data) { ovcamWriteRegister(subaddr, data); }

// Reads the content of a configuration register.
//
// Parameters : subaddr = register address.
unsigned char ovcamReadRegister (unsigned char subaddr);
static inline unsigned char __attribute__ ((deprecated)) SCCB_Read (unsigned
        char subaddr) { return ovcamReadRegister(subaddr); }

// Captures a row from the camera.
//
// Parameters : a pointer to an array of 160 characters
extern void ovcamGetRow (void *row);

// Parameters: a pointer to buffer and number of pixels to capture
extern void ovcamGetPixels(void *row, unsigned int row_len);

static inline void __attribute__ ((deprecated)) getRow (void *row) {
        ovcamGetRow(row); }

// Captures a frame from the camera.
//
// Parameters : a pointer to an array of 160x100 characters
//
// TODO (fgb) : Need to figure out why it throws an address error.
//extern void ovcamGetFrame (void *frame);

// Waits until a new frame starts, discarding current frame fraction
static inline void ovcamWaitForNewFrame (void)
{
    while(OVCAM_VSYNC);
    while(!OVCAM_VSYNC);
}

// Turns the camera on
static inline void ovcamTurnOn (void)
{
    #if defined(__IMAGEPROC1)
    OVCAM_REG2_EN = 1;
    #endif
    OVCAM_PWDN = 0;
}

// Turns the camera off
static inline void ovcamTurnOff (void)
{
    #if defined(__IMAGEPROC1)
    OVCAM_REG2_EN = 0;
    #endif
    OVCAM_PWDN = 1;
}


#endif // __OVCAM_H
