/*
 * Copyright (c) 2010, Regents of the University of California
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
 * Wrapper of SPI read/write functionality
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanlay S. Baek     2010-8-10    Initial release
 *
 * Notes:
 *  - Uses an SPI bus (SPI1 for ImageProc2, SPI2 for MikroElectronika).
 *  - TODO (stanbaek) : Implement DMA for SPI.
 */

#include "ipspi1.h"
#include "utils.h"
#include <stdio.h>
#include "lcd.h"

#if defined(__MIKRO)
    #define SPI_BUF         SPI2BUF
    #define SPI_CON1bits    SPI2CON1bits
    #define SPI_CON2        SPI2CON2
    #define SPI_STATbits    SPI2STATbits
    #define SPI_CS          _LATG9
    #define SLPTR           _LATF0      
#elif defined(__EXP16DEV)
    #define SPI_BUF         SPI2BUF
    #define SPI_CON1bits    SPI2CON1bits
    #define SPI_CON2        SPI2CON2
    #define SPI_STATbits    SPI2STATbits
    #define SPI_CS          _LATG9
    #define SLPTR           _LATB1
#elif defined(__BASESTATION)
    #define SPI_BUF         SPI2BUF
    #define SPI_CON1bits    SPI2CON1bits
    #define SPI_CON2        SPI2CON2
    #define SPI_STATbits    SPI2STATbits
    #define SPI_CS          _LATG9
    #define SLPTR           _LATE5
#else
    #define SPI_BUF         SPI1BUF
    #define SPI_CON1bits    SPI1CON1bits
    #define SPI_CON2        SPI1CON2
    #define SPI_STATbits    SPI1STATbits
    #define SPI_CS          _LATB2
    #define SLPTR           _LATB15
#endif


/******************************************************************************
* Function Name : configSPI 
* Description   : This routine sets up SPI bus for this module
* Parameters    : None
* Return Value  : None
*******************************************************************************/
void ipspi1Config(void) {

    // SPI interrupt is not used.
    //IFS0bits.SPI2IF = 0; // Clear the Interrupt Flag
    //IEC0bits.SPI2IE = 0; // Disable the Interrupt

    // SPI1CON1 Register Settings
    SPI_CON1bits.DISSCK = 0; // Internal Serial Clock is Enabled
    SPI_CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI_CON1bits.MODE16 = 0; // Communication is byte-wide (8 bits)
    SPI_CON1bits.SMP = 0; // Input data is sampled at middle of data output time
    SPI_CON1bits.SSEN = 0; // SS1 pin is used
    SPI_CON1bits.CKE = 1; // Serial output data changes on transition
                        // from active clock state to idle clock state
    SPI_CON1bits.CKP = 0; // Idle state for clock is a low level;
                            // active state is a high level
    SPI_CON1bits.MSTEN = 1; // Master mode Enabled

    // Set up SCK frequency of 1.25Mhz for 40 MIPS
    SPI_CON1bits.SPRE = 0b000; // Secondary prescale    8:1
    SPI_CON1bits.PPRE = 0b10; // Primary prescale       4:1

    // SPI2CON2 Register Settings
    SPI_CON2 = 0x0000; // Framed SPI2 support disabled

    // SPI2STAT Register Settings
    SPI_STATbits.SPISIDL = 1; // Discontinue module when device enters idle mode
    SPI_STATbits.SPIROV = 0; // Clear Overflow
    SPI_STATbits.SPIEN = 1; // Enable SPI module
}

/******************************************************************************
* Function Name :   ipspi1GetByte 
* Description   :   This function will read single byte from SPI bus. 
* Parameters    :   None 
* Return Value  :   contents of SPIBUF register                           
******************************************************************************/
byte ipspi1GetByte(void) {
    SPI_STATbits.SPIROV = 0;
    SPI_BUF = 0x00;     // initiate bus cycle
    while(SPI_STATbits.SPITBF);
    while(!SPI_STATbits.SPIRBF);
    return (SPI_BUF & 0xff);    // return byte read 
}

/******************************************************************************
* Function Name : ipspi1PutByte
* Description   : This routine writes a single byte to SPI bus.                                 
* Parameters    : Single data byte for SPI bus          
* Return Value  : contents of SPIBUF register
*******************************************************************************/
byte ipspi1PutByte(byte dout) {   
    byte c;
    SPI_BUF = dout;   // initiate SPI bus cycle by byte write 
    while(SPI_STATbits.SPITBF);
    while(!SPI_STATbits.SPIRBF);
    c = SPI_BUF;    // read out to avoid overflow 
    return c;
}

/******************************************************************************
* Function Name : ipspi1ChipSelect
* Description   : This routine toggles the chip select pin
* Parameters    : Single Boolean value indicating whether to select the chip
* Return Value  : None 
*******************************************************************************/
void ipspi1ChipSelect(byte select) {
    SPI_CS = select;
}
