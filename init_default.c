/*
 * Copyright (c) 2010-2012, Regents of the University of California
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
 * Default Initialization Routines
 *
 * by Fernando L. Garcia Bermudez & Stanley S. Baek
 *
 * v.1.0
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez     2010-6-21   Initial release
 *  Stanley S. Baek                 2010-6-25   Expanded to even more boards
 *
 */

#include <xc.h>
#include "adc.h"

/* Configuration Bits (macros defined in processor header) */
#if !defined(__BOOTLOAD)

    #if defined(__EXP16DEV)
        // Primary OSC (XT, HS, EC) w/ PLL
        _FOSCSEL(FNOSC_PRIPLL);

        // XT oscillator & CLK Switch./Mon. Dis & OSC2 as CLK Out
        _FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT);

        // Watchdog Timer Disabled
        _FWDT(FWDTEN_OFF);

    #else
        // Primary OSC (XT, HS, EC) w/PLL & 2-Speed Startup Enabled (for fast EC)
        _FOSCSEL(FNOSC_PRIPLL & IESO_ON);

        // EC oscillator & CLK Switch./Mon. Dis & OSC2 as GPIO
        _FOSC(POSCMD_EC & FCKSM_CSDCMD & OSCIOFNC_ON);

        // Watchdog Timer Disabled
        _FWDT(FWDTEN_OFF);


    #endif
    
    #if defined(__MIKRO)
        _FICD(ICS_PGD3 & // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
              JTAGEN_OFF           // JTAG Port Enable (JTAG is Disabled)
                );
    #else
        _FICD(ICS_PGD1 & // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
              JTAGEN_OFF           // JTAG Port Enable (JTAG is Disabled)
                );
    #endif
#endif // !defined(__BOOTLOAD)


void SetupClock(void)
{
#if !defined(__BOOTLOAD)

    #if defined(__EXP16DEV)
        // Setup for 40MIPS(Fcy) w/8MHz XT(Fin): Fcy = Fin * (M/(2 * N1 * N2))
        // Configure Oscillator to operate the device at 40Mhz
        // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
        // Fosc= 8M*40(2*2)=80Mhz for 8M input clock
        PLLFBD=38; // M=40
        CLKDIVbits.PLLPOST=0; // N1=2
        CLKDIVbits.PLLPRE=0; // N2=2
        OSCTUN=0; // Tune FRC oscillator, if FRC is used
        RCONbits.SWDTEN=0; // Disable Watch Dog Timer
    #else
        // Setup for 40MIPS(Fcy) w/40MHz XT(Fin): Fcy = Fin * (M/(2 * N1 * N2))
        _PLLDIV = 6;  // M = 8
        _PLLPRE = 0;  // N1 = 2
        _PLLPOST = 0; // N2 = 2
    #endif //defined(__EXP16DEV)

#endif //!defined(__BOOTLOAD)
}


void SwitchClocks(void)
{
#if !defined(__BOOTLOAD)

    //Wait for PLL to lock
    while(OSCCONbits.LOCK!=1);

#endif //!defined(__BOOTLOAD)
}


void SetupPorts(void)
{
    // Disabling all potential ADC AIO's to enable DIO's
    // (will enable the needed ones in SetupADC())
    AD1PCFGL = ENABLE_ALL_DIG_0_15;
    AD2PCFGL = ENABLE_ALL_DIG_0_15;

#if defined(__IMAGEPROC1)

    // LEDs: RB3(AN3), RD11(ext int), and RF0-1 are outputs
    LATB  = 0x0000;
    TRISB = 0b1111111111110111; // A/D Conv: RB0-2(A/D Conv) are analog inputs
    LATD  = 0x000;
    TRISD = 0b011111111111; // Data: RD0-7(PIXEL) and Sync: RD8-10(VSYNC/HREF/
                            // PCLK) remain inputs for the camera
    LATF  = 0x00;
    TRISF = 0b1111100; // Switches: RF2-3, Batt Supervisor: RF6(ext int) are
                       // all inputs

    // Camera PWDN: RC13-14 are outputs
    LATC  = 0x0000;
    TRISC = 0b1001111111111111;

    // DFMEM: SPI2 Slave Select is an output (RG9)
    LATG  = 0b0000000000;
    TRISG = 0b0111111111;

    // PWM: RE2 is an output managed thru the peripheral
    // PWM outputs (*not* hardware PWM)
    // All 8 pins (RE0 - RE7) are outputs
    LATE  = 0b00000000;
    TRISE = 0b00000000;


#elif defined(__IMAGEPROC24) || defined(__IMAGEPROC25)

    // LEDs: RB12-14 are outputs
    // SPI1 Slave Select is an output (RB2)
    // SLPTR for Radio is an output (RB15)
    // A/D Conv: RB-1, RB5, and RB8 are analog inputs
    LATB  = 0x0000;
    TRISB = 0b0000111111111011;

    // Camera PWDN: RC14 is an output; SPI2 RC15 is also output.
    LATC  = 0b1000000000000000;
    TRISC = 0b0011111111111111;

    // OVCAM: RD0-7(PIXEL), RC13(VSYNC), RF0(HREF), and RF1(PCLK) are inputs
    // RD8-RD11 are used for external interrupt
    // Batt Supervisor: RD9(ext int) is an input
    LATD = 0x0000;
    TRISD = 0xffff;

    // DFMEM: SPI2 Slave Select is an output (RG9)
    LATG  = 0b1000000000;
    TRISG = 0b0111111111;

    // PWMs: RE0, RE2, RE4, and RE6 are outputs managed thru the peripheral
    // PWM outputs (*not* hardware PWM)
    // All 8 pins (RE0 - RE7) are outputs
    LATE  = 0b00000000;
    TRISE = 0b00000000;


#elif defined(__BASESTATION) || defined(__BASESTATION2)

    // RD0-RD3 are used for LEDs
    // RD11 is input (Radio Interrupt)
    LATD = 0x0000;
    TRISD = 0b0000100000000000;

    // RE5 is output (Radio SLP_TR)
    LATE = 0x0000;
    TRISE = 0x0000;

    // RADIO: SPI2 Slave Select is an output (RG9)
    LATG  = 0b0000000000;
    TRISG = 0b0111111111;

#elif defined(__MIKRO)

    // LCD: RB0-RB7 are outputs
    // DEBUG RB8 - RB15 are outputs
    LATB  = 0x0000;
    TRISB = (1 << 0) | (1<<8) | (1 << 9) | (1<<10) | (1<<11);; //rb0, rb8-rb11 are inputs for ADC
    //TRISB = 0x0000;

    // RD0-RD7 are used for camera input data
    // RD8-RD11 are used for external interrupt
    LATD = 0x0000;
    TRISD = 0xffff;

    // SLPTR for Radio (RF0)
    LATF = 0x0000;
    TRISF = 0xfffe;

    // Radio: SPI2 Slave Select is an output (RG9)
    LATG  = 0b0000000000;
    TRISG = 0b0111111111;

    // PWM outputs (*not* hardware PWM)
    // All 8 pins (RE0 - RE7) are outputs
    LATE  = 0b00000000;
    TRISE = 0b00000000;


#elif defined(__EXP16DEV)

    // LEDs (D3-D10/RA0-RA7)
    LATA = 0x0000;
    TRISA = 0xFF00;

    // LCD control pins RS (RB15)
    // SLPTR for Radio (RB1)
    LATB = 0x0000;
    TRISB = 0b0111111111111101;

    // LCD control pins RW(RD5), EN(RD4)
    LATD = 0x0000;
    TRISD = 0xffcf;

    // LCD module: RE0-RE7 are outputs
    LATE  = 0x0000;
    TRISE = 0xff00;

    // Radio: SPI2 Slave Select is an output (RG9)
    LATG  = 0b0000000000;
    TRISG = 0b0111111111;

#endif
}
