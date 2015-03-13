/***********************************************************************
 * This file provides a basic template for writing dsPIC30F trap       *
 * handlers in C language for the C30 compiler                         *
 *                                                                     *
 * Add this file into your MPLAB project. Build your project, program  *
 * the device and run. If any trap occurs during code execution, the   *
 * processor vectors to one of these routines.                         *
 *                                                                     *
 * For additional information about dsPIC architecture and language    *
 * tools, refer to the following documents:                            *
 *                                                                     *
 * MPLAB C30 Compiler User's Guide                        : DS51284    *
 * dsPIC 30F MPLAB ASM30, MPLAB LINK30 and Utilites                    *
 *                                           User's Guide : DS51317    *
 * Getting Started with dsPIC DSC Language Tools          : DS51316    *
 * dsPIC 30F Language Tools Quick Reference Card          : DS51322    *
 * dsPIC 30F 16-bit MCU Family Reference Manual           : DS70046    *
 * dsPIC 30F General Purpose and Sensor Families                       *
 *                                           Data Sheet   : DS70083    *
 * dsPIC 30F/33F Programmer's Reference Manual            : DS70157    *
 *                                                                     *
 * Template file has been compiled with MPLAB C30 v3.00.               *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 *    Author:                                                          *
 *    Company:                                                         *
 *    Filename:       traps.c                                          *
 *    Date:           04/11/2007                                       *
 *    File Version:   3.00                                             *
 *    Devices Supported:  All PIC24F,PIC24H,dsPIC30F,dsPIC33F devices  *
 *                                                                     *
 **********************************************************************/
/*
 Updates:
 * Andrew Pullin    01/28/15    Added _where_was_i precall to all traps
 *                              Removed old-style includes, added <xc>
 */

#include<xc.h>

//Plain trap attribute
#define _trapISR           __attribute__((interrupt,no_auto_psv))
//Augmented trap attr, which calls assembly func _where_was_i first
#define _trapISR_WhereWasI __attribute__((interrupt(preprologue("rcall _where_was_i")),no_auto_psv))

//imageproc-lib includes
#include "utils.h"  //For LED control

//extern is acceptable here because it is referencing an assembly file
//extern unsigned long __errAddress;
//TODO: retained here until __errAddress can be tested in debug context; AP

/* ****************************************************************
 * Standard Exception Vector handlers if ALTIVT (INTCON2<15>) = 0  *
 *                                                                 *
 * Not required for labs but good to always include                *
 ******************************************************************/
void _trapISR_WhereWasI _OscillatorFail(void) {

    INTCON1bits.OSCFAIL = 0;
    while (1);
}

 void _trapISR_WhereWasI _AddressError(void) {

    INTCON1bits.ADDRERR = 0; //Clear the trap flags
    while (1) {
        //asm volatile("btg   PORTF, #1");
        LED_1 ^= 1;
        delay_ms(100);
        LED_2 ^= 1;
        delay_ms(100);
        LED_3 ^= 1;
        delay_ms(100);
        //for (k=0; k<100; k++) { delay_ms(5); }   // Waste approximatelly 50ms
    };

    while (1);
}

void _trapISR_WhereWasI _StackError(void) {

    INTCON1bits.STKERR = 0;
    while (1);
}

void _trapISR_WhereWasI _MathError(void) {

    INTCON1bits.MATHERR = 0;
    while (1);
}

/* ****************************************************************
 * Alternate Exception Vector handlers if ALTIVT (INTCON2<15>) = 1 *
 *                                                                 *
 * Not required for labs but good to always include                *
 ******************************************************************/
void _trapISR_WhereWasI _AltOscillatorFail(void) {

    INTCON1bits.OSCFAIL = 0;
    while (1);
}

void _trapISR_WhereWasI _AltAddressError(void) {

    INTCON1bits.ADDRERR = 0;
    while (1);
}

void _trapISR_WhereWasI _AltStackError(void) {

    INTCON1bits.STKERR = 0;
    while (1);
}

void _trapISR_WhereWasI _AltMathError(void) {

    INTCON1bits.MATHERR = 0;
    while (1);
}


/*  ; where_was_i assembly helper code
    ;
    ; This code is called by the Address Error Trap ISR.  It will save the address
    ; of the instruction that caused an illegal address reference trap.  Place
    ; _errAddress in your Watch window to see what happend.
    ;
    ; GLOBAL: _errAddress
    ;
 */

asm("; GLOBAL: _errAddress\n\t"
    ";\n\t"
    "  .section *,bss,near\n\t"
    "  .global __errAddress\n\t"
    "__errAddress:   .space 4\n\t"
    "  .text\n\t"
    "  .global _where_was_i\n\t"
    "_where_was_i:\n\t"
    "  push.d w0\n\t"
    "  sub w15,#12,w1           ; twelve bytes pushed since last trap!\n\t"
    "  mov [w1++], w0\n\t"
    "  mov w0, __errAddress\n\t"
    "  mov [w1], w0\n\t"
    "  mov.b WREG, __errAddress+2\n\t"
    "  pop.d w0\n\t"
    "  return\n\t");
