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
 * ADC + DMA module for motor BEMF
 *
 * by apullin
 *
 * v.1.0
 *
 * Revisions:
 *  Andrew Pullin        2012-10-23      Initial release
 *
 * Notes:
 *   >>>> THIS IS IMAGEPROC2.5 SPECIFIC CODE <<<<
 *  - This module configured ADC1 to scan through AN0, AN8, AN9, AN10, and AN11
 *      sequentially, to read the battery and 4 motor BEMF's.
 *  - DMA is used.
 *  - Values are stored locally, getter functions are provided.
 */

#include "adc.h"
#include "adc_pid.h"
#include "p33Fxxxx.h"
#include "ports.h"
#include "utils.h"


//Functions
static void adcSetupPeripheral(void);
//DMA related functions
static void initDma0(void);
void __attribute__((__interrupt__)) _DMA0Interrupt(void);


//Variables to store values as they come out of the DMA buffer
static unsigned int adc_MotorA, adc_MotorB, adc_MotorC, adc_MotorD; // motors
static unsigned int adc_AN8, adc_AN9, adc_AN10, adc_AN11;           //also motors
static unsigned int adc_AN0, adc_Vbatt; //battery

void adcSetup(void){
	adcSetupPeripheral();
	initDma0(); //DMA is needed to read multiple values from the ADC core
}

static void adcSetupPeripheral(void) {
    unsigned int AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value,
            AD1PCFGHvalue, AD1PCFGLvalue, AD1CSSHvalue, AD1CSSLvalue,
            AD1CHS0value, AD1CHS123value;

    AD1CON1value = ADC_MODULE_ON & //ADC module is enabled
            ADC_IDLE_CONTINUE & // ADC will continue in idle mode
            ADC_AD12B_10BIT & // ADC in 10 bit mode
            ADC_FORMAT_INTG & // ADC in integer format (CLARIFY)
            ADC_CLK_MPWM & // MCPWM interval ends sampling and starts conversion
            ADC_MULTIPLE & //Sequential sampling
            ADC_ADDMABM_ORDER & //DMA buffers are written in the order of conversion
            ADC_AUTO_SAMPLING_ON & //ADC does not need to be triggered manually
            ADC_SAMP_ON; //sample / hold amplifiers are sampling  (maybe incorrect)
    AD1CON2value = ADC_VREF_AVDD_AVSS & //Vref+ = AVdd , Vref- = AVss
            ADC_SCAN_ON & //Scan through ADC channels
            ADC_SELECT_CHAN_0 & //Only CH0, will scan
            ADC_ALT_BUF_OFF & //Use one 16 word buffer
            ADC_ALT_INPUT_OFF & // Alternate between MUXA and MUXB
            ADC_DMA_ADD_INC_4; //Increment DMA address after each sample
    AD1CON3value = ADC_CONV_CLK_SYSTEM & //Use System clock, not internal RC osc
            ADC_CONV_CLK_3Tcy & //Tad = 3 * Tcy TODO: Check this
            ADC_SAMPLE_TIME_1; //Sample Time = 1*Tad
    AD1CON4value = ADC_DMA_BUF_LOC_1; //This may be wrong (TODO)


    AD1CHS123value = 0; //Dummy value, CH 1,2,3 unused

    AD1CHS0value = ADC_CH0_NEG_SAMPLEA_VREFN & // Sample A, Vref- = AVss = ground
            ADC_CH0_POS_SAMPLEA_AN11; // Motor D


    AD1CSSHvalue = SCAN_NONE_16_31; //Skip AN16-AN131 for Input Scan, not avail on dsPic33
    //Scan: AN0, AN8, AN9, AN10, AN11
    AD1CSSLvalue = SCAN_NONE_0_15 | (1 << 0) //AN0
                                  | (1 << 8) //AN8
                                  | (1 << 9) //AN9
                                  | (1 << 10) //AN10
                                  | (1 << 11); //AN11

    //Set pins to analog inputs; also check init_default.c
    AD1PCFGHvalue = ENABLE_ALL_DIG_16_31; //Shouldn't matter, only AN0-15 on 706A
    AD1PCFGLvalue = ENABLE_AN0_ANA & //Battery
            ENABLE_AN8_ANA & //Motor A
            ENABLE_AN9_ANA & //Motor B
            ENABLE_AN10_ANA & //Motor C
            ENABLE_AN11_ANA; //Motor D

    SetChanADC1(AD1CHS123value, AD1CHS0value);
    OpenADC1(AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value, 
            AD1PCFGLvalue, AD1PCFGHvalue, AD1CSSHvalue, AD1CSSLvalue);

    //The following is a "patch" to the above settings
    AD1CON1bits.AD12B = 0; //10bit mode
    AD1CON2bits.SMPI = 5-1;  //every 5th sample
    AD1CON2bits.CHPS = 0;
    AD1CON1bits.SIMSAM = 0; //overridden
    AD1CON1bits.ADDMABM = 0;
    AD1CON4bits.DMABL = 0; //1 word buffer each
    AD1CON2bits.ALTS = 0;
    //AD1CHS0bits.CH0NA = 0;
    //AD1CHS0bits.CH0SA = 0; //overridden
    AD1CON2bits.VCFG = 000; //Avdd and Avss
    AD1CON2bits.CSCNA = 1;
    AD1CSSL = 0; //clear it first
    AD1CSSLbits.CSS0 = 1;
    AD1CSSLbits.CSS8 = 1;
    AD1CSSLbits.CSS9 = 1;
    AD1CSSLbits.CSS10 = 1;
    AD1CSSLbits.CSS11 = 1;




    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; //Disable A/D interrupt
}


//For testing purposes, should not be enabled
/*
void __attribute__((interrupt,no_auto_psv)) _ADC1Interrupt(void)
{
	//ADC sync indicator
	if(AD1CON1bits.DONE){
		LATB |= (1<<4);
	}else{
		LATB &= ~(1<<4);
	}
	IFS0bits.AD1IF = 0;
}
*/


//Getters for other modules to access values, by AN pin name
unsigned int adcGetAN0(){
	return adc_AN0;
}

unsigned int adcGetAN8(){
	return adc_AN8;
}

unsigned int adcGetAN9(){
	return adc_AN9;
}

unsigned int adcGetAN10(){
	return adc_AN10;
}

unsigned int adcGetAN11(){
	return adc_AN11;
}

//Getters for other modules to access values
unsigned int adcGetVbatt(){
	return adc_Vbatt;
}

unsigned int adcGetMotorA(){
	return adc_MotorA;
}

unsigned int adcGetMotorB(){
	return adc_MotorB;
}

unsigned int adcGetMotorC(){
	return adc_MotorC;
}

unsigned int adcGetMotorD(){
	return adc_MotorD;
}

//////////////////////////////////////////////////////////////////////
///////////////      DMA Section     /////////////////////////////////
//////////////////////////////////////////////////////////////////////

#define  SAMP_BUFF_SIZE	 		1		// Size of the input buffer per analog input

//Buffers need special attribute to be in DMA memory space
static int  BufferA[5][SAMP_BUFF_SIZE] __attribute__((space(dma)));
static int  BufferB[5][SAMP_BUFF_SIZE] __attribute__((space(dma)));

static unsigned int DmaBuffer = 0;


/*****************************************************************************
 * Function Name : initDma0
 * Description   : Setup function for DMA0, to read ADC1 into a buffer
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
static void initDma0(void) {
    DMA0CONbits.AMODE = 0; // Configure DMA for Register Indirect w/ post-increment
    DMA0CONbits.MODE = 2; // Configure DMA for Continuous Ping-Pong mode

    DMA0PAD = (int) &ADC1BUF0;
    //DMA0CNT = (SAMP_BUFF_SIZE*2)-1;
    DMA0CNT = 4; //See dsPIC user's manual. 5 analog reads -> DMA0CNT = 5-1 = 4

    DMA0REQ = 13; //ADC1 requests

    DMA0STA = __builtin_dmaoffset(BufferA);
    DMA0STB = __builtin_dmaoffset(BufferB);

    IFS0bits.DMA0IF = 0; //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1; //Set the DMA interrupt enable bit

    DMA0CONbits.CHEN = 1;
}
/*****************************************************************************
 * Function Name : _DMA0Interrupt
 * Description   : Interrupt hander for DMA0 , associated with ADC1 here.
                                  Motor BEMF vales are set through setter functions.
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void) {
    //This should really be done in an elegant way by selecting a pointer, and
    // not repeating code below.
    LED_3 = 1;
    if (DmaBuffer == 0) {
        adc_AN0 = BufferA[0][0];  //AN0
        adc_AN8 = BufferA[1][0];  //AN8
        adc_AN9 = BufferA[2][0];  //AN9
        adc_AN10 = BufferA[3][0]; //AN10
        adc_AN11 = BufferA[4][0]; //AN11
    } else {
        adc_AN0 = BufferB[0][0];  //AN0
        adc_AN8 = BufferB[1][0];  //AN8
        adc_AN9 = BufferB[2][0];  //AN9
        adc_AN10 = BufferB[3][0]; //AN10
        adc_AN11 = BufferB[4][0]; //AN11
    }

    //Update named variables
    adc_Vbatt  = adc_AN0;
    adc_MotorA = adc_AN8;
    adc_MotorB = adc_AN9;
    adc_MotorC = adc_AN10;
    adc_MotorD = adc_AN11;

    DmaBuffer ^= 1; //Toggle between buffers
    LED_3 = 0;
    IFS0bits.DMA0IF = 0; //Clear the DMA0 Interrupt Flag
}
// End DMA section
