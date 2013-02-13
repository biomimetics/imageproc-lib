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
 * Austria Microsystems AS5048B magnetic encoder I2C Interface
 *
 * by Duncan Haldane
 *
 * v.1.0
 *
 * Revisions:
 *  Duncan Haldane      2012-05-15    Initial release
 *  Andrew Pullin       2012-07-05    Ported to use i2c_driver module
 *   R. Fearing	2012-12-31  return fractional value, and put all encoder
                                          reading in this file, encoders 0-3 instead of 1-4 for simplicity.                 
 * Notes:
 *  - This module uses the I2C1 port for communicating with the AMS encoder chips
 */
#include "i2c_driver.h"
#include "i2c.h"
#include "ams-enc.h"
#include "utils.h"
#include "led.h"
 
#define LSB2ENCDEG 0.0219

#define ENC_I2C_CHAN        1 //Encoder is on I2C channel 1
 
unsigned int encAddr[8];	

EncObj encPos[NUM_ENC];

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static inline void encoderSetupPeripheral(void);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
/* ************
* Function Name : amsHallSetup
 * Description   : initialize I2C, and for initial calibration, set offset to current position
 * Parameters    : None
 * Return Value  : None
 ***************************/
void amsHallSetup()
{   int i;
      encSetup();
	// initialize structure
	for(i = 0; i< NUM_ENC; i++)
	{  encGetPos(i);	// get initial values w/o setting oticks
	// amsGetPos(i);
	   encPos[i].offset = encPos[i].pos; // initialize encoder
	   encPos[i].calibPos = 0;
   	   encPos[i].oticks = 0;   // set revolution counter to 0	
	}
}

void encSetup(void) {
    //setup I2C port I2C1
    encoderSetupPeripheral();
// LSB = R/W* . 
// 1. send slave <a2:a1>, a0=W (write reg address)
// 2. send slave register address <a7:a0>,  	
// 3. send slave <a2:a1>, a0=R (read reg data)
// 4. read slave data <a7:a0>
	encAddr[0] = 0b10000001;		//Encoder 0 rd;wr A1, A2 = low
	encAddr[1] = 0b10000000;		// write

    /***************Debugging */

    encAddr[0] = 0b10000011;        //Encoder 0 rd;wr A1, A2 = low
    encAddr[1] = 0b10000010;        // write
    
	encAddr[2] = 0b10000011;		//Encoder 1 rd;wr A2 = low, A1 = high
	encAddr[3] = 0b10000010;


	
	encAddr[4] = 0b10000101;		//Encoder 2 rd;wr A2 = high, A1 = low
	encAddr[5] = 0b10000100;

	encAddr[6] = 0b10000111;		//Encoder 3 rd;wr A2, A1 = high
	encAddr[7] = 0b10000110;

}

/*****************************************************************************
 * Function Name : encGetPos
 * Description   : Read the angular position of the encoder[num], write to struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void encGetPos(unsigned char num) {

    unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, encAddr[2*num+1]);	//Write address
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, encAddr[2*num]);		//Read address
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);

    encPos[num].pos = ((enc_data[1] << 6)+(enc_data[0] & 0x3F)); //concatenate registers
}

/*****************************************************************************
 * Function Name : amsGetPos/encSumPos
 * Description   : Count encoder[num] rollovers, write to struct encPos
 * Parameters    : None
 * Return Value  : None
 * .pos: 0 <= .pos <= 0x3fff,  0 to 2 pi range. 
  * see hall_velocity_ip2.5.ppt 
***************************************************************************/
void amsGetPos(unsigned char num) {
	unsigned int prev = encPos[num].pos;
	unsigned int update;
	encGetPos(num);
	update = encPos[num].pos;
	if (update > prev)
	{	if( (update-prev) > MAX_HALL/2 )	    	//Subtract one Rev count if diff > 180
		{	
            encPos[num].oticks--;
            LED_RED = ~LED_RED;

        }
	} 
	else
	{	if( (prev-update) > MAX_HALL/2 )		//Add one Rev count if -diff > 180
		{ 
            encPos[num].oticks++; 
            LED_BLUE = ~LED_BLUE;
        }
	}
}

/*****************************************************************************
 * Function Name : encGetFloatPos
 * Description   : Read the angular position of encoder[num] return as float
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
float encGetFloatPos(unsigned char num) {
    float pos;
    encGetPos(num);
    pos = encPos[num].pos* LSB2ENCDEG; //calculate Float
    return pos;
}

// range 0 rad = 0x0000, 2pi=0x3fff
// convert to 16 bits to be able to use signed long for leg angle 

/* void amsGetPos(unsigned char num)
{	int temp;
	encSumPos(num);
     	temp = encPos[num].pos- encPos[num].offset;
	if (temp < 0) temp = temp + MAX_HALL; // back in 0 , x < 2 pi range
     	encPos[num].calibPos = ((unsigned int)temp) << 2; // convert to 16 bits unsigned
}
*/

/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*****************************************************************************
 * Function Name : encoderSetupPeripheral
 * Description   : Setup I2C for encoders
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
static inline void encoderSetupPeripheral(void) { //same setup as ITG3200 for compatibility
    unsigned int I2C1CONvalue, I2C1BRGvalue;
    I2C1CONvalue = I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &
            I2C1_IPMI_DIS & I2C1_7BIT_ADD & I2C1_SLW_DIS &
            I2C1_SM_DIS & I2C1_GCALL_DIS & I2C1_STR_DIS &
            I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
            I2C1_STOP_DIS & I2C1_RESTART_DIS & I2C1_START_DIS;

    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 909KHz 	
    I2C1BRGvalue = 40;
    OpenI2C1(I2C1CONvalue, I2C1BRGvalue);
    IdleI2C1();
}
