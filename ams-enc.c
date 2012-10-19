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
 *                      
 * Notes:
 *  - This module uses the I2C1 port for communicating with the AMS encoder chips
 */
#include "i2c_driver.h"
#include "i2c.h"
#include "ams-enc.h"
#include "utils.h"

#define ENC_ADDR_R_RD 0b10000011		//A1 on AS5048B Pulled High, A2 Low
#define ENC_ADDR_R_WR 0b10000010

#define ENC_ADDR_L_RD 0b10000101		//A2 on AS5048B Pulled High, A1 Low
#define ENC_ADDR_L_WR 0b10000100

#define ENC_ADDR_AUX1_RD 0b10000001		//A1, A2 = low
#define ENC_ADDR_AUX1_WR 0b10000000

#define LSB2ENCDEG 0.0219

#define ENC_I2C_CHAN        1 //Encoder is on I2C channel 1

#define ABS(v) v * ((v>0) - (v<0))

typedef struct {
    unsigned short RPOS; //Leg position struct
	long rticks;
    unsigned short LPOS;
	long lticks;
} ENCPOS;

ENCPOS encPos;

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static inline void encoderSetupPeripheral(void);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void encSetup(void) {
    //setup I2C port I2C1
    encoderSetupPeripheral();

}

/*****************************************************************************
 * Function Name : encGetRPos
 * Description   : Read the angular position of the right encoder, write to struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void encGetRPos(void) {

    unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, ENC_ADDR_R_WR);
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, ENC_ADDR_R_RD);
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);

    encPos.RPOS = ((enc_data[1] << 6)+(enc_data[0] & 0x3F)); //concatenate registers
}

void encSumRPos(void) {

	int prev = encPos.RPOS;
	int update;
	encGetRPos();
	update = encPos.RPOS;
	
	if( (update-prev) > 8192 ){		    	//Subtract one Rev count if diff > 180
		encPos.rticks--;
	}
	
		if( (prev-update) > 8192 ){			//Add one Rev count if -diff > 180
		encPos.rticks++;
	}
		
}
	

/*****************************************************************************
 * Function Name : encGetLPos
 * Description   : Read the angular position of the left encoder, write to struct encPos
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
void encGetLPos(void) {

    unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, ENC_ADDR_L_WR);
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);
    
    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, ENC_ADDR_L_RD);
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);

    encPos.LPOS = ((enc_data[1] << 6)+(enc_data[0] & 0x3F)); //concatenate registers

    return;
}

/*****************************************************************************
 * Function Name : encGetAux1Pos
 * Description   : Read the angular position of the 1st auxiliary encoder
 * Parameters    : None
 * Return Value  : None
 *****************************************************************************/
float encGetAux1Pos(void) {

    //unsigned char apos;
    float apos;

     unsigned char enc_data[2];

    i2cStartTx(ENC_I2C_CHAN); //Setup to burst read both registers, 0xFE and 0xFF
    i2cSendByte(ENC_I2C_CHAN, ENC_ADDR_AUX1_WR);
    i2cSendByte(ENC_I2C_CHAN, 0xFE);
    i2cEndTx(ENC_I2C_CHAN);

    i2cStartTx(ENC_I2C_CHAN);
    i2cSendByte(ENC_I2C_CHAN, ENC_ADDR_AUX1_RD);
    i2cReadString(1,2,enc_data,10000);
    i2cEndTx(ENC_I2C_CHAN);

    apos = ((enc_data[1] << 6)+(enc_data[0] & 0x3F)) * LSB2ENCDEG; //concatenate registers

    return apos;
}

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
    I2C1BRGvalue = 95;
    OpenI2C1(I2C1CONvalue, I2C1BRGvalue);
    IdleI2C1();
}
