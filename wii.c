/*
 * Copyright (c) 2009-2010, Regents of the University of California
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
 * Communication with Wiimote IR Detector (I2C)
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2009-08-10    Initial release
 *                      
 * Notes:
 *  - Uses an I2C port for communicating with an Wii IR camera (I2C1 for
 *    ImageProc1 and I2C2 for ImageProc2)
 */

#include "utils.h"
#include "i2c.h"
#include "wii.h"

#define WII_ADDR_RD             0xB1    
#define WII_ADDR_WR             0xB0    // 0x58 << 1
#define WII_DATA_WAIT           1000
#define WII_SETUP_DELAY	        10	// 10 is safe...
#define WII_READ_DELAY          1
#define WII_POSTREAD_DELAY      4

#define WII_DATA_WIDTH          16


#define wiiReadString(a,b,c) MastergetsI2C2(a,b,c)

/******************************************************************************
* Sensitivity Settings
* [0x02, 0x00, 0x00, 0x71, 0x01, 0x00, PO, 0x00, P1, P2, P3]
* P0 = MAX blob size (0x62 - 0xC8)
* P1 = Sensor GAIN, smaller = higher gain
* P2 = Sensor gain LIMIT, must be less than GAIN 
* P3 = MIN blob size, (3 - 5)
******************************************************************************/

const unsigned char WiiSensitivity[5][11] = {
    {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x72, 0x00, 0x20, 0x1F, 0x03},
    {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xC8, 0x00, 0x36, 0x35, 0x03},
    {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0xAA, 0x00, 0x64, 0x63, 0x03},
    {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x96, 0x00, 0xB4, 0xB3, 0x04},
    {0x02, 0x00, 0x00, 0x71, 0x01, 0x00, 0x96, 0x00, 0xFE, 0xFE, 0x05}};
    

/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/

static unsigned char wiiData[WII_DATA_WIDTH+1];
// The first byte of wiiData must be thrown away.

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

static void wiiWrite(unsigned char subaddr, unsigned char data);
static void wiiSendByte(unsigned char byte );
//static unsigned char wiiReceiveByte(void);
//static void wiiSendNack(void);
static void wiiStartTx(void);
static void wiiEndTx(void);
static void wiiSetupPeripheral(void);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void wiiSetupBasic(void) {

    wiiSetupPeripheral();

    wiiWrite(0x30, 0x01);
    delay_ms(WII_SETUP_DELAY);
    wiiWrite(0x30, 0x08);
    delay_ms(WII_SETUP_DELAY);
    wiiWrite(0x06, 0x90);
    delay_ms(WII_SETUP_DELAY);
    wiiWrite(0x08, 0xC0);
    delay_ms(WII_SETUP_DELAY);
    wiiWrite(0x1A, 0x40);
    delay_ms(WII_SETUP_DELAY);
    wiiWrite(0x33, 0x33);
    delay_ms(WII_SETUP_DELAY);

}

void wiiSetupAdvance(unsigned char sensitivity, unsigned char mode) {
//sensitivity: 1 = highest sensitivity, 5 = lowest

    if (sensitivity > 5 || sensitivity < 1) {
        wiiSetupBasic();
        return;
    }

    wiiSetupPeripheral();
    wiiWrite(0x30, 0x01);
    delay_ms(WII_SETUP_DELAY);
    
    wiiStartTx();
    wiiSendByte(WII_ADDR_WR);
    wiiSendByte(0x00);
    wiiSendByte(WiiSensitivity[sensitivity][0]);
    wiiSendByte(WiiSensitivity[sensitivity][1]);
    wiiSendByte(WiiSensitivity[sensitivity][2]);
    wiiSendByte(WiiSensitivity[sensitivity][3]);
    wiiSendByte(WiiSensitivity[sensitivity][4]);
    wiiSendByte(WiiSensitivity[sensitivity][5]);
    wiiSendByte(WiiSensitivity[sensitivity][6]);
    wiiEndTx();
    delay_ms(WII_SETUP_DELAY);

    wiiStartTx();
    wiiSendByte(WII_ADDR_WR);
    wiiSendByte(0x07);
    wiiSendByte(WiiSensitivity[sensitivity][7]);
    wiiSendByte(WiiSensitivity[sensitivity][8]);
    wiiEndTx();
    delay_ms(WII_SETUP_DELAY);

    wiiStartTx();
    wiiSendByte(WII_ADDR_WR);
    wiiSendByte(0x1A);
    wiiSendByte(WiiSensitivity[sensitivity][9]);
    wiiSendByte(WiiSensitivity[sensitivity][10]);
    wiiEndTx();
    delay_ms(WII_SETUP_DELAY);

    wiiWrite(0x33, mode);
    delay_ms(WII_SETUP_DELAY);

    wiiWrite(0x30, 0x08);
    delay_ms(WII_SETUP_DELAY);

}


void wiiGetData(WiiBlob* blobs) {

    wiiReadData();
    wiiConvertData(blobs);
}


void wiiConvertData(WiiBlob *blobs) {

    unsigned char* wiidata = wiiData + 1;
    unsigned int i;

    for (i = 0; i < 4; i++) {
        blobs[i].x = (((unsigned int)wiidata[i*3+2] & 0x0030) << 4) + wiidata[i*3];
        blobs[i].y = (((unsigned int)wiidata[i*3+2] & 0x00C0) << 2) + wiidata[i*3+1];
        blobs[i].size = wiidata[i*3+2] & 0x0F;

        if (blobs[i].x < 5 || blobs[i].x > 1018 || blobs[i].y < 5 || blobs[i].y > 763) {
            blobs[i].size = WII_INVALID_BLOB; // invalid blob
        }

    }
}


unsigned char* wiiToString(void) {
    return wiiData + 1; 
}

void wiiDumpData(unsigned char* buffer) {
    int i;
    for (i = 0; i < 12; i++) {
        buffer[i] = wiiData[i+1];
    }
}


unsigned char* wiiReadData(void) {
    wiiStartTx();
    wiiSendByte(WII_ADDR_WR);
    wiiSendByte(0x36);
    wiiEndTx();
    //delay_us(WII_READ_DELAY);       //this may not need.
    wiiStartTx();
    wiiSendByte(WII_ADDR_RD);
    wiiReadString(13, wiiData, WII_DATA_WAIT);
    wiiEndTx();   
    return wiiData + 1; 
}

// Just return the first nonzero blob.
// if there is no nonzero blob, return the previous one.
// Need to refine this function later.
// It would not work for more than two blobs at the same frame.
char wiiFindTarget(WiiBlob* blobs) {

    int i;
    static unsigned char valid_wii_index = 0;

    wiiConvertData(blobs);

    if (blobs[valid_wii_index].size != WII_INVALID_BLOB) {
        return valid_wii_index;
    } else {
        for (i = 0; i < 4; i++) {
            if (blobs[i].size != WII_INVALID_BLOB) {
                valid_wii_index = i;
                return i;
            }
        }
    }

    return -1;   // cound not find target.


    /* For flashing targets
    int i;
    static unsigned char patterns[4] = {0,0,0,0};
    unsigned char nibbles[2];
    unsigned char target_index = 4;

    wiiConvertData(blobs);

    for (i = 0; i < 4; ++i) {
        if (blobs[i].size == WII_INVALID_BLOB) {
            patterns[i] = patterns[i] << 1;     // pad 0 at the end
        } else { 
            patterns[i] = (patterns[i] << 1) + 1;     // pad 1 at the end
        }

        // nibbles should be the same for pattern detectioin except PATERN_1111
        nibbles[0] = patterns[i] & 0x0f;
        nibbles[1] = (patterns[i] >> 4) & 0x0f;

        if (nibbles[0] == 0x0f) {   // target found
            blobs[i].pattern = WII_PATTERN_1111;
            target_index = i;
        } else if (nibbles[0] != nibbles[1] || nibbles[0] == 0x00) {       // no pattern found yet
            blobs[i].pattern = WII_PATTERN_0000;
        } else if (nibbles[0] == 0x05 || nibbles[0] == 0x0a ) {
            blobs[i].pattern = WII_PATTERN_0101;
        } else if (nibbles[0] == 0x01 || nibbles[0] == 0x02 || nibbles[0] == 0x04 || nibbles[0] == 0x08) {
            blobs[i].pattern = WII_PATTERN_0001;
        } else if (nibbles[0] == 0x03 || nibbles[0] == 0x06 || nibbles[0] == 0x09 || nibbles[0] == 0x0c ) {
            blobs[i].pattern = WII_PATTERN_0011;
        } else {    // 0x07, 0x0b, 0x0d, 0x0e
            blobs[i].pattern = WII_PATTERN_0111;
        }

    }

    return target_index;

    */

}






/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/


/*****************************************************************************
* Function Name : wiiWrite
* Description   : Write a data to a register
* Parameters    : regaddr - address of register
*                 data - value to be written to the register
* Return Value  : None
*****************************************************************************/
static void wiiWrite( unsigned char subaddr, unsigned char data ){
    wiiStartTx();
    wiiSendByte(WII_ADDR_WR);
    wiiSendByte(subaddr);
    wiiSendByte(data);
    wiiEndTx();
}

/*****************************************************************************
* Function Name : wiiSendByte
* Description   : Send a byte to wii camera
* Parameters    : byte - a byte to send
* Return Value  : None
*****************************************************************************/
static void wiiSendByte( unsigned char byte ){
    MasterWriteI2C2(byte);
    while(I2C2STATbits.TRSTAT);
    while(I2C2STATbits.ACKSTAT);
}

/*****************************************************************************
* Function Name : wiiReceiveByte
* Description   : Receive a byte from wii camera
* Parameters    : None
* Return Value  : None
*****************************************************************************/
/*
static unsigned char wiiReceiveByte(void){
    return MasterReadI2C2();
}
*/

/*****************************************************************************
* Function Name : wiiSendNack
* Description   : Send NACK to wii camera
* Parameters    : None
* Return Value  : None
*****************************************************************************/
/*
static void wiiSendNack (void){
    NotAckI2C2();
    while(I2C2CONbits.ACKEN);
}
*/

/*****************************************************************************
* Function Name : wiiStartTx
* Description   : Start I2C transmission
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void wiiStartTx(void){
    StartI2C2();
    while(I2C2CONbits.SEN);
}

/*****************************************************************************
* Function Name : wiiEndTx
* Description   : End I2C transmission
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void wiiEndTx (void){
    StopI2C2();
    while(I2C2CONbits.PEN);
}

/*****************************************************************************
* Function Name : wiiSetupPeripheral
* Description   : Setup I2C for wiimote
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void wiiSetupPeripheral(void) {
    unsigned int I2C2CONvalue, I2C2BRGvalue;
    I2C2CONvalue = I2C2_ON & I2C2_IDLE_CON & I2C2_CLK_HLD &
                   I2C2_IPMI_DIS & I2C2_7BIT_ADD & I2C2_SLW_DIS &
                   I2C2_SM_DIS & I2C2_GCALL_DIS & I2C2_STR_DIS &
                   I2C2_NACK & I2C2_ACK_DIS & I2C2_RCV_DIS &
                   I2C2_STOP_DIS & I2C2_RESTART_DIS & I2C2_START_DIS;

    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 400KHz 	
    I2C2BRGvalue = 95; 
    OpenI2C2(I2C2CONvalue, I2C2BRGvalue);
    IdleI2C2();
}



