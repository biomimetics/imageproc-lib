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
 * Atmel AT86RF231 IEEE 802.15.4 wireless radio driver
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanlay S. Baek     2010-6-2    Initial release
 *
 * Notes:
 *  - TODO (stanbaek) : Polling may need to be implemented in macReceivePacket
 *  - TODO (stanbaek) : The rx_frame_ buffer can be overwritten before user
 *                      reads the buffer. Work-around: Need to modify ISR
 *                      and/or increase number of buffers.
 */

#include "at86rf.h"
#include "ports.h"      // for external interrupt
#include "at86rf231.h"
#include "generic_typedefs.h"
#include "payload.h"
#include "packet_queue.h"
#include "mac_packet.h"
#include "at86rf.h"
#include <stdlib.h>
#include <stdio.h>
//#include "lcd.h"
#include "ipspi1.h"
#include "radio.h"
#include "utils.h"

void phyReadId(unsigned char *id);
void phyHandleISR(void);
void phySetSlptr(char	val);
byte phyReadReg(byte addr);
void phyWriteReg(byte addr, byte val);
byte phyReadBit(byte addr, byte mask, byte pos);
void phyWriteBit(byte addr, byte mask, byte pos, byte val);


#define TRX_CMD_RW          (0xC0) // Register Write
#define TRX_CMD_RR          (0x80) // Register Read 
#define TRX_CMD_FW          (0x60) // Frame Transmit Mode
#define TRX_CMD_FR          (0x20) // Frame Receive Mode
#define TRX_CMD_SW          (0x40) // SRAM Write.
#define TRX_CMD_SR          (0x00) // SRAM Read.

#define MAX_FRAME_LEN       (127)

#if defined(__MIKRO)
    #define SLPTR           _LATF0      //_LATB15
#elif defined(__EXP16DEV)
    #define SLPTR           _LATB1
#elif defined(__BASESTATION)
    #define SLPTR           _LATE5
#else
    #define SLPTR           _LATB15
#endif

typedef enum {
    STATE_SLEEP = 0,
    STATE_TRX_OFF,
    STATE_TX_ON,
    STATE_RX_ON,
    STATE_RX_AACK_ON,
    STATE_TX_ARET_ON,
    STATE_BUSY_TX_ARET,
} RadioState;

volatile char g_current_state;
volatile char g_last_ackd;
volatile byte rx_num_bytes_;   // number of characters in the buffer
byte radio_lqi = 0xFF;

WordVal g_dest_pan_id;
WordVal g_dest_addr;
WordVal _PERSISTENT g_src_addr;
WordVal _PERSISTENT g_src_pan_id;
char    _PERSISTENT g_channel;

static PacketQueue rx_p_queue;
static PacketQueue tx_p_queue;
#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START	char saved_ipl; SET_AND_SAVE_CPU_IPL(saved_ipl, 7);
#define CRITICAL_SECTION_END RESTORE_CPU_IPL(saved_ipl);
#endif

unsigned char id[4];


/******************************************************************************
* Function Name : atInit  
* Description   : Initializes and configures the at86rf31 chip, sets the
*                 source address, and the source PAN ID, and configures the 
*                 interrupt for handling received packets.
* Parameters    : None
* Return Value  : None
*******************************************************************************/

void atInit(WordVal src_addr, WordVal pan_id) {

	char rndval;

    ipspi1Config();

    ipspi1ChipSelect(TRUE);     // set chip-select

	phyReadId(id);
	Nop();

    // transition to trx_off
    phyWriteReg(RG_TRX_STATE, CMD_FORCE_TRX_OFF); 

    // interrupt at the end of frame send/receive
    phyWriteReg(RG_IRQ_MASK, TRX_IRQ_TRX_END); 

    // automatic CRC generation fro tx operation
    phyWriteBit(SR_TX_AUTO_CRC_ON, 1); 

    // no clock on clkm pin
    phyWriteBit(SR_CLKM_CTRL, CLKM_NO_CLOCK); 

    //clear any pending iterrupt
    phyReadReg(RG_IRQ_STATUS);
    
    //set address
    g_src_addr = src_addr;
    phyWriteReg(RG_SHORT_ADDR_0, src_addr.byte.LB);
    phyWriteReg(RG_SHORT_ADDR_1, src_addr.byte.HB);

    //set pan id
    g_src_pan_id = pan_id;
    phyWriteReg(RG_PAN_ID_0, pan_id.byte.LB);
    phyWriteReg(RG_PAN_ID_1, pan_id.byte.HB);

    //transition to rx_on
    phyWriteReg(RG_TRX_STATE, CMD_RX_AACK_ON);

	//Set Retry value for Radio
	#if defined(__BASESTATION)
    atSetFrameRetries(0);
    atSetCSMARetries(0);
	#elif defined(__IMAGEPROC2)
	atSetFrameRetries(2);
    atSetCSMARetries(2);
    #endif

	//Construct 8bit random seed from 2-bit random generator inside RF231 chip
	rndval = phyReadBit(SR_RND_VALUE) << 6; delay_ms(1);
	rndval |= phyReadBit(SR_RND_VALUE) << 4; delay_ms(1);
	rndval |= phyReadBit(SR_RND_VALUE) << 2; delay_ms(1);
	rndval |= phyReadBit(SR_RND_VALUE); delay_ms(1);
	phyWriteBit(SR_CSMA_SEED_0, rndval);	
	
    rndval = phyReadBit(SR_RND_VALUE) << 2; delay_ms(1);
	rndval |= phyReadBit(SR_RND_VALUE); delay_ms(1);
	phyWriteBit(SR_CSMA_SEED_1, rndval);
    
	//phyWriteBit(SR_AACK_FVN_MODE, FRAME_VERSION_0);
    //phyWriteBit(SR_AACK_SET_PD, PD_ACK_BIT_SET_ENABLE);
    phyWriteBit(SR_AACK_DIS_ACK, ACK_ENABLE);
    phyWriteBit(SR_MAX_BE, 3);
    phyWriteBit(SR_MIN_BE, 1);
    phyWriteBit(SR_CCA_MODE, CCA_MODE_2); 

	//Set to +3.0 dBm TX power
	//phyWriteBit(SR_TX_PWR, 0x0);

	//High data rate setup
	#if defined(__RADIO_HIGH_DATA_RATE)
		//phyWriteBit(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
		phyWriteBit(SR_OQPSK_DATA_RATE, ALTRATE_500KBPS);
		phyWriteBit(SR_RX_PDT_LEVEL, 0x0);
	#endif

    g_current_state = STATE_RX_AACK_ON;
    rx_num_bytes_ = 0;
	//
    g_last_ackd = 0;

    ConfigINT4(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_5); // Radio    
}

char atSetChannel(char chan){
    if (CHAN_MIN <= chan && chan <= CHAN_MAX) {
        phyWriteBit(SR_CHANNEL, chan);
        g_channel = chan;
        return 1;
    } else {
        return 0;
    }
}

char atGetChannel(void){

    return phyReadBit(SR_CHANNEL);
    
}

void atSetPromMode(char on){
    if (on){
        phyWriteBit(SR_AACK_PROM_MODE, 1);
        phyWriteBit(SR_AACK_DIS_ACK, 1);
    }
    else{
        phyWriteBit(SR_AACK_PROM_MODE, 0);
        phyWriteBit(SR_AACK_DIS_ACK, 0);
    }
}

void atSetFrameRetries(char retry){
    phyWriteBit(SR_MAX_FRAME_RETRIES, retry);
}

void atSetCSMARetries(char retry){
    phyWriteBit(SR_MAX_CSMA_RETRIES, retry);
}

void atSetAntDiversity(char enable)
{
    phyWriteBit(SR_ANT_DIV_EN, enable);
    phyWriteBit(SR_ANT_EXT_SW_EN, enable);

}

void atRxPQueueInit(char rxpq_max_size) {

    rx_p_queue = pktqInit(rxpq_max_size);

}

void atTxPQueueInit(char txpq_max_size) {

    tx_p_queue = pktqInit(txpq_max_size);

}

PacketQueue atGetRxPacketQueue(void) {

    return rx_p_queue;

}

PacketQueue atGetTxPacketQueue(void) {

    return tx_p_queue;

}

/*****************************************************************************
* Function Name : phyReadId
* Description   : This function reads ID number of AT86RF231 chip
* Parameters    : A character array of size 4 to hold id value
* Return Value  : None
*****************************************************************************/
void phyReadId(unsigned char *id) {
    ipspi1ChipSelect(TRUE);     // just to make sure to set chip-select
    id[0] = phyReadReg(RG_PART_NUM);
    id[1] = phyReadReg(RG_VERSION_NUM);
    id[2] = phyReadReg(RG_MAN_ID_1);
    id[3] = phyReadReg(RG_MAN_ID_0);

}

unsigned char phyGetState(void) {
    return phyReadBit(SR_TRX_STATUS);
}

unsigned char phyReadRSSI(void){
    return phyReadBit(SR_RSSI);
}

unsigned char phyReadED(void){
    return phyReadReg(RG_PHY_ED_LEVEL);
}

//Return LQI from most recent packet received
unsigned char phyReadLQI(void) {
    return radio_lqi;
}

unsigned char phyGetLastAckd(void){
    return g_last_ackd;
}

MacPacket macCreatePacket() {
    MacPacket packet;
    packet = (MacPacket)malloc(sizeof(MacPacketStruct));
    packet->frame_ctrl.bits.packet_type = PACKET_TYPE_DATA;
    packet->frame_ctrl.bits.sec_en = SEC_EN_FALSE;
    packet->frame_ctrl.bits.frm_pending = FRM_PENDING_FALSE;
    packet->frame_ctrl.bits.ack_req = ACK_REQ_TRUE;
    packet->frame_ctrl.bits.pan_id_comp = PAN_ID_COMP_TRUE;
    packet->frame_ctrl.bits.reserved = RESERVED;
    packet->frame_ctrl.bits.dest_addr_mode = DEST_ADDR_MODE_16;
    packet->frame_ctrl.bits.frm_version = VERSION_2006;
    packet->frame_ctrl.bits.src_addr_mode = SRC_ADDR_MODE_16;
    packet->seq_num = 0;
    //NOTE: Reading these values from radio sometimes causes code to freeze!
    //packet->src_addr = macGetSrcAddr();
    //packet->src_pan_id = macGetSrcPANID(); 
    packet->src_addr = g_src_addr;
    packet->src_pan_id = g_src_pan_id;
    packet->dest_pan_id = packet->src_pan_id;
    return packet;
}

void macDeletePacket(MacPacket p) {
    free(p);
}


/*****************************************************************************
* Function Name : macReceivePacket
* Description   : receive data from radio without interrupt. 
*                 Users can call this function
*                 to manually receive frames. If ISR
*                 is used, then users do not have to call this function.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
MacPacket macReceivePacket() {
    
    byte i;
    Payload rx_payload;
    MacPacket mac_rx_packet = macCreatePacket();
    
    ipspi1ChipSelect(FALSE);     /* Select transceiver */
    ipspi1PutByte(TRX_CMD_FR);
   
    mac_rx_packet->payload_length             = ipspi1GetByte() - MAC_HEADER_LENGTH - CRC_LENGTH;
    mac_rx_packet->frame_ctrl.val.byte.LB     = ipspi1GetByte();
    mac_rx_packet->frame_ctrl.val.byte.HB     = ipspi1GetByte();
    mac_rx_packet->seq_num                    = ipspi1GetByte();
    mac_rx_packet->dest_pan_id.byte.LB        = ipspi1GetByte();
    mac_rx_packet->dest_pan_id.byte.HB        = ipspi1GetByte();
    mac_rx_packet->dest_addr.byte.LB          = ipspi1GetByte();
    mac_rx_packet->dest_addr.byte.HB          = ipspi1GetByte();
    if (!mac_rx_packet->frame_ctrl.bits.pan_id_comp) {
        mac_rx_packet->src_pan_id.byte.LB     = ipspi1GetByte();
        mac_rx_packet->src_pan_id.byte.HB     = ipspi1GetByte();
    }
    mac_rx_packet->src_addr.byte.LB           = ipspi1GetByte();
    mac_rx_packet->src_addr.byte.HB           = ipspi1GetByte();
    rx_payload = payCreateEmpty(mac_rx_packet->payload_length - PAYLOAD_HEADER_LENGTH);
    for (i = 0; i < mac_rx_packet->payload_length; ++i) {
        rx_payload->pld_data[i]               = ipspi1GetByte();
    }
    mac_rx_packet->payload = rx_payload;

    radio_lqi                                 = ipspi1GetByte(); 
    ipspi1ChipSelect(TRUE);     /* deselect transceiver */
    
    return mac_rx_packet;
}

/*****************************************************************************
* Function Name : macSendPacket
* Description   : send out data
* Parameters    : None 
* Return Value  : None                                                     
* Assumptions   : IntraPAN communications only (for now)
*****************************************************************************/
void macSendPacket(void) {
    if (g_current_state ==  STATE_BUSY_TX_ARET) {
        return;
    }

    byte state, i;
    static byte seq_num = 0;

    while (1) {     // wait until radio is not busy
        state = phyReadBit(SR_TRX_STATUS);
        if (state == CMD_TX_ARET_ON) {
            break;
        } else if (state == CMD_RX_AACK_ON) {
            phyWriteReg(RG_TRX_STATE, CMD_PLL_ON);
            phyWriteReg(RG_TRX_STATE, CMD_TX_ARET_ON); 
            break;
        }
    }
    
    MacPacket mac_tx_packet = (MacPacket)pktqPop(tx_p_queue);
    g_current_state = STATE_BUSY_TX_ARET;
    phySetSlptr(1);
    phySetSlptr(0);

    ipspi1ChipSelect(FALSE);
    ipspi1PutByte(TRX_CMD_FW);

    ipspi1PutByte(mac_tx_packet->payload_length + MAC_HEADER_LENGTH + CRC_LENGTH);

    ipspi1PutByte(mac_tx_packet->frame_ctrl.val.byte.LB);
    ipspi1PutByte(mac_tx_packet->frame_ctrl.val.byte.HB);
    ipspi1PutByte(seq_num);
    seq_num++;
    ipspi1PutByte(mac_tx_packet->dest_pan_id.byte.LB);
    ipspi1PutByte(mac_tx_packet->dest_pan_id.byte.HB);
    ipspi1PutByte(mac_tx_packet->dest_addr.byte.LB);
    ipspi1PutByte(mac_tx_packet->dest_addr.byte.HB);
    ipspi1PutByte(mac_tx_packet->src_addr.byte.LB);
    ipspi1PutByte(mac_tx_packet->src_addr.byte.HB);

    for (i = 0; i < mac_tx_packet->payload_length; i++)
    {
        ipspi1PutByte(mac_tx_packet->payload->pld_data[i]);
    }

    ipspi1ChipSelect(TRUE);
    payDelete(mac_tx_packet->payload);
    macDeletePacket(mac_tx_packet);

    //NOTE: Probably should set g_last_acked = 0 here...
}


/******************************************************************************
* Function Name : macGetSrcPANID 
* Description   : Read the register containing the source PAN ID and
* Parameters    : None
* Return Value  : WordVal structure containing the source PAN ID 
*******************************************************************************/
WordVal macGetSrcPANID() {

    WordVal pan_id;
    pan_id.byte.LB = phyReadReg(RG_PAN_ID_0);
    pan_id.byte.HB = phyReadReg(RG_PAN_ID_1);
    
    return pan_id;

}

/******************************************************************************
* Function Name : macSetSrcPANID 
* Description   : Set the source PAN ID 
* Parameters    : WordVal structure containing the source PAN ID 
* Return Value  : none 
*******************************************************************************/
void macSetSrcPANID(WordVal pan_id) {

    g_src_pan_id = pan_id;
    phyWriteReg(RG_PAN_ID_0, pan_id.byte.LB);
    phyWriteReg(RG_PAN_ID_1, pan_id.byte.LB);

}

/******************************************************************************
* Function Name : macGetSrcAddr 
* Description   : Read the radio register and get the source network address
* Parameters    : none
* Return Value  : WordVal structure containing the source network address 
*******************************************************************************/
WordVal macGetSrcAddr() {

    WordVal src_addr;
    src_addr.byte.LB = phyReadReg(RG_SHORT_ADDR_0);
    src_addr.byte.HB = phyReadReg(RG_SHORT_ADDR_1);

    return src_addr;

}

/******************************************************************************
* Function Name : macSetSrcAddr 
* Description   : Set the radio's source network address
* Parameters    : WordVal structure containing the source network address
* Return Value  : none
*******************************************************************************/
void macSetSrcAddr(WordVal src_addr) {

    g_src_addr = src_addr;
    phyWriteReg(RG_SHORT_ADDR_0, src_addr.byte.LB);
    phyWriteReg(RG_SHORT_ADDR_1, src_addr.byte.HB);

}


/******************************************************************************
* Function Name : macGetDestPANID
* Description   : Get the destination radio's PAN ID x
* Parameters    : none
* Return Value  : WordVal structure containing the destination PAN ID 
*******************************************************************************/
WordVal macGetDestPANID() {

    return g_dest_pan_id;

}


/******************************************************************************
* Function Name : macSetDestPANID
* Description   : Set the PAN ID for the destination radio
* Parameters    : WordVal structure containing the destination PAN ID 
* Return Value  : none
*******************************************************************************/
void macSetDestPANID(WordVal pan_id) {

    g_dest_pan_id = pan_id;    

}


/******************************************************************************
* Function Name : macGetDestAddr
* Description   : Get the network address for the destination radio
* Parameters    : none
* Return Value  : WordVal structure containing the destination radio's network
*                 address
*******************************************************************************/
WordVal macGetDestAddr() {

    return g_dest_addr; 

}


/******************************************************************************
* Function Name : macSetDestAddr
* Description   : Set the network address for the destination radio
* Parameters    : WordVal structure containing the destination radio's network
*                 address
* Return Value  : none
*                 
*******************************************************************************/
void macSetDestAddr(WordVal addr) {

    g_dest_addr = addr; 

}

/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------

 * The functions below are intend    ed for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/


/*****************************************************************************
* Function Name : _INT4Interrupt
* Description   : Interrupt hander for 802.15.4 radio
* Parameters    : None
* Return Value  : None
*****************************************************************************/

void __attribute__((interrupt, no_auto_psv)) _INT4Interrupt(void) {
    phyHandleISR();
    _INT4IF = 0;    // Clear the interrupt flag
}


/*****************************************************************************
* Function Name : phyHandleISR
* Description   : The function is called by interrupt, IRQ_TRX_END. EXT_INT4 
*                 should be enabled and RD11 should be set as input.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void phyHandleISR(void) {
    byte irq_cause, trx_status;

    irq_cause = phyReadReg(RG_IRQ_STATUS);
    if (irq_cause & TRX_IRQ_TRX_END) {
        if (g_current_state == STATE_RX_AACK_ON) { // new packet arrived
            radioEnqueueRxPacket(macReceivePacket());            // receive the packet   
        } 
		else 
        {    // transmit is completed.
            trx_status = phyReadBit(SR_TRAC_STATUS);
            if (trx_status == TRAC_NO_ACK)
            {
                    g_last_ackd = 0;
            }
            if (trx_status != TRAC_SUCCESS)
            {
		g_last_ackd = 0;
                // Handle unsuccessful ACKs here
                // Code to be added later
                LED_2 = ~LED_2;
            }
			else{
				g_last_ackd = 1;
			}
			
            if(pktqIsEmpty(tx_p_queue)) {
                phyWriteReg(RG_TRX_STATE, CMD_PLL_ON);
                phyWriteReg(RG_TRX_STATE, CMD_RX_AACK_ON); // waiting for RX
                g_current_state = STATE_RX_AACK_ON;
            } 
            else {
                g_current_state = STATE_TX_ARET_ON;
		g_last_ackd = 0;
                macSendPacket();
            }
            
        }
    }
}


/*****************************************************************************
* Function Name : phySetSlptr
* Description   : Set the level of the SLP_TR pin.
* Parameters    : val -> 0 for LOW or 1 for HIGH level of the pin
* Return Value  : None                                                     
*****************************************************************************/
void phySetSlptr(char	val) {  	
    SLPTR = val;
}


/*****************************************************************************
* Function Name : phyReadReg                                           
* Description   : Read the value from a register.
* Parameters    : addr - the offset of the register
* Return Value  : register value                                                      
*****************************************************************************/
byte phyReadReg(byte addr) {
    byte c;
    ipspi1ChipSelect(FALSE);
    ipspi1PutByte(TRX_CMD_RR | addr);
    c = ipspi1GetByte();
    ipspi1ChipSelect(TRUE);
    return c;
}


/*****************************************************************************
* Function Name : phyWriteReg                                           
* Description   : Write a value to a register.
* Parameters    : addr 	 the offset of the register
*    	          val 	 the value to be written
* Return Value  : None                                                     
*****************************************************************************/
void phyWriteReg(byte addr, byte val) {
    ipspi1ChipSelect(FALSE);
    ipspi1PutByte(TRX_CMD_RW | addr);
    ipspi1PutByte(val);
    ipspi1ChipSelect(TRUE);
}


/*****************************************************************************
* Function Name : phyReadBit                                           
* Description   : Read a bit from a register.
* Parameters    : use sub-registers defined in at86rf231.h
* Return Value  : register bit                                                      
*****************************************************************************/
byte phyReadBit(byte addr, byte mask, byte pos) {

    byte data;
    data = phyReadReg(addr);
    data &= mask;
    data >>= pos;
    return data;
}

/*****************************************************************************
* Function Name : phyWriteBit                                           
* Description   : Write a bit to a register.
* Parameters    : use sub-registers defined in at86rf231.h
* Return Value  : None                             o                        
*****************************************************************************/
void phyWriteBit(byte addr, byte mask, byte pos, byte val) {
    byte temp;
    temp = phyReadReg(addr);
    temp &= ~mask;
    val <<= pos;
    val &= mask;
    val |= temp;
    phyWriteReg(addr, val);
}
