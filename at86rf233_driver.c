/**
 * Copyright (c) 2011, Regents of the University of California
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
 * Atmel AT86RF231 802.15.4 RF Transceiver IC Driver
 *
 * by Humphrey Hu
 *
 * v0.4
 *
 * Revisions:
 *  Humphrey Hu         2012-01-06      Initial implementation
 *  Humphrey Hu         2012-02-08      Added state transition methods
 *
 * Notes:
 *    No-Clock (sleep) RX listening mode not yet implemented
 */

#include "at86rf233.h"  // Current transceiver IC
#include "at86rf231_driver.h"
#include "ports.h"
#include "mac_packet.h"
#include "payload.h"
#include "spi.h"                // SFRs
#include "radio.h"              // Need radio status codes
#include "spi_controller.h"
#include "utils.h"

#include <string.h>

// Basic commands
#define TRX_CMD_RW          (0xC0) // Register Write
#define TRX_CMD_RR          (0x80) // Register Read
#define TRX_CMD_FW          (0x60) // Frame Transmit Mode
#define TRX_CMD_FR          (0x20) // Frame Receive Mode
#define TRX_CMD_SW          (0x40) // SRAM Write.
#define TRX_CMD_SR          (0x00) // SRAM Read.


// Based on 16-bit addressing for PAN and device and no
// auxiliary security header
// TODO: Move to appropriate header file, probably mac_packet.h
#define MAC_HEADER_LENGTH       (9)
#define CRC_LENGTH              (2)
#define FRAME_BUFFER_SIZE       (128)
#define DEFAULT_CSMA_RETRIES    (4)     /** Number of times to attempt medium acquisition */
#define DEFAULT_FRAME_RETRIES   (0)     /** Number of times to attempt frame resend */

// =========== Function stubs =================================================

static void trxSpiCallback(unsigned int irq_cause);
static void trxFillBuffer(void);
static void trxReadBuffer(void);

static void setupSPI(void);
static inline void trxSetSlptr(unsigned char);

static void trxWriteReg(unsigned char addr, unsigned char val);
static unsigned char trxReadReg(unsigned char addr);

static void trxWriteSubReg(unsigned char addr, unsigned char mask, unsigned char pos, unsigned char val);
static unsigned char trxReadSubReg(unsigned char addr, unsigned char mask, unsigned char pos);


// =========== Static variables ===============================================

static unsigned char is_ready = 0; // Mostly for debugging - no checking so code is faster
static TrxIrqHandler irqCallback;
// See at86rf231.h
static tal_trx_status_t trx_state;
static unsigned char frame_buffer[FRAME_BUFFER_SIZE];
static unsigned char last_rssi;
static unsigned char last_ackd = 0;


// =========== Public functions ===============================================

void trxSetup(unsigned char cs)
{

    // SPI setup
    setupSPI();     // Set up SPI com port
    spic1SetCallback(cs, &trxSpiCallback);  // Configure callback for spi interrupts
    trxReadReg(RG_IRQ_STATUS);   // Clear pending interrupts
    trxSetStateOff(); // Transition to TRX_OFF for configuring device
    trxWriteSubReg(SR_IRQ_MASK, TRX_IRQ_3_TRX_END); // Interrupt at end of transceive
    trxWriteSubReg(SR_SLOTTED_OPERATION, 0);  // Disable slotted operation
    trxWriteSubReg(SR_TX_AUTO_CRC_ON, 1); // Enable automatic TX CRC
    trxWriteSubReg(SR_CLKM_CTRL, CLKM_NO_CLOCK); // No clock on CLKM pin
    trxWriteSubReg(SR_MAX_CSMA_RETRIES, DEFAULT_CSMA_RETRIES); // Set CSMA attempts
    trxWriteSubReg(SR_MAX_FRAME_RETRIES, DEFAULT_FRAME_RETRIES); // Set resend attempts
    trxWriteSubReg(SR_RX_SAFE_MODE, 0); // Disable frame buffer protection
    trxWriteSubReg(SR_AACK_FVN_MODE, FRAME_VERSION_IGNORED); // Ignore frame version
    trxWriteSubReg(SR_SPI_CMD_MODE, SPI_CMD_MODE_PHY_RSSI); // First byte of SPI is RSSI register
    trxSetStateIdle();

    //Smart Receive mode
    trxWriteSubReg(SR_RX_RPC_EN, 1);
    //PLL energy save mode
    //trxWriteSubReg(SR_PLL_RPC_EN, 1);

    ConfigINT4(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_5); // Radio IC interrupt

    last_rssi = 0;
    is_ready = 1;
}


void trxReset(void) {

    unsigned int i;

    spic1Reset();   // Reset comm
    trxWriteSubReg(SR_TRX_CMD, CMD_TRX_OFF);  // Force to off state

    i = 0;
    while(i < 0xFFF0) {
        if(trxReadSubReg(SR_TRX_STATUS) == TRX_OFF) { break; }
        i++;
    }
    trx_state = TRX_OFF;

}

void trxCalibrate(void) {

    trxWriteSubReg(SR_FTN_START, 1);    // Begin calibration routine
    while(trxReadSubReg(SR_FTN_START) != 0);    // Wait for completion

}

void trxSetAddress(unsigned int addr) {

    trxWriteReg(RG_SHORT_ADDR_0, (addr & 0xff));
    trxWriteReg(RG_SHORT_ADDR_1, ((addr >> 8) & 0xff));

}

void trxSetPan(unsigned int pan) {

    trxWriteReg(RG_PAN_ID_0, (pan & 0xff));
    trxWriteReg(RG_PAN_ID_1, ((pan >> 8) & 0xff));

}

void trxSetChannel(unsigned char channel) {

    trxWriteSubReg(SR_CHANNEL, channel);

}

void trxSetRetries(unsigned int retries) {

    trxWriteSubReg(SR_MAX_FRAME_RETRIES, retries);

}

void trxSetIrqCallback(TrxIrqHandler handler) {

    irqCallback = handler;

}

void trxReadId(unsigned char *id) {

    id[0] = trxReadReg(RG_PART_NUM);      // should be 3
    id[1] = trxReadReg(RG_VERSION_NUM);   // should be 2
    id[2] = trxReadReg(RG_MAN_ID_1);      // should be 0x1F
    id[3] = trxReadReg(RG_MAN_ID_0);      // should be 0

}

// TODO: Have to check if this is valid in extended operating mode or not
unsigned char trxReadRSSI(void) {

    return 0x1F & last_rssi;

}

unsigned char trxReadED(void) {

    return trxReadReg(RG_PHY_ED_LEVEL);

}

unsigned char trxGetLastACKd(void) {

    return last_ackd;

}

void trxWriteFrameBuffer(MacPacket packet) {

    unsigned int i;
    unsigned char phy_len;
    Payload pld;

    // Linearize contents in buffer
    i = 0;
    phy_len = packet->payload_length + MAC_HEADER_LENGTH + CRC_LENGTH;

    frame_buffer[i++] = phy_len; //packet->payload_length + MAC_HEADER_LENGTH + CRC_LENGTH;
    frame_buffer[i++] = packet->frame_ctrl.val.byte.LB;
    frame_buffer[i++] = packet->frame_ctrl.val.byte.HB;
    frame_buffer[i++] = packet->seq_num;
    frame_buffer[i++] = packet->dest_pan_id.byte.LB;
    frame_buffer[i++] = packet->dest_pan_id.byte.HB;
    frame_buffer[i++] = packet->dest_addr.byte.LB;
    frame_buffer[i++] = packet->dest_addr.byte.HB;
    frame_buffer[i++] = packet->src_addr.byte.LB;
    frame_buffer[i++] = packet->src_addr.byte.HB;

    pld = macGetPayload(packet);
    if(pld == NULL) { return; }

    memcpy(frame_buffer + i, payToString(pld), payGetPayloadLength(pld));

    spic1BeginTransaction(TRX_CS);
    spic1Transmit(TRX_CMD_FW);
    spic1MassTransmit(phy_len, frame_buffer, phy_len*3); // 3*length microseconds timeout seems to work well

}

unsigned int trxReadFrameBuffer(MacPacket packet) {

    // Read received data from DMA memory into buffer
    return macReadFrame(frame_buffer, packet); // Decode frame data

}

unsigned int trxReadBufferDataLength(void) {

    return macReadDataLength(frame_buffer);

}

void trxBeginTransmission(void) {

    trxSetSlptr(1);
    trxSetSlptr(0);
    trx_state = BUSY_TX_ARET;   // Update state accordingly
    last_ackd = 0;
}

void trxSetStateTx(void) {

    CRITICAL_SECTION_START;

    if(trx_state == TX_ARET_ON) { // Fast return if already in desired state
        CRITICAL_SECTION_END;
        return;
    }

    trxWriteSubReg(SR_TRX_CMD, CMD_TX_ARET_ON); // Begin transition
    while(trxReadSubReg(SR_TRX_STATUS) != TX_ARET_ON);  // Wait for completion
    trx_state = TX_ARET_ON; // Update state

    CRITICAL_SECTION_END;

}

void trxSetStateRx(void) {

    CRITICAL_SECTION_START;

    if(trx_state == RX_AACK_ON) {  // Fast return
        CRITICAL_SECTION_END;
        return;
    }

    trxWriteSubReg(SR_TRX_CMD, CMD_RX_AACK_ON); // Begin transition
    while(trxReadSubReg(SR_TRX_STATUS) != RX_AACK_ON);  // Wait for completion
    trx_state = RX_AACK_ON; // Update state

    CRITICAL_SECTION_END;

}

void trxSetStateIdle(void) {

    CRITICAL_SECTION_START;

    if(trx_state == PLL_ON) {  // Fast return
        CRITICAL_SECTION_END;
        return;
    }

    trxWriteSubReg(SR_TRX_CMD, CMD_PLL_ON); // Begin transition
    while(trxReadSubReg(SR_TRX_STATUS) != PLL_ON);  // Wait for completion
    trx_state = PLL_ON; // Update state

    CRITICAL_SECTION_END;

}

void trxSetStateOff(void) {

    CRITICAL_SECTION_START;

    if(trx_state == TRX_OFF) {     // Fast return
        CRITICAL_SECTION_END;
        return;
    }

    trxWriteSubReg(SR_TRX_CMD, CMD_TRX_OFF);    // Begin transition
    while(trxReadSubReg(SR_TRX_STATUS) != TRX_OFF); // Wait for completion
    trx_state = TRX_OFF;    // Update state

    CRITICAL_SECTION_END;

}

// =========== Private functions ==============================================

/**
 * Writes a byte to a transceiver register.
 *
 * @param addr 8-bit register address. Use definitions in at86rf231.h
 * @param val 8-bit value to write to the register
 */
static void trxWriteReg(unsigned char addr, unsigned char val) {

    spic1BeginTransaction(TRX_CS);
    spic1Transmit(TRX_CMD_RW | addr);
    spic1Transmit(val);
    spic1EndTransaction();

}

/**
 * Reads a byte from a transceiver register.
 *
 * @param addr 8-bit register address. Use definitions in at86rf231.h
 * @return Read register contents
 */
static unsigned char trxReadReg(unsigned char addr) {

    unsigned char c;
    spic1BeginTransaction(TRX_CS);
    spic1Transmit(TRX_CMD_RR | addr);
    c = spic1Receive();
    spic1EndTransaction();
    return c;

}

/**
 * Write bits to a transceiver subregister.
 *
 * @param addr 8-bit main register address. Use definitions in at86rf231.h
 * @param mask Bitfield mask to apply to register value
 * @param pos Subregister position offset from main register LSB
 * @param val Value to write to subregister
 */
static void trxWriteSubReg(unsigned char addr, unsigned char mask, unsigned char pos, unsigned char val) {

    unsigned char temp;
    temp = trxReadReg(addr);
    temp &= ~mask;
    val <<= pos;
    val &= mask;
    val |= temp;
    trxWriteReg(addr, val);

}

/**
 * Read bits from a transceiver subregister.
 *
 * @param addr 8-bit main register address. Use definitions in at86rf231.h
 * @param mask Bitfield mask to apply to register value
 * @param pos Subregister position offset from main register LSB
 * @return Value read from subregister.
 */
static unsigned char trxReadSubReg(unsigned char addr, unsigned char mask, unsigned char pos) {

    unsigned char data;
    data = trxReadReg(addr);
    data &= mask;
    data >>= pos;
    return data;

}

void __attribute__((interrupt, no_auto_psv)) _INT4Interrupt(void) {

    unsigned char irq_cause, status;
    irq_cause = 0;
    status = 0xFF;

    irq_cause = trxReadReg(RG_IRQ_STATUS);    // Read and clear irq source

    if(irq_cause & TRX_IRQ_3_TRX_END) {

        status = trxReadSubReg(SR_TRAC_STATUS); // Determine transaction status

        // Transmit complete case
        if(trx_state == BUSY_TX_ARET) {

            trx_state = TX_ARET_ON; // State transition

            if(status == TRAC_SUCCESS) {
                last_ackd = 1;
                irqCallback(RADIO_TX_SUCCESS);
            } else if(status == TRAC_SUCCESS_DATA_PENDING) {
                irqCallback(RADIO_TX_SUCCESS);
            } else if(status == TRAC_CHANNEL_ACCESS_FAILURE) {
                irqCallback(RADIO_TX_FAILURE);
            } else if(status == TRAC_NO_ACK) {
                last_ackd = 0;
                irqCallback(RADIO_TX_FAILURE);
            } else if(status == TRAC_INVALID) {
                irqCallback(RADIO_TX_FAILURE);
            }

        } else if(trx_state == RX_AACK_ON) {

            // crc_valid = trxReadSubReg(SR_RX_CRC_VALID);
            // if(!crc_valid) {
            //    Drop packet if invalid
            // }
            if(status == TRAC_SUCCESS) {
                trxFillBuffer();
                irqCallback(RADIO_RX_START);
            } else if(status == TRAC_SUCCESS_WAIT_FOR_ACK) {
                trxFillBuffer();
                irqCallback(RADIO_RX_START);
                // TODO: Add support for proper slotted ACK operation
            } else if(status == TRAC_INVALID) {
                irqCallback(RADIO_RX_FAILURE);
            }
        }
    }

    _INT4IF = 0;                                // Clear interrupt flag

}

static void trxSpiCallback(unsigned int interrupt_code) {

    if(interrupt_code == SPIC_TRANS_SUCCESS) {

        spic1EndTransaction(); // End previous transaction

        if(trx_state == RX_AACK_ON) {
            trxReadBuffer();
            irqCallback(RADIO_RX_SUCCESS);

        } else if(trx_state == TX_ARET_ON) {

            // Packet was successfully transferred to the radio buffer
            // Do something?

        }

    } else if(interrupt_code == SPIC_TRANS_TIMEOUT) {

        // This is indicative of some form of failure!
        spic1Reset();
        irqCallback(RADIO_HW_FAILURE);

    }

}

// TODO: Optimize to read only packet length, not whole buffer
/**
 * Begin transfer of frame data from transceiver into DMA buffer
 */
static void trxFillBuffer(void) {

    spic1BeginTransaction(TRX_CS);
    last_rssi = spic1Transmit(TRX_CMD_FR);  // Begin write (returns RSSI because of SPI_CMD_MODE)
    //current_phy_len = spic1Receive(); // Read physical frame size
    //spic1MassTransmit(current_phy_len, NULL, current_phy_len*3); // DMA rest into buffer
    spic1MassTransmit(FRAME_BUFFER_SIZE, NULL, FRAME_BUFFER_SIZE*3); // DMA entire frame buffer into memory

}

/**
 * Copy frame buffer contents from DMA into static software buffer
 */
static void trxReadBuffer(void) {

    spic1ReadBuffer(FRAME_BUFFER_SIZE, frame_buffer);

}

static void setupSPI(void)
{
    spicSetupChannel1(TRX_CS,
                      ENABLE_SCK_PIN &
                      ENABLE_SDO_PIN &
                      SPI_MODE16_OFF &
                      SPI_SMP_OFF &
                      SPI_CKE_ON &
                      SLAVE_ENABLE_OFF &
                      CLK_POL_ACTIVE_HIGH &
                      MASTER_ENABLE_ON &
                      PRI_PRESCAL_1_1 &
                      SEC_PRESCAL_6_1);
}

static inline void trxSetSlptr(unsigned char val) {
    SLPTR = val;
    Nop();
    Nop();
}
