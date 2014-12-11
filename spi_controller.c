/**
 * Copyright (c) 2011-2013, Regents of the University of California
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
 * Master Mode SPI Controller for the dsPIC33F
 *
 * by Humphrey Hu
 *
 * Revisions:
 *  Humphrey Hu     2011-11-10      Initial implemetation
 *  Humphrey Hu     2012-02-09      Code refactor and comments
 *
 * Notes:
 *  - Add #defines to remove buffers when not in use.
 *  - If DMA operations never return, calls to blocking methods will deadlock.
 *    This is a problem if a higher-level timeout fires before the spic timeout
 */

#include "spi_controller.h"
#include "atomic.h"
#include "spi.h"
#include "timer.h"
#include "dma.h"
#include "init_default.h"
#include "utils.h"

#include <string.h>

/*
 This is now done in bsp-ip*.h
// This section is board-specific
// TODO: Generalize or move to BSP header
#if defined(__IMAGEPROC2)

    #define SPI1_CS             (_LATB2)    // Radio Chip Select
    #define SPI2_CS1            (_LATG9)    // Flash Chip Select
    #define SPI2_CS2            (_LATC15)   // MPU6000 Chip Select

#endif
*/

// DMA channels allocated as per Wiki assignments
#define SPIC1_DMAR_CONbits      (DMA2CONbits)
#define SPIC1_DMAR_CNT          (DMA2CNT)
#define SPIC1_DMAR_REQbits      (DMA2REQbits)
#define SPIC1_DMAW_CONbits      (DMA3CONbits)
#define SPIC1_DMAW_CNT          (DMA3CNT)
#define SPIC1_DMAW_REQbits      (DMA3REQbits)

#define SPIC2_DMAR_CONbits      (DMA4CONbits)
#define SPIC2_DMAR_CNT          (DMA4CNT)
#define SPIC2_DMAR_REQbits      (DMA4REQbits)
#define SPIC2_DMAW_CONbits      (DMA5CONbits)
#define SPIC2_DMAW_CNT          (DMA5CNT)
#define SPIC2_DMAW_REQbits      (DMA5REQbits)

#define SPI1_REQ_VAL            (0x00A) // SPI1 Transfer Done Interrupt
#define SPI2_REQ_VAL            (0x021) // SPI2 Transfer Done Interrupt

#define SPIC1_RX_BUFF_LEN       (128) // Radio buffer is 128 bytes
#define SPIC1_TX_BUFF_LEN       (128)

#define SPIC2_RX_BUFF_LEN       (528) // Flash page is 264/528 bytes
#define SPIC2_TX_BUFF_LEN       (528) // Currently not in use

#define US_TO_TICKS(X)          ((X*10)/16) // Microseconds to cycles with 64:1 prescale

#define SPI_CS_ACTIVE           (0)
#define SPI_CS_IDLE             (1)


// =========== Function Prototypes ============================================
static void setupDMASet1(void);
static void setupDMASet2(void);

// =========== Static Variables ===============================================

/** Interrupt handlers */
static SpicIrqHandler int_handler_ch1[1];
static SpicIrqHandler int_handler_ch2[2];

/** Port configurations */
static unsigned int spicon_ch1[1];
static unsigned int spicon_ch2[2];

/** Current port statuses */
DECLARE_SPINLOCK_H(spi_port_ch1);
DECLARE_SPINLOCK_C(spi_port_ch1);
DECLARE_SPINLOCK_H(spi_port_ch2);
DECLARE_SPINLOCK_C(spi_port_ch2);

/** Current port chip select */
static unsigned char port_cs_line[SPIC_NUM_PORTS];

// Port 1 buffers
static unsigned char spic1_rx_buff[SPIC1_RX_BUFF_LEN] __attribute__((space(dma)));
static unsigned char spic1_tx_buff[SPIC1_TX_BUFF_LEN] __attribute__((space(dma)));

// Port 2 buffers
static unsigned char spic2_rx_buff[SPIC2_RX_BUFF_LEN] __attribute__((space(dma)));
static unsigned char spic2_tx_buff[SPIC2_TX_BUFF_LEN] __attribute__((space(dma)));

// =========== Public Methods =================================================

void spicSetupChannel1(unsigned char cs, unsigned int spiCon1) {

    setupDMASet1();                     // Set up DMA channels
    spicon_ch1[cs] = spiCon1;           // Remember SPI config
    spi_port_ch1_reset();               // Initialize status

}

void spicSetupChannel2(unsigned char cs, unsigned int spiCon1) {

    setupDMASet2();
    spicon_ch2[cs] = spiCon1;           // Remember SPI config
    spi_port_ch2_reset();               // Initialize status

}

void spic1SetCallback(unsigned char cs, SpicIrqHandler handler) {

    int_handler_ch1[cs] = handler;

}


void spic2SetCallback(unsigned char cs, SpicIrqHandler handler) {

    int_handler_ch2[cs] = handler;

}

int spic1BeginTransaction(unsigned char cs) {
    // TODO: Timeout?

    // TODO: generalize?
    if (cs > 0)
      // Only one CS line is supported
      return -1;

    spi_port_ch1_lock(); // Wait for port to become available
    // Reconfigure port
    SPI1STAT = 0;
    SPI1CON1 = spicon_ch1[cs];
    SPI1STAT = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;
    port_cs_line[0] = cs;
    SPI1_CS = SPI_CS_ACTIVE;    // Activate chip select

    return 0;
}

int spic2BeginTransaction(unsigned char cs) {
    // TODO: Timeout?

    // TODO: generalize?
    if (cs > 1)
      // Two CS lines are supported
      return -1;

    spi_port_ch2_lock(); // Wait for port to become available
    // Reconfigure port
    SPI2STAT = 0;
    SPI2CON1 = spicon_ch2[cs];
    SPI2STAT = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;
    port_cs_line[1] = cs;
    if (cs == 0)
      SPI2_CS1 = SPI_CS_ACTIVE;     // Activate chip select
    if (cs == 1)
      SPI2_CS2 = SPI_CS_ACTIVE;     // Activate chip select

    return 0;
}

void spic1EndTransaction(void) {

    // Only one CS line
    SPI1_CS = SPI_CS_IDLE;  // Idle chip select after freeing since may cause irq
    spi_port_ch1_unlock();  // Free port

}

void spic2EndTransaction(void) {

    if (port_cs_line[1] == 0)
      SPI2_CS1 = SPI_CS_IDLE;  // Idle chip select
    if (port_cs_line[1] == 1)
      SPI2_CS2 = SPI_CS_IDLE;  // Idle chip select
    spi_port_ch2_unlock();     // Free port

}

void spic1Reset(void) {

    SPI1_CS = SPI_CS_IDLE;          // Disable chip select
    SPIC1_DMAR_CONbits.CHEN = 0;    // Disable DMA module
    SPIC1_DMAW_CONbits.CHEN = 0;
    SPI1STATbits.SPIROV = 0;        // Clear overwrite bit
    spi_port_ch2_unlock();          // Release lock on channel

}

void spic2Reset(void) {

    SPI2_CS1 = SPI_CS_IDLE;         // Disable chip select
    SPI2_CS2 = SPI_CS_IDLE;         // Disable chip select
    SPIC2_DMAR_CONbits.CHEN = 0;    // Disable DMA module
    SPIC2_DMAW_CONbits.CHEN = 0;
    SPI2STATbits.SPIROV = 0;
    spi_port_ch2_unlock();          // Release lock on channel

}

unsigned char spic1Transmit(unsigned char data) {

    unsigned char c;
    SPI1STATbits.SPIROV = 0;        // Clear overflow bit
    SPI1BUF = data;                 // Initiate SPI bus cycle by byte write
    while(SPI1STATbits.SPITBF);     // Wait for transmit to complete
    while(!SPI1STATbits.SPIRBF);    // Wait for receive to complete
    c = SPI1BUF;                    // Read out received data to avoid overflow
    return c;

}

unsigned char spic2Transmit(unsigned char data) {

    unsigned char c;
    SPI2STATbits.SPIROV = 0;        // Clear overflow bit
    SPI2BUF = data;                 // Initiate SPI bus cycle by byte write
    while(SPI2STATbits.SPITBF);     // Wait for transmit to complete
    while(!SPI2STATbits.SPIRBF);    // Wait for receive to complete
    c = SPI2BUF;                    // Read out received data to avoid overflow
    return c;

}

// Note that this is the same as transmit with data = 0x00
unsigned char spic1Receive(void) {

    unsigned char c;
    SPI1STATbits.SPIROV = 0;        // Clear overflow bit
    SPI1BUF = 0x00;                 // Initiate SPI bus cycle by byte write
    while(SPI1STATbits.SPITBF);     // Wait for transmit to complete
    while(!SPI1STATbits.SPIRBF);    // Wait for receive to complete
    c = SPI1BUF;                    // Read out received data to avoid overflow
    return c;

}


// Note that this is the same as transmit with data = 0x00
unsigned char spic2Receive(void) {

    unsigned char c;
    SPI2STATbits.SPIROV = 0;        // Clear overflow bit
    SPI2BUF = 0x00;                   // Initiate SPI bus cycle by byte write
    while(SPI2STATbits.SPITBF);        // Wait for transmit to complete
    while(!SPI2STATbits.SPIRBF);    // Wait for receive to complete
    c = SPI2BUF;                    // Read out received data to avoid overflow
    return c;

}


unsigned int spic1MassTransmit(unsigned int len, unsigned char *buff, unsigned int timeout) {

    // Make sure requested length is in range
    if(len > SPIC1_TX_BUFF_LEN) {
        len = SPIC1_TX_BUFF_LEN;
    }

    // If data is to be written
    if(buff != NULL) {
        memcpy(spic1_tx_buff, buff, len);   // Copy data to DMA memory
        SPIC1_DMAR_CONbits.NULLW = 0;   // Ensure null writes are disabled
        SPIC1_DMAW_CONbits.NULLW = 0;
    } else {
        SPIC1_DMAR_CONbits.NULLW = 1;   // Else use null write mode
        SPIC1_DMAW_CONbits.NULLW = 1;
    }

    //From dsPIC33 datasheet:
    //    DMAxCNT + 1 represents the number of DMA requests the channel must
    //    service before the data block transfer is considered complete.
    //    That is, a DMAxCNT value of ?0? will transfer one element.
    //So, DMAx_CNT = len-1 below. Noted here to prevent confusion.
    //TODO: will this cause a problem if called with len=0?
    SPIC1_DMAR_CNT = len - 1;
    SPIC1_DMAW_CNT = len - 1;
    SPIC1_DMAR_CONbits.CHEN = 1;    // Begin transmission
    SPIC1_DMAW_CONbits.CHEN = 1;
    SPIC1_DMAW_REQbits.FORCE = 1;
    return len;

}


/**
 * Transmit the contents of a buffer on port 2 via DMA
 */
unsigned int spic2MassTransmit(unsigned int len, unsigned char *buff, unsigned int timeout) {

    // Make sure requested length is in range
    if(len > SPIC2_TX_BUFF_LEN) {
        len = SPIC2_TX_BUFF_LEN;
    }

    // If data is to be written
    if(buff != NULL) {
        memcpy(spic2_tx_buff, buff, len);   // Copy data to DMA memory
        SPIC2_DMAR_CONbits.NULLW = 0;   // Ensure null writes are disabled
        SPIC2_DMAW_CONbits.NULLW = 0;
    } else {
        SPIC2_DMAR_CONbits.NULLW = 1;   // Else use null write mode
        SPIC2_DMAW_CONbits.NULLW = 1;
    }

    //From dsPIC33 datasheet:
    //    DMAxCNT + 1 represents the number of DMA requests the channel must
    //    service before the data block transfer is considered complete.
    //    That is, a DMAxCNT value of ?0? will transfer one element.
    //So, DMAx_CNT = len-1 below. Noted here to prevent confusion.
    //TODO: will this cause a problem if called with len=0?
    SPIC2_DMAR_CNT = len - 1;
    SPIC2_DMAW_CNT = len - 1;
    SPIC2_DMAR_CONbits.CHEN = 1;    // Begin transmission
    SPIC2_DMAW_CONbits.CHEN = 1;
    SPIC2_DMAW_REQbits.FORCE = 1;
    return len;

}

unsigned int spic1ReadBuffer(unsigned int len, unsigned char *buff) {

    // Make sure requested length is in range
    if(len > SPIC1_RX_BUFF_LEN) {
        len = SPIC1_RX_BUFF_LEN;
    }

    memcpy(buff, spic1_rx_buff, len);   // Read DMA buffer contents into buffer
    return len;

}

unsigned int spic2ReadBuffer(unsigned int len, unsigned char *buff) {

    // Make sure requested length is in range
    if(len > SPIC2_RX_BUFF_LEN) {
        len = SPIC2_RX_BUFF_LEN;
    }

    memcpy(buff, spic2_rx_buff, len);   // Read DMA buffer contents into buffer
    return len;
}

// =========== Private Functions ==============================================
// TODO: Check for DMA error codes and return appropriate interrupt cause
// ISR for DMA2 interrupt, currently DMAR for channel 1
void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void) {

    // Call registered callback function
    int_handler_ch1[port_cs_line[0]](SPIC_TRANS_SUCCESS);
    _DMA2IF = 0;

}

// ISR for DMA3 interrupt, currently DMAW for channel 1
void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void) {

    _DMA3IF = 0;

}

// ISR for DMA4 interrupt, currently DMAR for channel 2
void __attribute__((interrupt, no_auto_psv)) _DMA4Interrupt(void) {

    // Call registered callback function
    int_handler_ch2[port_cs_line[1]](SPIC_TRANS_SUCCESS);
    _DMA4IF = 0;

}

// ISR for DMA5 interrupt, currently DMAW for channel 2
// Currently not used, though it may be useful for debugging
void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void) {

    _DMA5IF = 0;

}

static void setupDMASet1 (void)
{
    DMA2CON = DMA2_REGISTER_POST_INCREMENT & // Increment address after each byte
              DMA2_ONE_SHOT &                // Stop module after transfer complete
              PERIPHERAL_TO_DMA2 &           // Receive data from peripheral to memory
              DMA2_SIZE_BYTE &               // Byte-size transactions
              DMA2_INTERRUPT_BLOCK &         // Interrupt after entire transaction
              DMA2_NORMAL &                  //
              DMA2_MODULE_OFF;               // Start module disabled

    DMA2REQ = SPI1_REQ_VAL;
    DMA2STA = __builtin_dmaoffset(spic1_rx_buff);
    DMA2STB = __builtin_dmaoffset(spic1_rx_buff);
    DMA2PAD = (volatile unsigned int) &SPI1BUF;
    DMA2CNT = 0;    // Default

    // Need this to avoid compiler bitlength issues
    unsigned long priority = DMA2_INT_PRI_6;
    SetPriorityIntDMA2(priority);

    EnableIntDMA2;
    _DMA2IF  = 0;   // Clear DMA interrupt flag

    DMA3CON = DMA3_REGISTER_POST_INCREMENT & // Increment address after each byte
              DMA3_ONE_SHOT &                // Stop module after transfer complete
              DMA3_TO_PERIPHERAL &           // Send data to peripheral from memory
              DMA3_SIZE_BYTE &               // Byte-size transaction
              DMA3_INTERRUPT_BLOCK &         // Interrupt after entire transaction
              DMA3_NORMAL &                  //
              DMA3_MODULE_OFF;               // Start module disabled

    DMA3REQ = SPI1_REQ_VAL;
    DMA3STA = __builtin_dmaoffset(spic1_tx_buff);
    DMA3STB = __builtin_dmaoffset(spic1_tx_buff);
    DMA3PAD = (volatile unsigned int) &SPI1BUF;
    DMA3CNT = 0;    // Default

    priority = DMA3_INT_PRI_6;
    SetPriorityIntDMA3(priority);
    DisableIntDMA3; // Only need one of the DMA interrupts
    _DMA3IF  = 0;   // Clear DMA interrupt
}

static void setupDMASet2 (void)
{
    DMA4CON = DMA4_REGISTER_POST_INCREMENT & // Increment address after ea/byte
              DMA4_ONE_SHOT &                // Stop module after transfer done
              PERIPHERAL_TO_DMA4 &           // Receive data from perip to mem
              DMA4_SIZE_BYTE &               // Byte-size transactions
              DMA4_INTERRUPT_BLOCK &         // Interrupt after transaction
              DMA4_NORMAL &                  //
              DMA4_MODULE_OFF;               // Start module disabled

    DMA4REQ = SPI2_REQ_VAL;
    DMA4STA = __builtin_dmaoffset(spic2_rx_buff);
    DMA4STB = __builtin_dmaoffset(spic2_rx_buff);
    DMA4PAD = (volatile unsigned int) &SPI2BUF;
    DMA4CNT = 0;    // Default

    // Need this to avoid compiler bitlength issues
    unsigned long priority = DMA4_INT_PRI_6;
    SetPriorityIntDMA4(priority);

    EnableIntDMA4;
    _DMA4IF  = 0;   // Clear DMA interrupt flag

    DMA5CON = DMA5_REGISTER_POST_INCREMENT & // Increment address after ea/byte
              DMA5_ONE_SHOT &                // Stop module after transfer done
              DMA5_TO_PERIPHERAL &           // Send data to periph from mem
              DMA5_SIZE_BYTE &               // Byte-size transaction
              DMA5_INTERRUPT_BLOCK &         // Interrupt after transaction
              DMA5_NORMAL &                  //
              DMA5_MODULE_OFF;               // Start module disabled

    DMA5REQ = SPI2_REQ_VAL;
    DMA5STA = __builtin_dmaoffset(spic2_tx_buff);
    DMA5STB = __builtin_dmaoffset(spic2_tx_buff);
    DMA5PAD = (volatile unsigned int) &SPI2BUF;
    DMA5CNT = 0;    // Default

    priority = DMA5_INT_PRI_6;
    SetPriorityIntDMA5(priority);
    DisableIntDMA5; // Only need one of the DMA interrupts
    _DMA5IF  = 0;   // Clear DMA interrupt
}
