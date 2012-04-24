/*
 * Copyright (c) 2008-2011, Regents of the University of California
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
 * ATMEL DataFlash Memory (dfmem) Interface
 *
 * by Fernando L. Garcia Bermudez
 *
 * v.beta
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez 2008-7-23   Initial release
 *                              2010-7-19   Blocking read/writes tested
 *  Stanley S. Baek             2010-8-30   Added buffer read/writes and sector
 *                                          erase for improving writing speeds.
 *  Andrew Pullin               2011-6-7    Added ability to query for chip
 *  w/Fernando L. Garcia Bermudez           size and flags to handle them.
 *  Andrew Pullin               2011-9-23   Added ability for deep power-down.
 *  Humphrey  Hu                2012-1-22   Enabled DMA on SPI port
 *
 * Notes:
 *  - Uses an SPI port for communicating with the memory chip and DMA
 *    channels 4 and 5 with spi_controller.c
 */

 // TODO (humhu) : Divide into generic nvmem (non-volatile memory) device class interface and
 //                DFMEM-specific driver to match radio class/driver
 // TODO (humhu) : Use a better non-ghetto mutex
 // TODO (humhu) : Add a rudimentary filesystem
 // TODO (humhu) : Add defines to switch between DMA/bitbang modes

#include <stdlib.h>
#include "p33Fxxxx.h"
#include "spi.h"
#include "dfmem.h"
#include "spi_controller.h"        // For DMA
#include "utils.h"

// TODO (humhu) : Consolidate into some BSP header
#if (defined(__IMAGEPROC1) || defined(__IMAGEPROC2) || defined(__MIKRO) || defined(__EXP16DEV))
// MIKRO & EXP16DEV has no FLASHMEM, but needs this for compile

    // SPIx pins
    #define SPI_CS          _LATG9

    // SPIx Registers
    #define SPI_BUF         SPI2BUF
    #define SPI_CON1        SPI2CON1
    #define SPI_CON2        SPI2CON2
    #define SPI_STAT        SPI2STAT
    #define SPI_STATbits    SPI2STATbits

#endif

// TODO (humhu) : Consolidate into BSP header
// Handle different chip sizes
#if (defined(__DFMEM_8MBIT))
    #define BYTE_ADDRESS_BITS   9
#elif (defined(__DFMEM_16MBIT) || defined(__DFMEM_32MBIT))
    #define BYTE_ADDRESS_BITS   10
#elif (defined(__DFMEM_64MBIT))
    #define BYTE_ADDRESS_BITS   11
#else
    #error "You need to specify the size of your memory chip by defining one of the following flags: __DFMEM_8MBIT, __DFMEM_16MBIT, or __DFMEM_32MBIT."
#endif

// Commands
#define WRITE_PAGE_VIA_BUFFER1              0x82
#define WRITE_PAGE_VIA_BUFFER2              0x85
#define WRITE_TO_BUFFER1                    0x84
#define WRITE_TO_BUFFER2                    0x87
#define WRITE_BUFFER1_TO_PAGE_NO_ERASE      0x88
#define WRITE_BUFFER2_TO_PAGE_NO_ERASE      0x89
#define WRITE_BUFFER1_TO_PAGE_WITH_ERASE    0x83
#define WRITE_BUFFER2_TO_PAGE_WITH_ERASE    0x86

#define READ_PAGE                           0xD2
#define READ_PAGE_TO_BUFFER_1               0x53
#define READ_PAGE_TO_BUFFER_2               0x55

#define ERASE_PAGE      0x81
#define ERASE_BLOCK     0x50
#define ERASE_SECTOR    0x7C

// Ghetto mutex constants
#define MUTEX_LOCKED    (0x01)
#define MUTEX_FREE        (0x00)

/*-----------------------------------------------------------------------------
 *          Private variables
-----------------------------------------------------------------------------*/

union {
    unsigned long address;
    unsigned char chr_addr[4];
} MemAddr;

static unsigned char mutex;    // Ghetto mutex

/*----------------------------------------------------------------------------
 *          Declaration of private functions
 ---------------------------------------------------------------------------*/

static inline unsigned char dfmemExchangeByte (unsigned char byte);
static inline void dfmemWriteByte (unsigned char byte);
static inline unsigned char dfmemReadByte (void);
static inline void dfmemSelectChip(void);
static inline void dfmemDeselectChip(void);
static void dfmemSetupPeripheral(void);

static void spiCallback(unsigned int irq_source);
static unsigned char checkMutex(void);
static void setMutex(unsigned char);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void dfmemSetup(void)
{
    dfmemSetupPeripheral();
    dfmemDeselectChip();

    spic2SetCallback(&spiCallback);

}

void dfmemWrite (unsigned char *data, unsigned int length, unsigned int page,
        unsigned int byte, unsigned char buffer)
{
    unsigned char command;

    while(!dfmemIsReady());

    // Choose command dependent on buffer choice
    if (buffer == 1) {
        command = WRITE_PAGE_VIA_BUFFER1;
    } else {
        command = WRITE_PAGE_VIA_BUFFER2;
    }

    // Restructure page/byte addressing
    // 1 don't care bit + 13 page address bits + byte address bits
    MemAddr.address = (((unsigned long)page) << BYTE_ADDRESS_BITS) + byte;


    // Write data to memory
    dfmemSelectChip();
    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    setMutex(MUTEX_LOCKED);

    // TODO (humhu) : Abstract this line into something like dfmemMassTransfer?
    spic2MassTransmit(length, data, 2*length);
    // Wait until transmit finishes?
    while(checkMutex() != MUTEX_FREE);

    dfmemDeselectChip();
}

void dfmemWriteBuffer (unsigned char *data, unsigned int length,
        unsigned int byte, unsigned char buffer)
{
    unsigned char command;

    // Choose command dependent on buffer choice
    if (buffer == 1) {
        command = WRITE_TO_BUFFER1;
    } else {
        command = WRITE_TO_BUFFER2;
    }

    // Restructure page/byte addressing
    // 14 don't care bit + byte address bits
    MemAddr.address = (unsigned long)byte;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    setMutex(MUTEX_LOCKED);

    spic2MassTransmit(length, data, 2*length);

    while(checkMutex() != MUTEX_FREE);

    dfmemDeselectChip();
}

void dfmemWriteBuffer2MemoryNoErase (unsigned int page, unsigned char buffer)
{
    unsigned char command;

    while(!dfmemIsReady());

    // Choose command dependent on buffer choice
    if (buffer == 1) {
        command = WRITE_BUFFER1_TO_PAGE_NO_ERASE;
    } else {
        command = WRITE_BUFFER2_TO_PAGE_NO_ERASE;
    }

    // Restructure page/byte addressing
    // 1 don't care bit + 13 page address bits + don't care bits
    MemAddr.address = ((unsigned long)page) << BYTE_ADDRESS_BITS;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    dfmemDeselectChip();
}

void dfmemPush (unsigned char *data, unsigned int length, unsigned int page_reset)
{
    static unsigned int page = 0;
    static unsigned int byte = 0;
    static unsigned char buffer = 0;

    if (page_reset != 0xffff) {
        page = page_reset;
    }

    if (length > 512 || length == 0) return;

    if (length + byte > 512) {
        dfmemWriteBuffer2MemoryNoErase(page++, buffer);
        buffer ^= 0x01; // toggle buffer
        byte = 0;
    }

    dfmemWriteBuffer(data, length, byte, buffer);
    byte += length;

}

void dfmemRead (unsigned int page, unsigned int byte, unsigned int length,
        unsigned char *data)
{
    while(!dfmemIsReady());

    // Restructure page/byte addressing
    // 1 don't care bit + 13 page address bits + byte address bits
    MemAddr.address = (((unsigned long)page) << BYTE_ADDRESS_BITS) + byte;

    // Read data from memory
    dfmemSelectChip();

    dfmemWriteByte(READ_PAGE);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    dfmemWriteByte(0x00); // 4 don't care bytes
    dfmemWriteByte(0x00);
    dfmemWriteByte(0x00);
    dfmemWriteByte(0x00);

    setMutex(MUTEX_LOCKED);

    unsigned int read_bytes;
    read_bytes = spic2MassTransmit(length, NULL, 2*length);

    while(checkMutex() != MUTEX_FREE);

    spic2ReadBuffer(read_bytes, data);

    dfmemDeselectChip();
}

void dfmemReadPage2Buffer (unsigned int page, unsigned char buffer)
{
    unsigned char command;

    while(!dfmemIsReady());

    // Choose command dependent on buffer choice
    if (buffer == 1) {
        command = READ_PAGE_TO_BUFFER_1;
    } else {
        command = READ_PAGE_TO_BUFFER_2;
    }

    // 1 don't care bit + 13 page address bits + don't care bits
    MemAddr.address = ((unsigned long)page) << BYTE_ADDRESS_BITS;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    dfmemDeselectChip();
}

void dfmemErasePage (unsigned int page)
{
    while(!dfmemIsReady());

    // Restructure page/byte addressing
    MemAddr.address = ((unsigned long)page) << BYTE_ADDRESS_BITS;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(ERASE_PAGE);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    dfmemDeselectChip();
}

void dfmemEraseBlock (unsigned int page)
{
    while(!dfmemIsReady());

    // Restructure page/byte addressing
    MemAddr.address = ((unsigned long)page) << BYTE_ADDRESS_BITS;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(ERASE_BLOCK);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    dfmemDeselectChip();
}

void dfmemEraseSector (unsigned int page)
{
    while(!dfmemIsReady());

    // Restructure page/byte addressing
    MemAddr.address = ((unsigned long)page) << BYTE_ADDRESS_BITS;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(ERASE_SECTOR);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    dfmemDeselectChip();
}

void dfmemEraseChip (void)
{
    while(!dfmemIsReady());

    dfmemSelectChip();

    dfmemWriteByte(0xC7);
    dfmemWriteByte(0x94);
    dfmemWriteByte(0x80);
    dfmemWriteByte(0x9A);

    dfmemDeselectChip();
}

unsigned char dfmemIsReady (void)
{
    return (dfmemGetStatus() >> 7);
}

unsigned char dfmemGetStatus (void)
{
    unsigned char byte;

    dfmemSelectChip();

    dfmemWriteByte(0xD7);
    byte = dfmemReadByte();

    dfmemDeselectChip();

    return byte;
}

// The manufacturer and device id command (0x9F) returns 4 bytes normally
// (including info on id, family, density, etc.), but this functions returns
// just the manufacturer id and discards the rest when deselecting the chip.
unsigned char dfmemGetManufacturerID (void)
{
    unsigned char byte;

    dfmemSelectChip();

    dfmemWriteByte(0x9F);
    byte = dfmemReadByte();

    dfmemDeselectChip();

    return byte;
}

// The manufacturer and device id command (0x9F) returns 4 bytes normally
// (including info on id, family, density, etc.), but this functions returns
// only the 5 bits pertaining to the memory density.
unsigned char dfmemGetChipSize (void)
{
    unsigned char byte;

    dfmemSelectChip();

    dfmemWriteByte(0x9F);
    byte = dfmemReadByte(); // Manufacturer ID, not needed
    byte = dfmemReadByte() & 0b00011111;

    dfmemDeselectChip();

    return byte;
}

void dfmemDeepSleep()
{
    dfmemSelectChip();

    dfmemWriteByte(0xB9);

    dfmemDeselectChip();
}

void dfmemResumeFromDeepSleep()
{
    dfmemSelectChip();

    dfmemWriteByte(0xAB);

    dfmemDeselectChip();
}


/*-----------------------------------------------------------------------------
 *          Private functions
-----------------------------------------------------------------------------*/

void spiCallback(unsigned int irq_source) {

    if(irq_source == SPIC_TRANS_SUCCESS) {

        spic2EndTransaction();
        setMutex(MUTEX_FREE);    // Unblock anything waiting on transfer

    } else if(irq_source == SPIC_TRANS_TIMEOUT) {

        spic2Reset();   // Reset hardware?

    }

}

static unsigned char checkMutex(void) {

    unsigned char stat;
    CRITICAL_SECTION_START;
    stat = mutex;
    CRITICAL_SECTION_END;
    return stat;

}

static void setMutex(unsigned char stat) {

    CRITICAL_SECTION_START;
    mutex = stat;
    CRITICAL_SECTION_END;

}

// Sends a byte to the memory chip and returns the byte read from it
//
// Parameters   :   byte to send.
// Returns      :   received byte.
static inline unsigned char dfmemExchangeByte (unsigned char byte)
{
    SPI_BUF = byte;
    while(SPI_STATbits.SPITBF);
    while(!SPI_STATbits.SPIRBF);
    SPI_STATbits.SPIROV = 0;
    return SPI_BUF;
}

// Sends a byte to the memory chip.
//
// It discards the byte it receives when transmitting this one as it should
// not be important and so that it doesn't stay in the received queue.
//
// Parameters : byte to send.
static inline void dfmemWriteByte (unsigned char byte)
{
    spic2Transmit(byte);
}

// Receives a byte from the memory chip.
//
// It sends a null byte so as to issue the required clock cycles for receiving
// one from the memory.
//
// Returns : received byte.
static inline unsigned char dfmemReadByte (void)
{
    return spic2Receive();
}

// Selects the memory chip.
//static inline void dfmemSelectChip(void) { SPI_CS = 0; }

static inline void dfmemSelectChip(void) {
    spic2BeginTransaction();
}

// Deselects the memory chip.
//static inline void dfmemDeselectChip(void) { SPI_CS = 1; }

static inline void dfmemDeselectChip(void) {
    spic2EndTransaction();
}


// Initializes the SPIx bus for communicating with the memory.
//
// The MCU is the SPI master and the clock isn't continuous.
static void dfmemSetupPeripheral(void)
{
    SPI_CON1 = ENABLE_SCK_PIN & ENABLE_SDO_PIN & SPI_MODE16_OFF & SPI_SMP_OFF &
               SPI_CKE_ON & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH &
               MASTER_ENABLE_ON & PRI_PRESCAL_1_1 & SEC_PRESCAL_4_1;
    SPI_CON2 = FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT & FRAME_POL_ACTIVE_HIGH &
               FRAME_SYNC_EDGE_PRECEDE;
    SPI_STAT = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;
}
