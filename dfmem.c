/*
 * Copyright (c) 2008-2013, Regents of the University of California
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
 * v.1.0
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez 2008-7-23   Initial release.
 *                              2010-7-19   Blocking read/writes tested.
 *  Stanley S. Baek             2010-8-30   Added buffer read/writes and sector
 *                                          erase for improving writing speeds.
 *  Andrew Pullin               2011-6-7    Added ability to query for chip
 *  w/Fernando L. Garcia Bermudez           size and flags to handle them.
 *  Andrew Pullin               2011-9-23   Added ability for deep power-down.
 *  Humphrey  Hu                2012-1-22   Enabled DMA on SPI port.
 *  Andrew Pullin               2012-4-8    Adding auto flash geometry and
 *                                          some telemetry helper functions.
 *
 * Notes:
 *  - Uses an SPI port for communicating with the memory chip and DMA
 *    channels 4 and 5 with spi_controller.c
 */

 // TODO (humhu) : Divide into generic nvmem (non-volatile memory) device class interface and
 //                DFMEM-specific driver to match radio class/driver
 // TODO (humhu) : Add a rudimentary filesystem

#include <stdlib.h>
#include <string.h>

#include <xc.h>
#include "spi.h"
#include "dfmem.h"
#include "spi_controller.h"        // For DMA
#include "utils.h"

// Flash geometry
// 8 Mbit
#define FLASH_8MBIT_MAX_SECTOR              16
#define FLASH_8MBIT_MAX_PAGES               4096
#define FLASH_8MBIT_BUFFERSIZE              264
#define FLASH_8MBIT_BYTES_PER_PAGE          264
#define FLASH_8MBIT_PAGES_PER_BLOCK         8
#define FLASH_8MBIT_BLOCKS_PER_SECTOR       32
#define FLASH_8MBIT_PAGES_PER_SECTOR        256  //Calculated; not directly in datasheet
#define FLASH_8MBIT_BYTE_ADDRESS_BITS       9

// 16 Mbit
#define FLASH_16MBIT_MAX_SECTOR             16
#define FLASH_16MBIT_MAX_PAGES              4096
#define FLASH_16MBIT_BUFFERSIZE             528
#define FLASH_16MBIT_BYTES_PER_PAGE         528
#define FLASH_16MBIT_PAGES_PER_BLOCK        8
#define FLASH_16MBIT_BLOCKS_PER_SECTOR      32
#define FLASH_16MBIT_PAGES_PER_SECTOR       256  //Calculated; not directly in datasheet
#define FLASH_16MBIT_BYTE_ADDRESS_BITS      10

// 32 Mbit
#define FLASH_32MBIT_MAX_SECTOR             64
#define FLASH_32MBIT_MAX_PAGES              8192
#define FLASH_32MBIT_BUFFERSIZE             528
#define FLASH_32MBIT_BYTES_PER_PAGE         528
#define FLASH_32MBIT_PAGES_PER_BLOCK        8
#define FLASH_32MBIT_BLOCKS_PER_SECTOR      16   // --> THIS VALUE IS WRONG IN THE DATASHEET! 16 IS CORRECT.
#define FLASH_32MBIT_PAGES_PER_SECTOR       128  //Calculated; not directly in datasheet
#define FLASH_32MBIT_BYTE_ADDRESS_BITS      10

// 64 Mbit
#define FLASH_64MBIT_MAX_SECTOR             32
#define FLASH_64MBIT_MAX_PAGES              8192
#define FLASH_64MBIT_BUFFERSIZE             1056
#define FLASH_64MBIT_BYTES_PER_PAGE         1056
#define FLASH_64MBIT_PAGES_PER_BLOCK        8
#define FLASH_64MBIT_BLOCKS_PER_SECTOR      32
#define FLASH_64MBIT_PAGES_PER_SECTOR       256  //Calculated; not directly in datasheet
#define FLASH_64MBIT_BYTE_ADDRESS_BITS      11

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

/*-----------------------------------------------------------------------------
 *          Private variables
-----------------------------------------------------------------------------*/

// Memory geometry
static DfmemGeometryStruct dfmem_geo;


// Placeholders
static unsigned int currentBuffer = 0;
static unsigned int currentBufferOffset = 0;
static unsigned int nextPage = 0;

enum FlashSizeType {
    DFMEM_8MBIT    = 0b00101,
    DFMEM_16MBIT   = 0b00110,
    DFMEM_32MBIT   = 0b00111,
    DFMEM_64MBIT   = 0b01000
 };

union {
    unsigned long address;
    unsigned char chr_addr[4];
} MemAddr;

unsigned char userSecRegID[64];
unsigned char factorySecRegID[64];

/*----------------------------------------------------------------------------
 *          Declaration of private functions
 ---------------------------------------------------------------------------*/

static inline void dfmemWriteByte (unsigned char byte);
static inline unsigned char dfmemReadByte (void);
static inline void dfmemSelectChip(void);
static inline void dfmemDeselectChip(void);
static void dfmemSetupPeripheral(void);
static void dfmemGeometrySetup(void);
static void dfmemReadSecurityRegister(void);

static void spiCallback(unsigned int irq_source);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void dfmemSetup(void)
{
    dfmemSetupPeripheral();
    spic2SetCallback(DFMEM_CS, &spiCallback);
    while(!dfmemIsReady());

    dfmemGeometrySetup();

    //Readback security register, which gets chip-unique identifiers
    // ID is stored in a static var here, and dfmem has a public getter
    dfmemReadSecurityRegister();
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
    MemAddr.address = (((unsigned long)page) << dfmem_geo.byte_address_bits) + byte;

    // Write data to memory
    dfmemSelectChip();
    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    spic2MassTransmit(length, data, 2*length);
    //while (length--) { dfmemWriteByte(*data++); }
    //dfmemDeselectChip();
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

    spic2MassTransmit(length, data, 2*length);
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
    MemAddr.address = ((unsigned long)page) << dfmem_geo.byte_address_bits;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    currentBufferOffset = 0;

    dfmemDeselectChip();
}

void dfmemRead (unsigned int page, unsigned int byte, unsigned int length,
        unsigned char *data)
{
    while(!dfmemIsReady());

    // Restructure page/byte addressing
    // 1 don't care bit + 13 page address bits + byte address bits
    MemAddr.address = (((unsigned long)page) << dfmem_geo.byte_address_bits) + byte;

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

    unsigned int read_bytes;
    read_bytes = spic2MassTransmit(length, NULL, 2*length);
    dfmemSelectChip(); // Busy wait
    spic2ReadBuffer(read_bytes, data);  // reads DMA buffer
    //while (length--) { *data++ = dfmemReadByte(); }

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
    MemAddr.address = ((unsigned long)page) << dfmem_geo.byte_address_bits;

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
    MemAddr.address = ((unsigned long)page) << dfmem_geo.byte_address_bits;

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
    MemAddr.address = ((unsigned long)page) << dfmem_geo.byte_address_bits;

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
    MemAddr.address = ((unsigned long)page) << dfmem_geo.byte_address_bits;

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
// just the manufacturer id and discards the rest when de-selecting the chip.
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

void dfmemSave(unsigned char* data, unsigned int length)
{   //If this write will not fit into the buffer, then
       if (currentBufferOffset + length > dfmem_geo.buffer_size)
       { dfmemSync(); //  i) write current buffer to memory, and  ii) switch to new buffer
       }
 /*       dfmemWriteBuffer2MemoryNoErase(nextPage, currentBuffer);
        currentBuffer = (currentBuffer) ? 0 : 1; // toggle buffer
        currentBufferOffset = 0;  // reset to beginning
        nextPage++;
*/
    //  write data into buffer
    dfmemWriteBuffer(data, length, currentBufferOffset, currentBuffer);
    currentBufferOffset += length;
}

// write last buffer to memory
void dfmemSync()
{
    //if currentBufferOffset == 0, then we don't need to write anything to be sync'd
    if(currentBufferOffset != 0){
        dfmemWriteBuffer2MemoryNoErase(nextPage, currentBuffer);
        currentBuffer = (currentBuffer) ? 0 : 1; //Toggle buffer number
        currentBufferOffset = 0; // reset to beginning of buffer
        nextPage++;
    }
}

void dfmemGetGeometryParams(DfmemGeometry geo)
{
    if(geo == NULL) { return; }

    memcpy(geo, &dfmem_geo, sizeof(DfmemGeometryStruct));
}

void dfmemZeroIndex()
{
    currentBuffer = 0;
    currentBufferOffset = 0;
    nextPage = 0;
}

uint64_t dfmemGetUnqiueID(){
    //Grab first 8 bytes of 64-byte factory value
    uint64_t id = *((uint64_t*)(factorySecRegID));
    return id;
}

/*-----------------------------------------------------------------------------
 *          Private functions
-----------------------------------------------------------------------------*/

void spiCallback(unsigned int irq_source) {

    if(irq_source == SPIC_TRANS_SUCCESS) {

        dfmemDeselectChip();

    } else if(irq_source == SPIC_TRANS_TIMEOUT) {

        spic2Reset();   // Reset hardware

    }

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
static inline void dfmemSelectChip(void) { spic2BeginTransaction(DFMEM_CS); }

// De-selects the memory chip.
static inline void dfmemDeselectChip(void) { spic2EndTransaction(); }

// Initializes the SPIx bus for communicating with the memory.
//
// The MCU is the SPI master and the clock isn't continuous.
static void dfmemSetupPeripheral(void)
{
    spicSetupChannel2(DFMEM_CS,
                      ENABLE_SCK_PIN &
                      ENABLE_SDO_PIN &
                      SPI_MODE16_OFF &
                      SPI_SMP_OFF &
                      SPI_CKE_ON &
                      SLAVE_ENABLE_OFF &
                      CLK_POL_ACTIVE_HIGH &
                      MASTER_ENABLE_ON &
                      PRI_PRESCAL_1_1 &
                      SEC_PRESCAL_4_1);
}

// Figures out memory geometry by querying its size
static void dfmemGeometrySetup(void)
{
    unsigned char size;
    size = dfmemGetChipSize();

    switch(size){
        case DFMEM_8MBIT:
            dfmem_geo.max_sector        = FLASH_8MBIT_MAX_SECTOR;
            dfmem_geo.max_pages         = FLASH_8MBIT_MAX_PAGES;
            dfmem_geo.buffer_size        = FLASH_8MBIT_BUFFERSIZE;
            dfmem_geo.bytes_per_page    = FLASH_8MBIT_BYTES_PER_PAGE;
            dfmem_geo.pages_per_block   = FLASH_8MBIT_PAGES_PER_BLOCK;
            dfmem_geo.blocks_per_sector = FLASH_8MBIT_BLOCKS_PER_SECTOR;
            dfmem_geo.pages_per_sector  = FLASH_8MBIT_PAGES_PER_SECTOR;
            dfmem_geo.byte_address_bits = FLASH_8MBIT_BYTE_ADDRESS_BITS;
            break;
        case DFMEM_16MBIT:
            dfmem_geo.max_sector        = FLASH_16MBIT_MAX_SECTOR;
            dfmem_geo.max_pages         = FLASH_16MBIT_MAX_PAGES;
            dfmem_geo.buffer_size        = FLASH_16MBIT_BUFFERSIZE;
            dfmem_geo.bytes_per_page    = FLASH_16MBIT_BYTES_PER_PAGE;
            dfmem_geo.pages_per_block   = FLASH_16MBIT_PAGES_PER_BLOCK;
            dfmem_geo.blocks_per_sector = FLASH_16MBIT_BLOCKS_PER_SECTOR;
            dfmem_geo.pages_per_sector  = FLASH_16MBIT_PAGES_PER_SECTOR;
            dfmem_geo.byte_address_bits = FLASH_16MBIT_BYTE_ADDRESS_BITS;
            break;
        case DFMEM_32MBIT:
            dfmem_geo.max_sector        = FLASH_32MBIT_MAX_SECTOR;
            dfmem_geo.max_pages         = FLASH_32MBIT_MAX_PAGES;
            dfmem_geo.buffer_size        = FLASH_32MBIT_BUFFERSIZE;
            dfmem_geo.bytes_per_page    = FLASH_32MBIT_BYTES_PER_PAGE;
            dfmem_geo.pages_per_block   = FLASH_32MBIT_PAGES_PER_BLOCK;
            dfmem_geo.blocks_per_sector = FLASH_32MBIT_BLOCKS_PER_SECTOR;
            dfmem_geo.pages_per_sector  = FLASH_32MBIT_PAGES_PER_SECTOR;
            dfmem_geo.byte_address_bits = FLASH_32MBIT_BYTE_ADDRESS_BITS;
            break;
        case DFMEM_64MBIT:
            dfmem_geo.max_sector        = FLASH_64MBIT_MAX_SECTOR;
            dfmem_geo.max_pages         = FLASH_64MBIT_MAX_PAGES;
            dfmem_geo.buffer_size        = FLASH_64MBIT_BUFFERSIZE;
            dfmem_geo.bytes_per_page    = FLASH_64MBIT_BYTES_PER_PAGE;
            dfmem_geo.pages_per_block   = FLASH_64MBIT_PAGES_PER_BLOCK;
            dfmem_geo.blocks_per_sector = FLASH_64MBIT_BLOCKS_PER_SECTOR;
            dfmem_geo.pages_per_sector  = FLASH_64MBIT_PAGES_PER_SECTOR;
            dfmem_geo.byte_address_bits = FLASH_64MBIT_BYTE_ADDRESS_BITS;
            break;
        default:
            // TODO (apullin, fgb) : Do something. Probably communicate back with user.
            break;
    }
}

static void dfmemReadSecurityRegister(void){

    int i; // Loop variable, MPLABX is not C99 standard

    dfmemSelectChip();

    dfmemWriteByte(0x77); //Read security register opcode
    dfmemWriteByte(0x77); //Dummy write, 3 bytes required for opcode 0x77
    dfmemWriteByte(0x77); //Dummy write, 3 bytes required for opcode 0x77
    dfmemWriteByte(0x77); //Dummy write, 3 bytes required for opcode 0x77

    //User ID is the first 64 bytes
    for(i = 0; i < 64; i++){
        userSecRegID[i] = dfmemReadByte();
    }

    //Factory ID is the second 64 bytes
    for(i = 0; i < 64; i++){
        factorySecRegID[i] = dfmemReadByte();
    }
    dfmemDeselectChip();

}
