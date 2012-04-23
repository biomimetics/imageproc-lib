/*
 * Copyright (c) 2008-2012, Regents of the University of California
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
 * v.1.0 beta
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez 2008-7-23   Initial release
 *                              2010-7-19   Blocking read/writes tested
 *  Stanley S. Baek             2010-8-30   Added buffer read/writes and sector
 *                                          erase for improving writing speeds.
 *  Andrew Pullin               2011-6-7    Added ability to query for chip
 *  w/Fernando L. Garcia Bermudez           size and flags to handle them.
 *  Andrew Pullin               2011-9-23   Added ability for deep power-down.
 *                              2012-4-8    Adding auto flash geometry and
 *                                          some telemetry helper functions.
 *
 * Notes:
 *  - Uses an SPI port for communicating with the memory chip.
 */

#include "p33Fxxxx.h"
#include "spi.h"
#include "dfmem.h"
#include "utils.h"


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
static unsigned int dfmem_byte_address_bits;
static unsigned int dfmem_max_sector;
static unsigned int dfmem_max_pages;
static unsigned int dfmem_buffersize;
static unsigned int dfmem_bytes_per_page;
static unsigned int dfmem_pages_per_block;
static unsigned int dfmem_blocks_per_sector;
static unsigned int dfmem_pages_per_sector;

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


/*----------------------------------------------------------------------------
 *          Declaration of private functions
 ---------------------------------------------------------------------------*/

static inline unsigned char dfmemExchangeByte (unsigned char byte);
static inline void dfmemWriteByte (unsigned char byte);
static inline unsigned char dfmemReadByte (void);
static inline void dfmemSelectChip(void);
static inline void dfmemDeselectChip(void);
static void dfmemSetupPeripheral(void);
static void dfmemGeometrySetup(void);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void dfmemSetup(void)
{
    dfmemSetupPeripheral();
    dfmemDeselectChip();
    dfmemGeometrySetup();
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
    MemAddr.address = (((unsigned long)page) << dfmem_byte_address_bits) + byte;

    // Write data to memory
    dfmemSelectChip();
    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    while (length--) { dfmemWriteByte(*data++); }
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

    while (length--) { dfmemWriteByte(*data++); }

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
    MemAddr.address = ((unsigned long)page) << dfmem_byte_address_bits;

    // Write data to memory
    dfmemSelectChip();

    dfmemWriteByte(command);
    dfmemWriteByte(MemAddr.chr_addr[2]);
    dfmemWriteByte(MemAddr.chr_addr[1]);
    dfmemWriteByte(MemAddr.chr_addr[0]);

    currentBufferOffset = 0;

    dfmemDeselectChip();
}

void dfmemPush (unsigned char *data, unsigned int length, unsigned int page_reset)
{
    /*
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
    */

}

void dfmemRead (unsigned int page, unsigned int byte, unsigned int length,
        unsigned char *data)
{
    while(!dfmemIsReady());

    // Restructure page/byte addressing
    // 1 don't care bit + 13 page address bits + byte address bits
    MemAddr.address = (((unsigned long)page) << dfmem_byte_address_bits) + byte;

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

    while (length--) { *data++ = dfmemReadByte(); }

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
    MemAddr.address = ((unsigned long)page) << dfmem_byte_address_bits;

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
    MemAddr.address = ((unsigned long)page) << dfmem_byte_address_bits;

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
    MemAddr.address = ((unsigned long)page) << dfmem_byte_address_bits;

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
    MemAddr.address = ((unsigned long)page) << dfmem_byte_address_bits;

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

void dfmemSave(unsigned char* data, unsigned int length)
{
    //If this write will fit into the buffer, then just put it there
    if (currentBufferOffset + length >= dfmem_buffersize) {
        dfmemWriteBuffer2MemoryNoErase(nextPage, currentBuffer);
        currentBuffer = (currentBuffer) ? 0 : 1;
        currentBufferOffset = 0;
        nextPage++;
    }

    //We know there won't be an overrun here because of the previous 'if'
    // TODO (fgb) : Shouldn't this happen only when the buffer is full,
    //              probably calling dfmemSync?
    dfmemWriteBuffer(data, length, currentBufferOffset, currentBuffer);
    currentBufferOffset += length;
}

void dfmemSync()
{
    while(!dfmemIsReady());

    //if currentBufferOffset == 0, then we don't need to write anything to be sync'd
    if(currentBufferOffset != 0){
        dfmemWriteBuffer2MemoryNoErase(nextPage, currentBuffer);
        currentBuffer = (currentBuffer) ? 0 : 1; //Toggle buffer number
        currentBufferOffset = 0;
        nextPage++;
    }
}

void dfmemReadSample(unsigned long sampNum, unsigned int sampLen, unsigned char *data)
{
    unsigned int samplesPerPage = dfmem_bytes_per_page / sampLen; //round DOWN int division
    unsigned int pagenum = sampNum / samplesPerPage;
    unsigned int byteOffset = (sampNum - pagenum*samplesPerPage)*sampLen;

    dfmemRead(pagenum, byteOffset, sampLen, data);
}

void dfmemEraseSectorsForSamples(unsigned long numSamples, unsigned int sampLen)
{
    // TODO (apullin) : Add an explicit check to see if the number of saved
    //                  samples will fit into memory!
    LED_2 = 1;
    unsigned int firstPageOfSector, i;

    //avoid trivial case
    if(numSamples == 0){ return;}

    //Saves to dfmem will NOT overlap page boundaries, so we need to do this level by level:
    unsigned int samplesPerPage = dfmem_bytes_per_page / sampLen; //round DOWN int division
    unsigned int numPages = (numSamples + samplesPerPage - 1) / samplesPerPage; //round UP int division
    unsigned int numSectors = ( numPages + dfmem_pages_per_sector-1) / dfmem_pages_per_sector;

    //At this point, it is impossible for numSectors == 0
    //Sector 0a and 0b will be erased together always, for simplicity
    //Note that numSectors will be the actual number of sectors to erase,
    //   even though the sectors themselves are numbered starting at '0'
    dfmemEraseSector(0); //Erase Sector 0a
    dfmemEraseSector(8); //Erase Sector 0b

    //Start erasing the rest from Sector 1:
    for(i=1; i <= numSectors; i++){
        firstPageOfSector = dfmem_pages_per_sector * i;
        //hold off until dfmem is ready for secort erase command
        while(!dfmemIsReady());
        //LED should blink indicating progress
        LED_2 = ~LED_2;
        //Send actual erase command
        dfmemEraseSector(firstPageOfSector);
    }

    //Leadout flash, should blink faster than above, indicating the last sector
    while(!dfmemIsReady()){
        LED_2 = ~LED_2;
        delay_ms(75);
    }
    LED_2 = 0; //Green LED off

    //Since we've erased, reset our place keeper vars
    currentBuffer = 0;
    currentBufferOffset = 0;
    nextPage = 0;
}


/*-----------------------------------------------------------------------------
 *          Private functions
-----------------------------------------------------------------------------*/

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
    dfmemExchangeByte(byte);
}

// Receives a byte from the memory chip.
//
// It sends a null byte so as to issue the required clock cycles for receiving
// one from the memory.
//
// Returns : received byte.
static inline unsigned char dfmemReadByte (void)
{
    return dfmemExchangeByte(0x00);
}

// Selects the memory chip.
static inline void dfmemSelectChip(void) { SPI_CS = 0; }

// Deselects the memory chip.
static inline void dfmemDeselectChip(void) { SPI_CS = 1; }

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

// Figures out memory geometry by querying its size
static void dfmemGeometrySetup(void)
{
    unsigned char size;
    size = dfmemGetChipSize();

    switch(size){
        case DFMEM_8MBIT:
            dfmem_max_sector        = FLASH_8MBIT_MAX_SECTOR;
            dfmem_max_pages         = FLASH_8MBIT_MAX_PAGES;
            dfmem_buffersize        = FLASH_8MBIT_BUFFERSIZE;
            dfmem_bytes_per_page    = FLASH_8MBIT_BYTES_PER_PAGE;
            dfmem_pages_per_block   = FLASH_8MBIT_PAGES_PER_BLOCK;
            dfmem_blocks_per_sector = FLASH_8MBIT_BLOCKS_PER_SECTOR;
            dfmem_pages_per_sector  = FLASH_8MBIT_PAGES_PER_SECTOR;
            dfmem_byte_address_bits = FLASH_8MBIT_BYTE_ADDRESS_BITS;
            break;
        case DFMEM_16MBIT:
            dfmem_max_sector        = FLASH_16MBIT_MAX_SECTOR;
            dfmem_max_pages         = FLASH_16MBIT_MAX_PAGES;
            dfmem_buffersize        = FLASH_16MBIT_BUFFERSIZE;
            dfmem_bytes_per_page    = FLASH_16MBIT_BYTES_PER_PAGE;
            dfmem_pages_per_block   = FLASH_16MBIT_PAGES_PER_BLOCK;
            dfmem_blocks_per_sector = FLASH_16MBIT_BLOCKS_PER_SECTOR;
            dfmem_pages_per_sector  = FLASH_16MBIT_PAGES_PER_SECTOR;
            dfmem_byte_address_bits = FLASH_16MBIT_BYTE_ADDRESS_BITS;
            break;
        case DFMEM_32MBIT:
            dfmem_max_sector        = FLASH_32MBIT_MAX_SECTOR;
            dfmem_max_pages         = FLASH_32MBIT_MAX_PAGES;
            dfmem_buffersize        = FLASH_32MBIT_BUFFERSIZE;
            dfmem_bytes_per_page    = FLASH_32MBIT_BYTES_PER_PAGE;
            dfmem_pages_per_block   = FLASH_32MBIT_PAGES_PER_BLOCK;
            dfmem_blocks_per_sector = FLASH_32MBIT_BLOCKS_PER_SECTOR;
            dfmem_pages_per_sector  = FLASH_32MBIT_PAGES_PER_SECTOR;
            dfmem_byte_address_bits = FLASH_32MBIT_BYTE_ADDRESS_BITS;
            break;
        case DFMEM_64MBIT:
            dfmem_max_sector        = FLASH_64MBIT_MAX_SECTOR;
            dfmem_max_pages         = FLASH_64MBIT_MAX_PAGES;
            dfmem_buffersize        = FLASH_64MBIT_BUFFERSIZE;
            dfmem_bytes_per_page    = FLASH_64MBIT_BYTES_PER_PAGE;
            dfmem_pages_per_block   = FLASH_64MBIT_PAGES_PER_BLOCK;
            dfmem_blocks_per_sector = FLASH_64MBIT_BLOCKS_PER_SECTOR;
            dfmem_pages_per_sector  = FLASH_64MBIT_PAGES_PER_SECTOR;
            dfmem_byte_address_bits = FLASH_64MBIT_BYTE_ADDRESS_BITS;
            break;
        default:
            // TODO (apullin, fgb) : Do something. Probably communicate back with user.
            break;
    }
}
