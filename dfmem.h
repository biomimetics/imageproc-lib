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
 * Header for the ATMEL DataFlash Memory (dfmem) Interface
 *
 * by Fernando L. Garcia Bermudez
 *
 * v.1.0 beta
 *
 * Usage:
 *  #include "dfmem.h"
 *
 *  // Initialize memory interface
 *  dfmemSetup();
 *
 *  unsigned int page = 0, byte = 0, buffer = 1;
 *  unsigned char phrase[] = "Just what do you think you're doing, Dave?",
 *                itsaid[sizeof(phrase)];
 *
 *  // Send string to the memory
 *  dfmemWrite(phrase, sizeof(phrase), page, byte, buffer);
 *
 *  // Get same string back
 *  dfmemRead(page, byte, sizeof(phrase), itsaid);
 *
 *  // Both phrase and itsaid now contain the same string
 */

#ifndef __DFMEM_H
#define __DFMEM_H


// Handles initialization of communication peripherals and makes sure the
// memory is initially deselected.
void dfmemSetup (void);

// Writes the contents of a data array to memory.
//
// It's an implementation of the dfmem's "Main Memory Page Program Through
// Buffer" command.
//
// The pointer and length of a data array is passed along with the page where
// they will be written starting at a certain byte and utilizing one of the
// dfmem's internal buffers.
//
// Parameters : data = pointer to the input data array,
//              length = length of this data array (should be <= 528-byte),
//              page = 0-8191 (13 bits),
//              byte = 0-527 (10 bits),
//              buffer = 1 or 2.
void dfmemWrite (unsigned char *data, unsigned int length, unsigned int page,
        unsigned int byte, unsigned char buffer);

// Writes the contents of a data array to a memory buffer.
//
// It's an implementation of the dfmem's "Buffer Write" command.
//
// The pointer and length of a data array is passed along with the starting
// byte where they will be written in a specified dfmem's internal buffer.
//
// Parameters : data = pointer to the input data array,
//              length = length of this data array (should be <= 528-byte),
//              byte = 0-527 (10 bits),
//              buffer = 1 or 2.
void dfmemWriteBuffer (unsigned char *data, unsigned int length,
        unsigned int byte, unsigned char buffer);

// Writes the contents of a buffer to a memory page without pre-erasing it.
//
// It's an implementation of the dfmem's "Buffer to Main Memory Page Program
// without Built-in Erase" command.
//
// The page where the specified dfmem's internal buffer contents will be
// written is passed.
//
// Parameters : page = 0-8191 (13 bits),
//              buffer = 1 or 2.
void dfmemWriteBuffer2MemoryNoErase (unsigned int page, unsigned char buffer);

// Pushes the contents of a data array to a memory buffer. If the buffer is
// full (528 bytes), the contents of the buffer is written to a memory page
// without pre-erasing. After that, the memory page is incremented for the
// next operations. This function would be useful when you don't want to keep
// track of the current memory address to write. For the very first time you
// call this function set the page number using page_reset. After that, you
// can put -1(0xffff) for page_reset to use the internal page number.
//
// Parameters : data = pointer to the input data array,
//              length = length of this data array (should be <= 528 byte),
//              page_reset = reset the page number to write the data.
//                  If page_reset is -1(0xffff), page number will not be reset
// TODO (fgb) : Needs further debugging.
//void dfmemPush (unsigned char *data, unsigned int length, unsigned int page_reset);

// Read the contents of a memory page into a data array.
//
// It's an implementation of the dfmem's "Main Memory Page Read" command.
//
// The page and the starting byte of the memory to read is passed along with
// the length of data to be retrieved into the data array.
//
// Parameters : page = 0-8191 (13 bits),
//              byte = 0-527 (10 bits),
//              length = # of bytes to retrieve (should be <= 528-byte),
//              data = pointer to the output data array.
void dfmemRead (unsigned int page, unsigned int byte, unsigned int length,
        unsigned char *data);

// Read the contents of a memory page into a memory buffer.
//
// It's an implementation of the dfmem's "Main Memory Page to Buffer Transfer"
// command.
//
// The page and the specified dfmem's internal buffer are passed.
//
// Parameters : page = 0-8191 (13 bits),
//              buffer = 1 or 2.
//
//  TODO (fgb) : Needs further testing before release.
//void dfmemReadPage2Buffer (unsigned int page, unsigned char buffer)

// Erase the contents of a memory page.
//
// It's an implementation of the dfmem's "Page Erase" command.
//
// The page to be erased is passed.
//
// Parameters : page = 0-8191 (13 bits).
void dfmemErasePage(unsigned int page);

// Erase the contents of a memory block.
//
// It's an implementation of the dfmem's "Block Erase" command.
//
// One of the pages within the specified block is passed.
//
// Parameters : page = 0-8191 (13 bits).
void dfmemEraseBlock(unsigned int page);

// Erase the contents of a memory sector.
//
// It's an implementation of the dfmem's "Sector Erase" command.
//
// One of the pages within the specified sector is passed.
//
// Parameters : page = 0-8191 (13 bits).
void dfmemEraseSector(unsigned int page);

// Erase the contents of the whole memory chip.
//
// It's an implementation of the dfmem's "Chip Erase" command.
void dfmemEraseChip(void);

// Requests dfmem status register and returns its ready(RDY) bit.
//
// Returns : ready(RDY) bit
unsigned char dfmemIsReady (void);

// Requests dfmem status register and returns it.
//
// Returns : status register
unsigned char dfmemGetStatus (void);

// Requests dfmem manufacturer id and returns it.
//
// Returns : manufacturer id
unsigned char dfmemGetManufacturerID (void);

// Requests dfmem device ID data, returning the memory density.
//
// Returns : device density code (5 bits)
unsigned char dfmemGetChipSize (void);

// Puts dfmem in deep power-down mode.
void dfmemDeepSleep();

// Resumes dfmem from deep power-down mode.
void dfmemResumeFromDeepSleep();

// Saves to the current buffer, keeping track of position in memory.
void dfmemSave(unsigned char* data, unsigned int length);

// This function will write the current buffer into the flash memory if it
// contains any data, and then swaps the buffer pointer.
void dfmemSync();

// Reads back a "sample" from the flash memory following special page alignment
// rules: Samples do not cross page boundaries, and start from the beginning
// of the page.
void dfmemReadSample(unsigned long, unsigned int, unsigned char*);

// Erases enough sectors to fit a specified number of samples into the flash
void dfmemEraseSectorsForSamples(unsigned long, unsigned int);

#endif // __DFMEM_H
