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
 * Master Mode SPI Controller for the dsPIC33F
 *
 *    by Humphrey Hu
 *
 * Revisions:
 *  Humphrey Hu         2011-11-10    Initial implemetation
 *                      
 * Usage:
 *        
 * This module serves as a wrapper for DMA-enabled SPI. Data is transferred
 * and received in "transactions," which can be started and ended with the 
 * respective function calls.
 *
 * Buffer and port configuration is in spi_controller.c
 * 
 * Ex:
 *  spi1BeginTransaction();
 *  // Transfer/Receive methods here
 *  spi1EndTransaction();
 *
 * Data transfer and reception methods come in synchronous single-byte 
 * and asynchronous mass-transfer varieties: 
 *
 * The synchronous methods return once the data (a single byte) has been 
 * read/written, and is useful for small transactions where the overhead of 
 * buffering is prohibitive, or when timing is critical.
 *
 * The asynchronous methods return once the target data has been buffered,
 * and continue the transaction in the background without using processor resources.
 * Once the transaction is complete, the registered callback is executed with a
 * interrupt source flag. This method of transfer is useful for large transactions
 * where transferring the data synchronously would consume too much processor time.
 *
 */


#ifndef __SPI_CONTROLLER_H
#define __SPI_CONTROLLER_H

#define SPIC_NUM_PORTS             (2)

/** Interrupt sources */
typedef enum {
    SPIC_TRANS_SUCCESS, /** Successful transceive */
    SPIC_TRANS_TIMEOUT, /** Transceive timeout */
} SpicIrqSrc;

/**
 * Interrupt handler type that must be registered to the driver.
 * The handler is called on interrupts with the source as the parameter.
 */
typedef void (*SpicIrqHandler) (unsigned int irq_cause);

/**
 * Set up the SPI-DMA module for use.
 *
 * This must be run before module port access methods can be used.
 */
void spicSetupChannel1(void);
void spicSetupChannel2(void);

/**
 * Set the interrupt handler for port 1.
 *
 * @param handler Function pointer to interrupt handler
 */
void spic1SetCallback(SpicIrqHandler handler);

/**
 * Begin a transaction on port 1.
 *
 * Acquires and locks the port, and then sets the appropriate chip select line
 * levels as defined in spi_controller.c
 *
 * This function blocks if there is an ongoing transaction, returning
 * after it has acquired the port. Note that this can result in deadlocks
 * if used improperly.
 */
void spic1BeginTransaction(void);

/**
 * End a transaction on port 1.
 */
void spic1EndTransaction(void);

/**
 * Resets port 1.
 *
 * Use this when a deadlock is detected. This function cancels the current
 * transaction and restores the port to its default state.
 */
void spic1Reset(void);

/**
 * Transmit a byte on port 1 during a transaction.
 *
 * @param data Byte of data to send
 * @return Byte read from SPI bus
 */
unsigned char spic1Transmit(unsigned char data);

/**
 * Receive a byte on port 1 by writing a NULL byte.
 *
 * @return Byte read from SPI bus
 */
unsigned char spic1Receive(void);

/**
 * Transmit the contents of a buffer on port 1 via DMA
 *
 * This method copies the buffer contents into port 1's transmit buffer, so the buffer 
 * can be volatile. Since this method uses DMA, it returns before the write is completed.
 * 
 * @param len Number of bytes to write
 * @param buff Array of data or NULL to write all NULL bytes
 * @param timeout Number of milliseconds to wait before transaction timeout
 * @return Number of bytes saved to buffer
 */
unsigned int spic1MassTransmit(unsigned int len, unsigned char *buff, unsigned int timeout);

/**
 * Read the contents of port 1's DMA buffer
 *
 * Copies the contents of port 1's DMA receive buffer into another buffer. This should be
 * preceded by a DMA transmit.
 *
 * @param len Number of bytes to read
 * @param buff Buffer to write into
 * @return Number of bytes read out of buffer
 */
unsigned int spic1ReadBuffer(unsigned int len, unsigned char *buff);

/**
 * Set the interrupt handler for port 2.
 *
 * @param handler Function pointer to interrupt handler
 */
void spic2SetCallback(SpicIrqHandler);

/**
 * Begin a transaction on port 2.
 *
 * Acquires and locks the port, and then sets the appropriate chip select line
 * levels as defined in spi_controller.c
 *
 * This function blocks if there is an ongoing transaction, returning
 * after it has acquired the port. Note that this can result in deadlocks
 * if used improperly.
 */
void spic2BeginTransaction(void);

/**
 * End a transaction on port 2.
 */
void spic2EndTransaction(void);

/**
 * Resets port 2.
 *
 * Use this when a deadlock is detected. This function cancels the current
 * transaction and restores the port to its default state.
 */
void spic2Reset(void);

/**
 * Transmit a byte on port 2 during a transaction.
 *
 * @param data Byte of data to send
 * @return Byte read from SPI bus
 */
unsigned char spic2Transmit(unsigned char data);

/**
 * Receive a byte on port 2 by writing a NULL byte.
 *
 * @return Byte read from SPI bus
 */
unsigned char spic2Receive(void);

/**
 * Transmit the contents of a buffer on port 2 via DMA
 *
 * This method copies the buffer contents into port 2's transmit buffer, so the buffer 
 * can be volatile. Since this method uses DMA, it returns before the write is completed.
 * 
 * @param len Number of bytes to write
 * @param buff Array of data or NULL to write all NULL bytes
 * @param timeout Number of milliseconds to wait before transaction timeout
 * @return Number of bytes saved to buffer
 */
unsigned int spic2MassTransmit(unsigned int len, unsigned char *buff, unsigned int timeout);

/**
 * Read the contents of port 2's DMA buffer
 *
 * Copies the contents of port 2's DMA receive buffer into another buffer. This should be
 * preceded by a DMA transmit.
 *
 * @param len Number of bytes to read
 * @param buff Buffer to write into
 * @return Number of bytes read out of buffer
 */
unsigned int spic2ReadBuffer(unsigned int len, unsigned char *buff);

#endif


