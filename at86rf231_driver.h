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
 * Usage:
 *  It is recommended that the transceiver be in IDLE or OFF states when setting
 *  parameters.
 */

#ifndef __AT86RF231_DRIVER
#define __AT86RF231_DRIVER

#include "mac_packet.h"

/**
 * Interrupt handler type that must be registered to the driver.
 * The handler is called on interrupts with the source as the parameter.
 */
typedef void (*TrxIrqHandler)(unsigned int irq_cause);

/**
 * Initialize the transceiver hardware and software state.
 *
 * This function should be called before using the other transceiver functions.
 * Upon completion, the transceiver SPI port and configuration registers will be
 * initialized to default values as specified in at86rf231_driver.c, and the 
 * transceiver will be in an idle state.
 */
void trxSetup(void);

/**
 * Reset the transceiver hardware and software state.
 *
 * This function should be called when a hardware or state problem arises. Upon
 * completion, the transceiver SPI port and state will be reset to idle.
 */
void trxReset(void);

/**
 * Begin a transceiver calibration routine.
 *
 * The hardware calibrate functionality compensates for noise and other conditions.
 * According to Atmel documentation, this should be called no less than once every
 * 5 minutes of operation and takes at most 35 us.
 */
void trxCalibrate(void);

/**
 * Register the interrupt handler.
 *
 * Stores a function pointer indicating the interrupt handler for the transceiver.
 *
 * @param handler Function pointer to interrupt handler
 */
void trxSetIrqCallback(TrxIrqHandler handler);

/**
 * Set the transceiver local 16-bit address.
 *
 * @param addr 16-bit address
 */
void trxSetAddress(unsigned int addr);

/**
 * Set the transceiver local 16-bit PAN ID.
 *
 * @param pan_id 16-bit PAN ID
 */
void trxSetPan(unsigned int pan_id);

/**
 * Set the transceiver operating channel.
 *
 * @param channel 8-bit channel from 0 to 15
 */
void trxSetChannel(unsigned char channel);

/**
 * Set the transceiver hardware retry limit.
 *
 * This sets the maximum number of times the transceiver will attempt to resend
 * a packet if no ACK is received.
 *
 * @param retries 16-bit number of attempts
 */
void trxSetRetries(unsigned int retries);

/**
 * Read the transceiver hardware ID.
 *
 * @param id 4 length byte array to write ID string into.
 */
void trxReadId(unsigned char *id);

/**
 * Return the last buffered RSSI value
 * @return 0 to 28 with resolution of 3 decibels. Subtract 91 to scale.
 */
unsigned char trxReadRSSI(void);

/**
 * Read the last ED value
 * @return 0x00 to 0x54 in decibels. Subtract 91 to scale.
 */
unsigned char trxReadED(void);

/**
 * Write the contents of a packet to the transceiver.
 *
 * Serializes and writes a MacPacket object to the transceiver via its SPI port. 
 * The current implementation uses a DMA buffer, so the method will return before 
 * the write is complete.
 *
 * @param packet MacPacket object to write to the transceiver.
 */
void trxWriteFrameBuffer(MacPacket packet);

/**
 * Read the contents of the software frame buffer to a packet.
 *
 * Attempts to decode the frame buffer data into a MacPacket object. Depending on
 * the data and supported packet formats, this attempt may fail.
 *
 * @param packet MacPacket object to store decoded data in.
 * @return 1 for successful decode, 0 for failure.
 */
unsigned int trxReadFrameBuffer(MacPacket packet);

/**
 * Read the length of the current frame buffer data's payload.
 *
 * Returns the data length of the currently received transceiver frame data,
 * assuming the software frame buffer contains a valid frame.
 *
 * @return length of frame data in bytes
 */
unsigned int trxReadBufferDataLength(void);

/**
 * Begin transmission of transceiver contents.
 *
 * Initiates the frame transmit process for data that was written to the transceiver.
 */
void trxBeginTransmission(void);

/**
 * Sets transceiver to transmit state.
 *
 * Initiates a state transition to a transmit state. This method blocks until the transition
 * completes.
 */
void trxSetStateTx(void);

/**
 * Sets transceiver to receive state.
 *
 * Initiates a state transition to a receive state. This method blocks until the transition
 * completes.
 */
void trxSetStateRx(void);

/**
 * Sets transceiver to idle state.
 *
 * Initiates a state transition to a idle state. This method blocks until the transition
 * completes.
 */
void trxSetStateIdle(void);

/**
 * Sets transceiver to off state.
 *
 * Initiates a state transition to a off state. This method blocks until the transition
 * completes.
 */
void trxSetStateOff(void);

#endif
 
 
 
 
