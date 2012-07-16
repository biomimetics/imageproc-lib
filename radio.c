/**
 * Copyright (c) 2011-2012, Regents of the University of California
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
 * High Level Wireless Communications Driver
 *
 * by Humphrey Hu
 *
 * v. 0.4
 *
 * Revisions:
 *  Humphrey Hu      2011-06-06      Initial implementation
 *  Humphrey Hu      2012-02-03      Structural changes to reduce irq handler runtime
 */

#include "utils.h"
#include "init_default.h"
#include "radio.h"
#include "payload.h"
#include "carray.h"
#include "mac_packet.h"
#include "sclock.h"
#include "timer.h"
#include "led.h"
#include "ppool.h"

#include "at86rf231.h"  // Current transceiver IC
#include "at86rf231_driver.h"

#include <stdlib.h>

#define RADIO_DEFAULT_SRC_ADDR                  (0x1101)
#define RADIO_DEFAULT_SRC_PAN                   (0x1001)
#define RADIO_DEFAULT_CHANNEL                   (0x15)
#define RADIO_DEFAULT_RETRIES                   (3)

#define RADIO_DEFAULT_PACKET_RETRIES            (2)
#define TX_TIMEOUT_MS                           (75)
#define DEFAULT_WATCHDOG_TIME                     (1000)

#define RADIO_CALIB_PERIOD                      (300000) // 5 minutes

// =========== Static variables ===============================================

// State information
static unsigned char is_ready = 0;
static RadioState radio_state;
static unsigned char packet_sqn, retries;

static unsigned long last_calib_timestamp, progress_timestamp;
static unsigned int watchdog_timeout, watchdog_running;

// In/out packet FIFO queues
static CircArray tx_queue, rx_queue;

// Local config information
static unsigned int local_addr, local_pan, max_packet_retries;
static unsigned char local_channel;


// =========== Function stubs =================================================

// IRQ handlers
void trxCallback(unsigned int irq_cause);
static inline void watchdogProgress(void);

static void radioReset(void);

// Internal processing
static void radioProcessTx(void);
static void radioProcessRx(void);

// Internal state management methods
static unsigned int radioBeginTransition(void);
static unsigned int radioSetStateTx(void);
static unsigned int radioSetStateRx(void);
static unsigned int radioSetStateIdle(void);
static unsigned int radioSetStateOff(void);


// =========== Public functions ===============================================

// Initialize radio software and hardware
void radioInit(unsigned int tx_queue_length, unsigned int rx_queue_length) {
    
    tx_queue = carrayCreate(tx_queue_length);
    rx_queue = carrayCreate(rx_queue_length);

    packet_sqn = 0;
    retries = 0;
    last_calib_timestamp = 0;
        
    radioDisableWatchdog();
    radioSetWatchdogTime(DEFAULT_WATCHDOG_TIME);        

    trxSetup(); // Configure transceiver IC and driver
    trxSetIrqCallback(&trxCallback);

    radioSetSrcAddr(RADIO_DEFAULT_SRC_ADDR);
    radioSetSrcPanID(RADIO_DEFAULT_SRC_PAN);
    radioSetChannel(RADIO_DEFAULT_CHANNEL);
    radioSetRetries(RADIO_DEFAULT_PACKET_RETRIES);    
    trxSetRetries(RADIO_DEFAULT_RETRIES);

    is_ready = 1;

    trxSetStateRx();

}

void radioSetSrcAddr(unsigned int src_addr) {
    local_addr = src_addr;
    trxSetAddress(src_addr);
}

unsigned int radioGetSrcAddr(void) {
    return local_addr;
}

void radioSetSrcPanID(unsigned int src_pan_id) {
    local_pan = src_pan_id;
    trxSetPan(src_pan_id);
}

unsigned int radioGetSrcPanID(void) {
    return local_pan;
}

void radioSetChannel(unsigned char channel) {
    local_channel = channel;
    trxSetChannel(channel);
}

unsigned char radioGetChannel(void) {
    return local_channel;
}

void radioSetRetries(unsigned char retries) {
    max_packet_retries = retries;
}

unsigned char radioGetRetries(void) {
    return max_packet_retries;
}

void radioSetWatchdogState(unsigned char state) {
    if(state == 0) { radioDisableWatchdog(); }
    else if(state == 1) { radioEnableWatchdog(); }
}

void radioEnableWatchdog(void) {
    watchdog_running = 1;
    watchdogProgress();
}

void radioDisableWatchdog(void) {
    watchdog_running = 0;
}

void radioSetWatchdogTime(unsigned int time) {
    watchdog_timeout = time;
    watchdogProgress();
}

MacPacket radioDequeueRxPacket(void) {
    return (MacPacket)carrayPopTail(rx_queue);
}

unsigned int radioEnqueueTxPacket(MacPacket packet) {
    return carrayAddTail(tx_queue, packet);
}

unsigned int radioTxQueueEmpty(void) {
    return carrayIsEmpty(tx_queue);
}

unsigned int radioTxQueueFull(void) {
    return carrayIsFull(tx_queue);
}

unsigned int radioGetTxQueueSize(void) {
    return carrayGetSize(tx_queue);
}

unsigned int radioRxQueueEmpty(void){
    return carrayIsEmpty(rx_queue);
}

unsigned int radioRxQueueFull(void) {
    return carrayIsFull(rx_queue);
}

unsigned int radioGetRxQueueSize(void) {
    return carrayGetSize(rx_queue);
}

void radioFlushQueues(void) {
    while (!carrayIsEmpty(tx_queue)) {
        radioReturnPacket((MacPacket)carrayPopTail(tx_queue));
    }
    while (!carrayIsEmpty(rx_queue)) {
        radioReturnPacket((MacPacket)carrayPopTail(rx_queue));
    }
}

MacPacket radioRequestPacket(unsigned int data_size) {

    MacPacket packet;

    packet = ppoolRequestFullPacket(data_size);
    if(packet == NULL) { return NULL; }

    macSetSrc(packet, local_pan, local_addr);
    macSetDestPan(packet, local_pan);

    return packet;

}

MacPacket radioCreatePacket(unsigned int data_size) {
    return NULL;
}

unsigned int radioReturnPacket(MacPacket packet) {
    return ppoolReturnFullPacket(packet);
}

void radioDeletePacket(MacPacket packet) {
    return;
}

// The Big Function
void radioProcess(void) {

    unsigned long currentTime;
    
    currentTime = sclockGetLocalMillis();

    if(watchdog_running) {
        if(currentTime - progress_timestamp > DEFAULT_WATCHDOG_TIME) {
            radioReset();
            return;
        }
    }

    // Process pending outgoing packets
    if(!radioTxQueueEmpty()) {

        // Return if can't get to Tx state at the moment
        if(!radioSetStateTx()) { return; }
        watchdogProgress();
        radioProcessTx(); // Process outgoing buffer
        return;

    }

#if defined(RADIO_AUTOCALIBRATE) // Auto calibration routine
    // Check if calibration is necessary
    currentTime = sclockGetLocalMillis();
    if(currentTime - last_calib_timestamp > RADIO_CALIB_PERIOD) {
        if(!radioSetStateOff()) { return; }
        trxCalibrate();
        last_calib_timestamp = currentTime;
    }
#endif

    // Default to Rx state
    if(!radioSetStateRx()) { return; }

    // If the code runs to this point, all buffers are clear and radio is idle
    watchdogProgress();

}


// =========== Private functions ==============================================

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {

    // Disable and reset timer
    DisableIntT3;
    WriteTimer3(0);
    radioReset();
    _T3IF = 0;

}

static void radioReset(void) {
    
    trxReset();
    radio_state = STATE_OFF;
    radioSetStateIdle();
    watchdogProgress();
    LED_ORANGE = 0;
    LED_RED = ~LED_RED;

}

/**
 * Transceiver interrupt handler
 *
 * Note that this doesn't need critical sections since this will
 * only be called in interrupt context
 * @param irq_cause Interrupt source code
 */
void trxCallback(unsigned int irq_cause) {

    if(radio_state == STATE_SLEEP) {
        // Shouldn't be here since sleep isn't implemented yet!
    }
    else if(radio_state == STATE_IDLE) {
        // Shouldn't be getting interrupts when idle
    }
    else if(radio_state == STATE_RX_IDLE) {

        // Beginning reception process
        if(irq_cause == RADIO_RX_START) {
            LED_ORANGE = 1;
            radio_state = STATE_RX_BUSY;
        }

    } else if(radio_state == STATE_RX_BUSY) {

        // Reception complete
        if(irq_cause == RADIO_RX_SUCCESS) {
            radioProcessRx();   // Process newly received data
            LED_ORANGE = 0;
            radio_state = STATE_RX_IDLE;    // Transition after data processed
        }

    } else if(radio_state == STATE_TX_IDLE) {
        // Shouldn't be getting interrupts when waiting to transmit
    } else if(radio_state == STATE_TX_BUSY) {

        radio_state = STATE_TX_IDLE;
        LED_ORANGE = 0;
        // Transmit successful
        if(irq_cause == RADIO_TX_SUCCESS) {
            radioReturnPacket(carrayPopHead(tx_queue));
            radioSetStateRx();
        } else if(irq_cause == RADIO_TX_FAILURE) {
            // If no more retries, reset retry counter
            retries++;
            if(retries > max_packet_retries) {
                retries = 0;
                radioReturnPacket((MacPacket)carrayPopHead(tx_queue));
                radioSetStateRx();
            }
        }
    }

    // Hardware error
    if(irq_cause == RADIO_HW_FAILURE) {
        // Reset everything
        trxReset();
        //radioFlushQueues();
    }
}

/**
 * Set the radio to a transmit state
 */
static unsigned int radioSetStateTx(void) {

    unsigned int lockAcquired;

    // If already in Tx mode
    if(radio_state == STATE_TX_IDLE) { return 1; }

    // Attempt to begin transition
    lockAcquired = radioBeginTransition();
    if(!lockAcquired) { return 0; }

    trxSetStateTx();
    radio_state = STATE_TX_IDLE;
    return 1;

}

/**
 * Set the radio to a receive state
 */
static unsigned int radioSetStateRx(void) {

    unsigned int lockAcquired;

    // If already in Rx mode
    if(radio_state == STATE_RX_IDLE) { return 1; }

    // Attempt to begin transitionin
    lockAcquired = radioBeginTransition();
    if(!lockAcquired) { return 0; }

    trxSetStateRx();
    radio_state = STATE_RX_IDLE;
    return 1;

}

/**
 * Sets the radio to an idle state
 */
static unsigned int radioSetStateIdle(void) {

    unsigned int lockAcquired;

    // If already in idle mode
    if(radio_state == STATE_IDLE) { return 1; }

    // Attempt to begin transitionin
    lockAcquired = radioBeginTransition();
    if(!lockAcquired) { return 0; }

    trxSetStateIdle();
    radio_state = STATE_IDLE;
    return 1;

}

/**
* Sets the radio to an off state
*/
static unsigned int radioSetStateOff(void) {

   unsigned int lockAcquired;

   // If already in idle mode
   if(radio_state == STATE_OFF) { return 1; }

   // Attempt to begin transitionin
   lockAcquired = radioBeginTransition();
   if(!lockAcquired) { return 0; }

   trxSetStateOff();
   radio_state = STATE_OFF;
   return 1;

}

/**
 * Atomically checks and set the radio to transitioning state.
 *
 * Note that the radio in transitioning state will capture but disregard
 * interrupts from the transceiver.
 * @return 1 if lock acquired, 0 otherwise
 */
static unsigned int radioBeginTransition(void) {

    unsigned int busy;

    CRITICAL_SECTION_START

    busy =  (radio_state == STATE_RX_BUSY)
    || (radio_state == STATE_TX_BUSY)
    || (radio_state == STATE_TRANSITIONING);

    if(!busy) {
        radio_state = STATE_TRANSITIONING;
    }

    CRITICAL_SECTION_END

    return !busy;

}

/**
 * Process a pending packet send request
 */
static void radioProcessTx(void) {

    MacPacket packet;

    packet = (MacPacket) carrayPeekHead(tx_queue); // Find an outgoing packet
    if(packet == NULL) { return; }

    // State should be STATE_TX_IDLE upon entering function
    radio_state = STATE_TX_BUSY;    // Update state
    LED_ORANGE = 1;                    // Indicate RX activity

    macSetSeqNum(packet, packet_sqn++); // Set packet sequence number

    trxWriteFrameBuffer(packet); // Write packet to transmitter and send
    trxBeginTransmission();

}

/**
 * Process a pending packet receive request
 */
static void radioProcessRx(void) {

    MacPacket packet;
    unsigned char len;

    if(radioRxQueueFull()) { return; } // Don't bother if rx queue full

    len = trxReadBufferDataLength(); // Read received frame data length
    packet = radioRequestPacket(len - PAYLOAD_HEADER_LENGTH); // Pull appropriate packet from pool

    if(packet == NULL) { return; }

    trxReadFrameBuffer(packet); // Retrieve frame from transceiver
    packet->timestamp = sclockGetLocalTicks(); // Mark local time of reception

    if(!carrayAddTail(rx_queue, packet)) {
        radioReturnPacket(packet); // Check for failure
    }

}

static inline void watchdogProgress(void) {

    progress_timestamp = sclockGetLocalMillis();

}
