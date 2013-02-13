/**
 * Copyright (c) 2012, Regents of the University of California
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
 * Packet Pool
 *
 * by Humphrey Hu
 *
 * v 0.1
 *
 * Revisions:
 *  Humphrey Hu         2012-02-04      Initial implementation
 *  Humphrey Hu         2012-02-08      Restructured to allow requesting of
 *                                      separate items
 *  Humphrey Hu         2012-02-13      Fixed initialization bug
 *
 * Notes:
 *
 */

#include "ppool.h"
#include "mac_packet.h"
#include "carray.h"
#include <stdlib.h>

// ================ CONSTANTS =================================================

#define NUM_CUTOFFS         (3)
#define NUM_PACKETS         (30)
#define MAX_PAYLOAD_SIZE    (117)

/** Sort in ascending order
 * 117 is max data size (127 - 8 - 2 for MPDU and payload headers)
 */
static unsigned int CUTOFFS[NUM_CUTOFFS] = {12, 86, MAX_PAYLOAD_SIZE};
static unsigned int QUANTITIES[NUM_CUTOFFS] = {20, 8, 2};
static unsigned char is_initialized = 0;

// ================ STATIC VARIABLES ==========================================
/** FastQueue object pools */
static CircArray packet_pool, payload_pools[NUM_CUTOFFS];

// ================ FUNCTION STUBS ============================================

static inline int ppoolFindOffsetIndex(unsigned int size);

// ================ PUBLIC FUNCTIONS ==========================================

// Initialize the module
unsigned int ppoolInit(void) {

    unsigned int i, j;
    MacPacket packet;
    Payload pld;
    CircArray fq;

    if(is_initialized) { return 1; }

    packet_pool = carrayCreate(NUM_PACKETS);
    if(packet_pool == NULL) {
        return 0;
    }

    for(i = 0; i < NUM_CUTOFFS; i++) {
        payload_pools[i] = carrayCreate(QUANTITIES[i]);
        if(payload_pools[i] == NULL) {
            ppoolClose();
            return 0;
        }
    }

    for(i = 0; i < NUM_PACKETS; i++) {
        packet = macCreateDataPacket();
        if(packet == NULL) {
            break;
        }
        carrayAddHead(packet_pool, packet);
    }
    unsigned int num, size;
    for(i = 0; i < NUM_CUTOFFS; i++) {
        fq = payload_pools[i];
        num = QUANTITIES[i];
        size = CUTOFFS[i];
        for(j = 0; j < num; j++) {
            pld = payCreateEmpty(size);
            if(pld == NULL) {
                break;
            }
            carrayAddHead(fq, pld);
        }
    }

    is_initialized = 1;
    return 1;

}

void ppoolClose(void) {

    unsigned int i;
    MacPacket packet;
    Payload pld;
    CircArray fq;

    if(packet_pool != NULL) {
        while(!carrayIsEmpty(packet_pool)) {
            packet = carrayPopTail(packet_pool);
            macDeletePacket(packet);
        }
        carrayDelete(packet_pool);
    }

    for(i = 0; i < NUM_CUTOFFS; i++) {
        fq = payload_pools[i];
        if(fq != NULL) {
            while(!carrayIsEmpty(fq)) {
                pld = carrayPopTail(fq);
                payDelete(pld);
            }
            carrayDelete(fq);
        }
    }

}

// Request a MacPacket with payload capacity of appropriate size
// Note that a request is only fulfilled if there is a packet in the
// smallest appropriate size category, ie. the largest packets will
// never be allocated to small requests.

MacPacket ppoolRequestFullPacket(unsigned int size) {

    MacPacket packet;
    Payload pld;

    packet = ppoolRequestPacket();
    if(packet == NULL) { return NULL; }

    pld = ppoolRequestPayload(size);
    if(pld == NULL) {
        // Assume that we don't have to check the return value
        ppoolReturnPacket(packet);
        return NULL;
    }

    macSetPayload(packet, pld);

    return packet;

}

MacPacket ppoolRequestPacket(void) {

    MacPacket packet;

    packet = (MacPacket) carrayPopTail(packet_pool);    // Get a packet
    if(packet == NULL) {    // Check for failure
        return NULL;
    }

    return packet;

}

Payload ppoolRequestPayload(unsigned int size) {

    int index;
    Payload pld;
    CircArray payq;

    index = ppoolFindOffsetIndex(size);
    if(index < 0) {
        return NULL;
    }

    payq = payload_pools[index]; // Retrieve appropriate payload pool
    pld = (Payload) carrayPopTail(payq);  // Get a payload
    if(pld == NULL) {   // Check for failure
        return NULL;
    }
    pld->data_length = size;
    return pld;

}

// Return a combined packet + payload to the pool
unsigned int ppoolReturnFullPacket(MacPacket packet) {

    Payload pld;

    if(packet == NULL) { return 0; }    // Don't deal with damaged returns
    pld = macGetPayload(packet);
    if(pld == NULL) { return 0; }

    if(!ppoolReturnPayload(pld)) {    // Try to return packet
        return 0;   // Check for failure
    }
    if(!ppoolReturnPacket(packet)) {  // Try to return payload
        return 0;   // Sort of screwed since we already returned packet...oh well
    }
    return 1;

}

unsigned int ppoolReturnPacket(MacPacket packet) {

    if(!carrayAddTail(packet_pool, packet)) {
        return 0;   // Check for failure
    }
    return 1;

}

unsigned int ppoolReturnPayload(Payload pld) {

    CircArray fq;
    int index;

    if(pld == NULL) { return 0; }   // Can't return NULLs

    index = ppoolFindOffsetIndex(payGetDataLength(pld));
    if(index < 0) { return 0; }     // Invalid payload size

    fq = payload_pools[index];
    if(!carrayAddTail(fq, pld)) {
        return 0;   // Check for failure
    }
    return 1;

}

// ================ PRIVATE FUNCTIONS =========================================
/**
 * Finds the index corresponding to a payload size
 *
 * @param size Payload size to search for
 * @return int -1 if not found, otherwise index corresponding to payload
 */
static inline int ppoolFindOffsetIndex(unsigned int size) {
    int i;
    for(i = 0; i < NUM_CUTOFFS; i++) {
        if(CUTOFFS[i] >= size) {
            return i;
        }
    }
    return -1;

}
