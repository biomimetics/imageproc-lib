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
 * Description:
 *  The packet pool is a static set of FastQueue, MacPacket, and Payload
 *  objects that provide a pool of packet/payloads for use with the radio.
 *  Requesting and returning these resources is faster and more stable than
 *  continually creating and destroying them.
 *
 * Usage:
 *  All the rules for video rentals apply to packet rentals.
 *      - Check the rental (for NULL) before using it
 *      - Don't return rentals damaged or altered
 *      - Don't return rentals more than once
 *      - Don't use rentals after returning them
 *
 *  To request a packet or payload:
 *      // Call the respective method
 *      MacPacket packet = ppoolRequestPacket();
 *      if(packet == NULL) {
 *          // Handle request failure
 *      }
 *
 *  To return a packet or payload:
 *      if(!ppoolReturnPacket(packet)) {
 *          // Handle return failure
 *      }
 *
 */

#ifndef __PPOOL_H_

#define __PPOOL_H

#include "mac_packet.h"

// Initialize packet pool module
unsigned int ppoolInit(void);
// Free module resources
void ppoolClose(void);

// Request a mac packet + payload
MacPacket ppoolRequestFullPacket(unsigned int size);
unsigned int ppoolReturnFullPacket(MacPacket packet);

// Request/return a mac packet
MacPacket ppoolRequestPacket(void);
unsigned int ppoolReturnPacket(MacPacket packet);

// Request/return a payload
Payload ppoolRequestPayload(unsigned int size);
unsigned int ppoolReturnPayload(Payload pld);

#endif
