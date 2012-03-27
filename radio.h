/*
 * Copyright (c) 2010, Regents of the University of California
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
 * Header for the wireless radio interface
 *
 * by Stanley S. Baek
 *
 * v.beta
 */

#include "packet_queue.h"
#include "payload.h"
#include "at86rf.h"
#include "mac_packet.h"

#ifndef __RADIO_H
#define __RADIO_H

void        radioInit(WordVal src_addr, WordVal src_pan_id, int rxpq_max_size, int txpq_max_size);
void        radioSetSrcAddr(WordVal src_addr);
WordVal     radioGetSrcAddr(void);
void        radioSetPanID(WordVal pan_id);
WordVal     radioGetPanID(void);
void        radioSetChannel(char chan);
char        radioGetChannel(void);
void        radioSetDestAddr(WordVal dest_addr);
WordVal     radioGetDestAddr(); 
char        radioSendPayload(WordVal dest_addr, Payload pld);
Payload     radioReceivePayload();
void        radioSetRetries();
void        radioEnqueueRxPacket(MacPacket macRxPacket);
MacPacket   radioDequeueRxPacket();
PacketQueue radioGetRxQueue();
int         radioIsRxQueueEmpty();
int         radioIsRxQueueFull();
int         radioGetRxQueueSize();
void        radioEnqueueTxPacket(MacPacket macTxPacket);
MacPacket   radioDequeueTxPacket();
PacketQueue radioGetTxQueue();
int         radioIsTxQueueEmpty();
int         radioIsTxQueueFull();
int         radioGetTxQueueSize();

#endif // __RADIO_H
