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
 * Wireless radio interface
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanlay S. Baek     2010-6-2    Initial release
 */

#include "radio.h"
#include "packet_queue.h"
#include "at86rf.h"
#include "utils.h"

static PacketQueue rx_p_queue;
static PacketQueue tx_p_queue;

void radioInit(WordVal src_addr, WordVal src_pan_id, int rxpq_max_size, int txpq_max_size) {
    atInit(src_addr, src_pan_id);
    macSetDestPANID(src_pan_id); //Intra-PAN communications only
    radioSetChannel(CHAN_MIN); //default channel is 11
    atRxPQueueInit(rxpq_max_size);
    atTxPQueueInit(txpq_max_size);
    rx_p_queue = atGetRxPacketQueue();
    tx_p_queue = atGetTxPacketQueue();

}

void radioSetSrcAddr(WordVal src_addr){

    macSetSrcAddr(src_addr);

}

void radioSetPanID(WordVal pan_id){

    macSetSrcPANID(pan_id);

}

void radioSetChannel(char chan) {

    atSetChannel(chan);

}

char radioGetChannel(void) {

    return atGetChannel();

}

WordVal radioGetSrcAddr(void) {

    return macGetSrcAddr();

}

WordVal radioGetPanID(void) {

    return macGetSrcPANID();
    
}

void radioSetDestAddr(WordVal dest_addr) {

    macSetDestAddr(dest_addr);

}

WordVal radioGetDestAddr() {

    return macGetDestAddr();

}

MacPacket radioCreatePacket() {

    return macCreatePacket();

}

void radioDeletePacket(MacPacket p) {
    
    return macDeletePacket(p);

}

char radioSendPayload(WordVal dest_addr, Payload pld) {
    
    MacPacket tx_packet = radioCreatePacket();
    tx_packet->dest_addr = dest_addr;
    tx_packet->dest_pan_id = macGetDestPANID();
    tx_packet->payload = pld;
    tx_packet->payload_length = payGetPayloadLength(pld);

    radioEnqueueTxPacket(tx_packet);
    macSendPacket();
    return 1;
}

Payload radioReceivePayload(void) {
    MacPacket rx_packet;
    Payload pld;
    if ((rx_packet = radioDequeueRxPacket()) != NULL) {
		pld = rx_packet->payload;
    	macDeletePacket(rx_packet);
    	return pld;
	} else {
		return NULL;
    }

}

void radioEnqueueRxPacket(MacPacket mac_rx_packet) {
    
    pktqPush(rx_p_queue, mac_rx_packet); 

}

MacPacket radioDequeueRxPacket() {

    return (MacPacket)pktqPop(rx_p_queue); 

}

PacketQueue radioGetRxQueue() {

    return rx_p_queue;

}

int radioIsRxQueueEmpty() {

    return pktqIsEmpty(rx_p_queue);

}

int radioIsRxQueueFull() {

    return pktqIsFull(rx_p_queue);

}

int radioGetRxQueueSize() {

    return pktqGetSize(rx_p_queue);

}

void radioEnqueueTxPacket(MacPacket mac_tx_packet) {

    pktqPush(tx_p_queue, mac_tx_packet);

}

MacPacket radioDequeueTxPacket() {

    return (MacPacket)pktqPop(tx_p_queue); 

}

PacketQueue radioGetTxQueue() {

    return tx_p_queue;

}

int radioIsTxQueueEmpty() {

    return pktqIsEmpty(tx_p_queue);

}

int radioIsTxQueueFull() {

    return pktqIsFull(tx_p_queue);

}

int radioGetTxQueueSize() {

    return pktqGetSize(tx_p_queue);

}
