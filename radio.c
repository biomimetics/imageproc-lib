/******************************************************************************
* Name: radio.c
* Desc: Software module for AT86RF231 (SPI)
* Date: 2010-06-02
* Author: stanbaek
*
* UNDER DEVELOPMENT == UNDER DEVELOPMENT == UNDER DEVELOPMENT
*
* TODO: 
*  1. Implement DMA for SPI.
*  2. Polling may need to be implemented -> radioReceivePacket()
*  3. The rx_frame_ buffer can be overwritten before user reads the buffer.
*     Work-around: Need to modify ISR and/or increase number of buffers.
*
* SPI1 is used for AT86RF231e
* SPI2 should be used if you run this module on
* MikroElektronika dev board because RB2 is used for LCD.
******************************************************************************/

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
