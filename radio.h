/******************************************************************************
* Name: radio.h
* Desc: Software module for AT86RF231 (SPI)
* Date: 2010-06-02
* Author: stanbaek
******************************************************************************/
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
