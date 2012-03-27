#ifndef __AT86RF_H
#define __AT86RF_H

#include "generic_typedefs.h"
#include "packet_queue.h"
#include "payload.h"

#define CHANNEL_ASSESSMENT_CARRIER_SENSE    0x00
#define CHANNEL_ASSESSMENT_ENERGY_DETECT    0x01

#define POWER_STATE_DEEP_SLEEP              0x00
#define POWER_STATE_OPERATE                 0xFF

#define PACKET_TYPE_MASK        0x03
#define BROADCAST_MASK          0x04
#define SECURITY_MASK           0x08
#define REPEAT_MASK             0x10
#define ACK_MASK                0x20
#define DSTPRSNT_MASK           0x40
#define SRCPRSNT_MASK           0x80

#define PACKET_TYPE_BEACON      0x00
#define PACKET_TYPE_DATA        0x01
#define PACKET_TYPE_ACK         0x02
#define PACKET_TYPE_COMMAND     0x03
#define PACKET_TYPE_RESERVE     0x04


//Frame Control bit settings
#define SEC_EN_FALSE            0x00
#define FRM_PENDING_FALSE       0x00
#define ACK_REQ_TRUE            0x01
#define ACK_REQ_FALSE           0x00
#define PAN_ID_COMP_TRUE        0x01
#define RESERVED                0x00
#define DEST_ADDR_MODE_16       0x02
#define VERSION_2006            0x01    
#define SRC_ADDR_MODE_16        0x02

#define MAC_HEADER_LENGTH       9 //11  //Based on 16-bit addressing for PAN and device and no 
                                    //auxiliary security header
#define CRC_LENGTH              2
#define MAX_DATA_LENGTH         100

#define CHAN_MIN            11
#define CHAN_MAX            26  

void        atInit(WordVal src_addr, WordVal pan_id);
char        atSetChannel(char chan);
char        atGetChannel(void);
void        atSetFrameRetries(char retry);
void        atSetCSMARetries(char retry);
void        atSetAntDiversity(char enable);
void        atRxPQueueInit(char rxpq_max_size);
void        atTxPQueueInit(char txpq_max_size);
PacketQueue atGetRxPacketQueue();
PacketQueue atGetTxPacketQueue();
unsigned char phyGetState(void);
unsigned char phyGetLastAckd(void);
unsigned char phyReadRSSI(void);
unsigned char phyReadED(void);
unsigned char phyReadLQI(void);
MacPacket   macCreatePacket();
void        macDeletePacket(MacPacket p);
MacPacket   macReceivePacket();
void        macSendPacket();
WordVal     macGetSrcPANID();
void        macSetSrcPANID(WordVal pan_id);
WordVal     macGetSrcAddr();
void        macSetSrcAddr(WordVal src_addr);
WordVal     macGetDestPANID();
void        macSetDestPANID(WordVal pan_id);
WordVal     macGetDestAddr();
void        macSetDestAddr(WordVal addr);
void        atSetPromMode(char on);


#endif
