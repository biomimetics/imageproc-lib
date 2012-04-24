/*
 * Copyright (c) 2010-2012, Regents of the University of California
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
 * IEEE 802.15.4 Packet Utilities
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu     2011-07-27      Minor comment changes
 *  Humphrey Hu     2011-10-01      Changed to full utility module
 *  Humphrey Hu     2012-02-20      Added packet fields for clock sync
 *
 * Usage:
 *
 *
 */


#ifndef __MACPACKET_H
#define __MACPACKET_H

#include "generic_typedefs.h"
#include "payload.h"

#define MAC_PACKET_TYPE_BEACON      (0)
#define MAC_PACKET_TYPE_DATA        (1)
#define MAC_PACKET_TYPE_ACK         (2)
#define MAC_PACKET_TYPE_COMMAND     (3)

#define MAC_SEC_DISABLED            (0)
#define MAC_SEC_ENABLED             (1)

#define MAC_FRAME_COMPLETE          (0)
#define MAC_FRAME_PENDING           (1)

#define MAC_ACK_NOT_REQUIRED        (0)
#define MAC_ACK_REQUIRED            (1)

#define MAC_INTRAPAN_OFF            (0)
#define MAC_INTRAPAN_ON             (1)

#define MAC_DEST_ADDR_MODE_NONE     (0)
#define MAC_DEST_ADDR_MODE_16BIT    (2)
#define MAC_DEST_ADDR_MODE_64BIT    (3)

#define MAC_FIRMWARE_2003           (0)
#define MAC_FIRMWARE_2006           (1)

#define MAC_SRC_ADDR_MODE_NONE      (0)
#define MAC_SRC_ADDR_MODE_16BIT     (2)
#define MAC_SRC_ADDR_MODE_64BIT     (3)

#define MAC_MAX_DATA_SIZE           (125)

typedef struct
{
    union
    {
        WordVal    val;
        struct
        {
            word        packet_type     : 3;        // type of packet. Possible types are
                                                    // 0 PACKET_TYPE_BEACON - Beacon
                                                    // 1 PACKET_TYPE_DATA -  Data
                                                    // 2 PACKET_TYPE_ACK -  Acknowledgement type
                                                    // 3 PACKET_TYPE_COMMAND - MAC command
                                                    // 4-7 PACKET_TYPE_RESERVE - Reserved type
            word        sec_en          : 1;        // 1: secure the MAC payload, 0: send plain text
                                                    // We are *not* currently implementing secured transmission
            word        frm_pending     : 1;        // 1: sending device has more data for recipient, 0: no more data
            word        ack_req         : 1;        // 1: acknowledgement required, 0: no acknowldgement

            word        pan_id_comp     : 1;        // 1: PAN ID compression subfield  (Intra-Pan Mode)
            word        reserved        : 3;
            word        dest_addr_mode  : 2;        // 0: PAN ID and dest addr not present,
                                                    // 1: reserved,
                                                    // 2: address field contains 16 bit short address
                                                    // 3: address field contains 64 bit extended address - NOT IMPLEMENTED
            word        frm_version     : 2;        // 0: IEEE 802.15.4-2003
                                                    // 1: IEEE 802.15.4-2006
                                                    // 2-3: Reserved
            word        src_addr_mode   : 2;        // 0: PAN ID and src addr not present,
                                                    // 1: reserved,
                                                    // 2: address field contains 16 bit short address
                                                    // 3: address field contains 64 bit extended address
        } bits;
    } frame_ctrl;                       // Frame control bits
    byte        seq_num;                // Packet sequence number
    WordVal     dest_pan_id;            // Destination PAN ID
    WordVal     dest_addr;              // Destination short address
    WordVal     src_pan_id;             // Source PAN ID
    WordVal     src_addr;               // Source short address
    //byte * auxSecHdr                  // Placeholder for auxiliary security header
    // The following are non-physical packet fields
    Payload     payload;                // Data payload object
    byte        payload_length;         // Data payload length in octets
    unsigned long timestamp;            // Local system time at which receiver received packet
} MacPacketStruct;

typedef MacPacketStruct* MacPacket;

// Object creation
MacPacket macCreateEmpty(void);
MacPacket macCreateDataPacket(void);
MacPacket macCreateBeaconPacket(void);
MacPacket macCreateCommandPacket(void);
void macDeletePacket(MacPacket packet);

// Getters/Setters
void macSetSeqNum(MacPacket packet, char num);

void macSetDestAddr(MacPacket packet, unsigned int dest_addr);
void macSetDestPan(MacPacket packet, unsigned int pan_id);

void macSetSrc(MacPacket, unsigned int, unsigned int);
unsigned int macGetSrcAddr(MacPacket);
unsigned int macGetSrcPan(MacPacket);

void macSetPayload(MacPacket, Payload);
Payload macGetPayload(MacPacket);

// Interpret serialized byte data in frame and write to packet
unsigned int macReadPhyLength(unsigned char* frame);
unsigned int macReadDataLength(unsigned char* frame);
unsigned int macReadFrame(unsigned char* frame, MacPacket packet);

#endif
