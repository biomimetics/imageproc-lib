/*
 * Copyright (c) 2010-2011, Regents of the University of California
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
 * IEEE 802.15.4 PPDU and MPDU Utilities
 *
 * by Humphrey Hu
 *
 * v.beta
 *
 * Revisions:
 *  Humphrey Hu         2011-07-27    Initial implementation
 *
 * Notes:
 *
 * To Do:
 *
 */

 #include "mac_packet.h"
 #include "payload.h"
 #include <stdlib.h>
#include <string.h>

 #define MAC_PHY_LEN_POS            (0)
 #define MAC_FCF_LB_POS             (1)
 #define MAC_FCF_HB_POS             (2)
 #define MAC_SEQ_NUM_POS            (3)

 #define MPDU_HEADER_BASE_LENGTH    (3)
 #define MAC_CRC_LENGTH             (2)

 MacPacket macCreateEmpty(void) {

    MacPacket packet = (MacPacket)malloc(sizeof(MacPacketStruct));
    if(packet == NULL) {
        return NULL;
    }

    // Default values
    packet->frame_ctrl.bits.sec_en = MAC_SEC_DISABLED;
    packet->frame_ctrl.bits.frm_pending = MAC_FRAME_COMPLETE;
    packet->frame_ctrl.bits.ack_req = MAC_ACK_REQUIRED;
    packet->frame_ctrl.bits.pan_id_comp = MAC_INTRAPAN_ON;
    packet->frame_ctrl.bits.dest_addr_mode = MAC_DEST_ADDR_MODE_16BIT;
    packet->frame_ctrl.bits.frm_version = MAC_FIRMWARE_2006;
    packet->frame_ctrl.bits.src_addr_mode = MAC_SRC_ADDR_MODE_16BIT;

    return packet;

}

MacPacket macCreateDataPacket() {

    MacPacket packet = macCreateEmpty();
    if(packet == NULL) {
        return NULL;
    }
    packet->frame_ctrl.bits.packet_type = MAC_PACKET_TYPE_DATA;
    return packet;

}

MacPacket macCreateBeaconPacket() {

    MacPacket packet = macCreateEmpty();
    if(packet == NULL) {
        return NULL;
    }
    packet->frame_ctrl.bits.packet_type = MAC_PACKET_TYPE_BEACON;
    return packet;

}

MacPacket macCreateCommandPacket() {

    MacPacket packet = macCreateEmpty();
    if(packet == NULL) {
        return NULL;
    }
    packet->frame_ctrl.bits.packet_type = MAC_PACKET_TYPE_COMMAND;
    return packet;

}

void macDeletePacket(MacPacket packet) {

    free(packet);

}

void macSetSeqNum(MacPacket packet, char seq_num) {

    packet->seq_num = seq_num;

}

void macSetDestAddr(MacPacket packet, unsigned int dest_addr) {

    packet->dest_addr.val = dest_addr;

}

void macSetDestPan(MacPacket packet, unsigned int dest_pan) {

    packet->dest_pan_id.val = dest_pan;

}

void macSetSrc(MacPacket packet, unsigned int src_pan_id, unsigned int src_addr) {

    packet->src_pan_id.val = src_pan_id;
    packet->src_addr.val = src_addr;

}

unsigned int macGetSrcAddr(MacPacket packet) {

    return packet->src_addr.val;

}

unsigned int macGetSrcPan(MacPacket packet) {

    unsigned char src_pan_compression;

    src_pan_compression = packet->frame_ctrl.bits.src_addr_mode
        && packet->frame_ctrl.bits.dest_addr_mode && packet->frame_ctrl.bits.pan_id_comp;

    if(src_pan_compression) {
        return packet->dest_pan_id.val;
    }
    return packet->src_pan_id.val;
}

void macSetPayload(MacPacket packet, Payload pld) {

    packet->payload = pld;
    packet->payload_length = payGetPayloadLength(pld);

}

Payload macGetPayload(MacPacket packet) {

    return packet->payload;

}

unsigned int macReadPhyLength(unsigned char* frame) {

    return frame[0];

}

unsigned int macReadDataLength(unsigned char* frame) {

    MacPacketStruct packet;
    unsigned char header_length, phy_length, src_pan_compression;

    header_length = MPDU_HEADER_BASE_LENGTH; // Account for phy_length and frame_ctrl bytes
    phy_length = frame[MAC_PHY_LEN_POS]; // Read PHY payload length from PPDU
    packet.frame_ctrl.val.byte.LB = frame[MAC_FCF_LB_POS];
    packet.frame_ctrl.val.byte.HB = frame[MAC_FCF_HB_POS];

    if(packet.frame_ctrl.bits.dest_addr_mode == MAC_DEST_ADDR_MODE_NONE) {
        header_length += 0; // No destination address
    } else if(packet.frame_ctrl.bits.dest_addr_mode == MAC_DEST_ADDR_MODE_16BIT) {
        header_length += 4; // Destination address + PAN
    } else if(packet.frame_ctrl.bits.dest_addr_mode == MAC_DEST_ADDR_MODE_64BIT) {
        header_length += 10; // Destination address + PAN
    }

    // If both addresses are present and intra-PAN mode is on, source PAN is dropped
    src_pan_compression = packet.frame_ctrl.bits.src_addr_mode
        && packet.frame_ctrl.bits.dest_addr_mode && packet.frame_ctrl.bits.pan_id_comp;

    if(packet.frame_ctrl.bits.src_addr_mode == MAC_SRC_ADDR_MODE_NONE) {
        header_length += 0; // No source address
    } else if(packet.frame_ctrl.bits.src_addr_mode == MAC_SRC_ADDR_MODE_16BIT) {
        if(src_pan_compression) {
            header_length += 0; // No source PAN
        } else {
            header_length += 2; // Source pan included
        }
        header_length += 2; // Plus source address
    }
    else if(packet.frame_ctrl.bits.src_addr_mode == MAC_SRC_ADDR_MODE_64BIT) {
        if(src_pan_compression) {
            header_length += 0; // No source PAN
        } else {
            header_length += 2; // Source pan included
        }
        header_length += 8; // Plus source address
    }

    return phy_length - header_length - MAC_CRC_LENGTH; // Calculate payload length

}

unsigned int macReadFrame(unsigned char* frame, MacPacket packet) {

    unsigned char i, header_length, phy_length, src_pan_compression;
    unsigned int data_length;
    Payload pld;

    if(packet == NULL) { return 0; }
    pld = macGetPayload(packet);
    if(pld == NULL) { return 0; }

    header_length = MPDU_HEADER_BASE_LENGTH;
    phy_length = frame[MAC_PHY_LEN_POS]; // Read PHY payload length from PPDU
    packet->frame_ctrl.val.byte.LB = frame[MAC_FCF_LB_POS];
    packet->frame_ctrl.val.byte.HB = frame[MAC_FCF_HB_POS];
    packet->seq_num = frame[MAC_SEQ_NUM_POS];

    i = MAC_SEQ_NUM_POS + 1; // Begin counting bytes

    // Read destination addresses
    if(packet->frame_ctrl.bits.dest_addr_mode == MAC_DEST_ADDR_MODE_NONE) {
        packet->dest_pan_id.byte.LB = 0;
        packet->dest_pan_id.byte.HB = 0;
        packet->dest_addr.byte.LB = 0;
        packet->dest_addr.byte.HB = 0;
    } else if(packet->frame_ctrl.bits.dest_addr_mode == MAC_DEST_ADDR_MODE_16BIT) {
        header_length += 4;
        packet->dest_pan_id.byte.LB = frame[i++];
        packet->dest_pan_id.byte.HB = frame[i++];
        packet->dest_addr.byte.LB = frame[i++];
        packet->dest_addr.byte.HB = frame[i++];
    } else if(packet->frame_ctrl.bits.dest_addr_mode == MAC_DEST_ADDR_MODE_64BIT) {
        // Not supported by packet object yet!
        packet->dest_pan_id.byte.LB = 0;
        packet->dest_pan_id.byte.HB = 0;
        packet->dest_addr.byte.LB = 0;
        packet->dest_addr.byte.HB = 0;
        header_length += 10;
    }

    // If both addresses are present and intra-PAN mode is on, source PAN is dropped
    src_pan_compression = packet->frame_ctrl.bits.src_addr_mode
        && packet->frame_ctrl.bits.dest_addr_mode && packet->frame_ctrl.bits.pan_id_comp;

    // Read source addresses
    if(packet->frame_ctrl.bits.src_addr_mode == MAC_SRC_ADDR_MODE_NONE) {
        packet->src_pan_id.byte.LB = 0;
        packet->src_pan_id.byte.HB = 0;
        packet->src_addr.byte.LB = 0;
        packet->src_addr.byte.HB = 0;
    } else if(packet->frame_ctrl.bits.src_addr_mode == MAC_SRC_ADDR_MODE_16BIT) {
        if(src_pan_compression) {
            header_length += 0;
            packet->src_pan_id.byte.LB = 0; //packet->dest_pan_id.byte.LB;
            packet->src_pan_id.byte.HB = 0; //packet->dest_pan_id.byte.HB;
        } else {
            header_length += 2;
            packet->src_pan_id.byte.LB = frame[i++];
            packet->src_pan_id.byte.HB = frame[i++];
        }
        header_length += 2;
        packet->src_addr.byte.LB = frame[i++];
        packet->src_addr.byte.HB = frame[i++];
    }
    else if(packet->frame_ctrl.bits.src_addr_mode == MAC_SRC_ADDR_MODE_64BIT) {
        // Not supported by packet object yet!
        if(src_pan_compression) {
            header_length += 0;
            packet->src_pan_id.byte.LB = 0;
            packet->src_pan_id.byte.HB = 0;
        } else {
            header_length += 2;
            packet->src_pan_id.byte.LB = frame[i++];
            packet->src_pan_id.byte.HB = frame[i++];
        }
        header_length += 8;
    }

    data_length = phy_length - header_length - MAC_CRC_LENGTH; // Calculate payload length

    memcpy(payToString(pld), frame + i, data_length);

    pld->data_length = data_length - PAYLOAD_HEADER_LENGTH; // macSetPayload requires accurate payload length
    macSetPayload(packet, pld);

    return 1;

 }
