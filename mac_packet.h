#ifndef __MACPACKET_H
#define __MACPACKET_H

#include "generic_typedefs.h"
#include "payload.h"

typedef struct
{
    union
    {
        WordVal    val;   
        struct
        {
            word        packet_type     : 3;        // type of packet. Possible types are
                                                    // * PACKET_TYPE_DATA - Data type
                                                    // * PACKET_TYPE_COMMAND -  Command type
                                                    // * PACKET_TYPE_ACK -  Acknowledgement type
                                                    // * PACKET_TYPE_RESERVE - Reserved type
            word        sec_en          : 1;        // 1: secure the MAC payload, 0: send plain text
                                                    // We are *not* currently implementing secured transmission
            word        frm_pending     : 1;        // 1: sending device has more data for recipient, 0: no more data
            word        ack_req         : 1;        // 1: acknowledgement required, 0: no acknowldgement
            
            word        pan_id_comp     : 1;        // 1: PAN ID compression subfield 
            word        reserved        : 3;
            word        dest_addr_mode  : 2;        // 0: PAN ID and dest addr not present, 
                                                    // 1: reserved, 
                                                    // 2: address field contains 16 bit short address 
                                                    // 3: address field contains 64 bit extended address - NOT IMPLEMENTED
            word        frm_version     : 2;        // 0: IEEE 802.15.4-2003 
                                                    // 1: IEEE 802.15.4
            word        src_addr_mode   : 2;        // 0: PAN ID and src addr not present, 
                                                    // 1: reserved, 
                                                    // 2: address field contains 16 bit short address 
                                                    // 3: address field contains 64 bit extended address 
        } bits;
    } frame_ctrl;

    byte        seq_num;
    WordVal     dest_pan_id;  
    WordVal     dest_addr;   //For the moment we are not implementing extended 64-bit addresses
    WordVal     src_pan_id;
    WordVal     src_addr;    //For the moment we are not implementing extended 64-bit addresses
    //byte * auxSecHdr  //Placeholder for auxiliary security header when we
                        //get around to implementing it.
    Payload     payload;
    byte        payload_length;



} MacPacketStruct;

typedef MacPacketStruct* MacPacket;

#endif
