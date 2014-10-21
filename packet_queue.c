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
 * Queue (FIFO) for Packets
 *
 * by Aaron M. Hoover
 *
 * v.beta
 *
 * Revisions:
 *  Aaron M. Hoover 2010-09-02     Initial release
 *                      
 * Notes:
 *
 */

#include <xc.h>
#include "queue.h"
#include "packet_queue.h"
#include "radio.h"
#include "utils.h"
#include <stdio.h>      // for NULL
#include <stdlib.h>     // for malloc


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

PacketQueue pktqInit(int max_size) {

    Queue pq = queueInit(max_size);
    return pq;

}

void pktqPush(PacketQueue pq, MacPacket mp) {

    MacPacket p;

    if (queueIsFull(pq)) {
        p = (MacPacket)queuePop(pq);
        payDelete(p->payload);
        macDeletePacket(p);
    }

    queueAppend(pq, mp);

}

MacPacket pktqPop(PacketQueue queue) {

    return (MacPacket)queuePop(queue);

}

MacPacket pktqGetFront(PacketQueue queue) {

    return (MacPacket)queueGetFront(queue);
}

int pktqIsFull(PacketQueue queue) {

    return queueIsFull(queue);

}

int pktqIsEmpty(PacketQueue queue) {

    return queueIsEmpty(queue);

}


int pktqGetSize(PacketQueue queue) {

    return queueGetSize(queue);

}
