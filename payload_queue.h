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
 * Queue (FIFO) for Payload
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2010-08-03     Initial release
 *                      
 * Notes:
 *
 * Usage:
 *
 * #include "payload_queue.h"
 * #include "payload.h"
 *
 * int max_size = 2;
 * unsigned char *item;
 * unsigned char type; 
 * unsigned char status; 
 *
 * PayQueue que = pqInit(max_size);
 *     
 * unsigned char *str1 = "  Ronen can open locks in 10 sec without a key";
 * unsigned char *str2 = "  Ronen can jump 5ft.";
 * unsigned char *str3 = "  Go go go!";
 * 
 * str1[0] = str2[0] = STATUS_SKEPTICAL;
 * str1[1] = str2[1] = TYPE_BLUFFING;
 *
 * Payload pay;
 * Payload pay1 = payCreateFromString(str1, strlen(str1));
 * Payload pay2 = payCreateFromString(str2, strlen(str2));
 * Payload pay3 = payCreateFromString(str3, strlen(str3));
 *
 * pqPush(que, pay1);
 * pqPush(que, pay2);
 * pqPush(que, pay3);
 * 
 * // size of the queue is still now 2
 * // and pay1 is already dropped.
 * 
 * pay = pqPop(que);  // pay is now pay2
 * pay = pqPop(que);  // pay is now pay3
 *
 */

#ifndef __PAYLOAD_QUEUE_H
#define __PAYLOAD_QUEUE_H

#include "queue.h"
#include "payload.h"

//typedef generic pointer type, Item;
typedef Queue PayQueue;

/******************************************************************************
* Function Name : queueInit                                     
* Description   : Create an instance of Queue with a given maximum size.
* Parameters    : The maximum size of items in the queue. Incoming items are 
*                 ignored if the queue is full
* Return Value  : An instance of Queue
*******************************************************************************/
PayQueue pqInit(int max_size);

/******************************************************************************
volatile PayQueue
* Function Name : pqPush                                     
* Description   : Insert an item at the end of queue.
* Parameters    : A queue and an item to be inserted. The item can be any 
*                 pointer type
* Return Value  : If the queue is full, the first item in the queue will be 
*                 dropped
*******************************************************************************/
void pqPush(PayQueue pq, Payload pld);

/******************************************************************************
* Function Name : pqPop
* Description   : Pop out the item from the front of queue.
* Parameters    : A queue...
* Return Value  : The Payload struct located at the front of the queue. 
*******************************************************************************/
Payload pqPop(PayQueue queue);

/******************************************************************************
* Function Name : pqIsFull
* Description   : Test if the queue is full.
* Parameters    : A queue...
* Return Value  : 1 will be returned if the queue is full. Otherwise, 0 will be
*                 returned.
*******************************************************************************/
int pqIsFull(PayQueue queue);

/******************************************************************************
* Function Name : pqIsEmpty
* Description   : Test if the queue is empty.
* Parameters    : A queue...
* Return Value  : 1 will be returned if the queue is empty. Otherwise, 0 will be
*                 returned.
*******************************************************************************/
int pqIsEmpty(PayQueue queue);

/******************************************************************************
* Function Name : pqGetSize
* Description   : Get the current size of the queue.
* Parameters    : A queue...
* Return Value  : the current size of the queue.
*******************************************************************************/
int pqGetSize(PayQueue queue);

#endif  // __PAYLOAD_QUEUE_H




