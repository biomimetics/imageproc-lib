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
 * Linked List Queue (FIFO) for generic elements
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
 * #include "queue.h"
 *
 * int max_size = 2;
 * unsigned char *item;
 * int command = 0x7f; 
 *
 * Queue que = queueInit(max_size);
 *     
 * queuePush(que, "Hello World");
 * queuePush(que, &command);
 * queuePush(que, "This string will not be inserted.");
 * 
 * // size of the queue is now 2
 * 
 * // queuePop returns a generic type pointer.
 * // So, you must use a cast operator.
 * item = (unsigned char*)queuePop(que);  
 * 
 * command = 0x78;
 * int *cmd = (int *)queuePop(que);
 *
 * // the current value of *cmd is 0x78;
 *
 *
 */

#ifndef __QUEUE_H
#define __QUEUE_H

//typedef generic pointer type, Item;
typedef void* Item;

// Node structure
typedef struct node {
    Item item;
    struct node *next;
} Node;

// Queue structure
typedef struct {
    Node *front;
    Node *rear;
    unsigned char mode;
    int size;
    int max_size;
} QueueStruct;

typedef QueueStruct* Queue;


/******************************************************************************
* Function Name : queueInit                                     
* Description   : Create an instance of Queue with a given maximum size.
* Parameters    : The maximum size of items in the queue. If the value of 
*                 max_size is zero, there is no size limitation for the queue. 
* Return Value  : An instance of Queue
*******************************************************************************/
Queue queueInit(int max_size);

/******************************************************************************
* Function Name : queuePush                                     
* Description   : Insert an item at the end of queue.
* Parameters    : A queue and an item to be inserted. The item can be any 
*                 pointer type
* Return Value  : If the queue is full, the first item in the queue will be 
*                 returned, and the incoming item will be inserted at the end.
*******************************************************************************/
Item queuePush(Queue q, Item item);

/******************************************************************************
* Function Name : queueAppend                                     
* Description   : Insert an item at the end of queue.
* Parameters    : A queue and an item to be inserted. The item can be any 
*                 pointer type
* Return Value  : If the queue is full, the item will not be not inserted and 
*                 0 is returned. If the item is successfully inserted, 
*                 1 will be returned.
*******************************************************************************/
unsigned int queueAppend(Queue q, Item item);


/******************************************************************************
* Function Name : queuePop
* Description   : Pop out the item from the front of queue.
* Parameters    : A queue...
* Return Value  : The item located at the front of the queue. The type of the 
*                 returned item has a generic type (void*). It is user's
*                 reponsiblity to cast the item with the right type. Read the
*                 Usage.
*******************************************************************************/
Item queuePop(Queue q);

/******************************************************************************
* Function Name : queueGetFront
* Description   : Get first item in queue without removing it
* Parameters    : A queue...
* Return Value  : The item located at the front of the queue. The type of the
*                 returned item has a generic type (void*). It is user's
*                 reponsiblity to cast the item with the right type. Read the
*                 Usage.
*******************************************************************************/
Item queueGetFront(Queue q);

/******************************************************************************
* Function Name : queueIsFull
* Description   : Test if the queue is full.
* Parameters    : A queue...
* Return Value  : 1 will be returned if the queue is full. Otherwise, 0 will be
*                 returned.
*******************************************************************************/
int queueIsFull(Queue q);

/******************************************************************************
* Function Name : queueIsEmpty
* Description   : Test if the queue is empty.
* Parameters    : A queue...
* Return Value  : 1 will be returned if the queue is empty. Otherwise, 0 will be
*                 returned.
*******************************************************************************/
int queueIsEmpty(Queue q);

/******************************************************************************
* Function Name : queueGetSize
* Description   : Get the current size of the queue.
* Parameters    : A queue...
* Return Value  : the current size of the queue.
*******************************************************************************/
int queueGetSize(Queue q);

#endif  // __QUEUE_H




