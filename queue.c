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
 */

#include "queue.h"
#include "utils.h"
#include <xc.h>
#include <stdio.h>      // for NULL
#include <stdlib.h>     // for malloc


#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START	char saved_ipl; SET_AND_SAVE_CPU_IPL(saved_ipl, 7);
#define CRITICAL_SECTION_END RESTORE_CPU_IPL(saved_ipl);	
#endif

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

Queue queueInit(int max_size) {
    // begin critical section
    CRITICAL_SECTION_START;

    Queue q = (Queue)malloc(sizeof(QueueStruct));

    if (q == NULL) return NULL;

    q->front = q->rear = NULL;
    q->size = 0;
    q->max_size = max_size;

    // end critical section    
    CRITICAL_SECTION_END;
    return q;
}


Item queuePush(Queue q, Item item) {

    Item ret = NULL;

    // begin critical section
    CRITICAL_SECTION_START;

    if (queueIsFull(q)) {
        ret = queuePop(q);
    }

    queueAppend(q, item);

    // end critical section    
    CRITICAL_SECTION_END;

    return ret;

}


unsigned int queueAppend(Queue q, Item item) {

    // begin critical section
    CRITICAL_SECTION_START;

    if (queueIsFull(q)) {
        // end critical section    
        CRITICAL_SECTION_END;
        return 0;
    }

    Node* new_node = (Node*)malloc(sizeof(Node));
    if (new_node == NULL) {
        // end critical section    
        CRITICAL_SECTION_END;
        return 0;
    }


    new_node->item = item;
    new_node->next = NULL;
    if (queueIsEmpty(q)) {
        q->front = new_node;    // insert the item at the front
    } else {
        q->rear->next = new_node; // append the item at the end
    }
    q->rear = new_node;     // relocated the rear node
    q->size++;

    // end critical section    
    CRITICAL_SECTION_END;

    return 1;

}


Item queuePop(Queue q) {

    // begin critical section
    CRITICAL_SECTION_START;

    if(queueIsEmpty(q)) {
        // end critical section    
        CRITICAL_SECTION_END;

        return NULL;
    }

    Node *front_node = q->front;
    Item item = front_node->item;

    q->front = front_node->next;  // reloacte the front node;
    free(front_node);

    q->size--;

    if(queueIsEmpty(q)) q->rear = NULL;

    // end critical section    
    CRITICAL_SECTION_END;

    return item;

}

Item queueGetFront(Queue q) {

    if(queueIsEmpty(q)) {
        return NULL;
    }

    Item item = q->front->item;
    //Item item = front_node->item;

    return item;
}


int queueIsFull(Queue q) {
    if(q->max_size == 0) return 1; // max_size == 0 means no max.
    return q->size == q->max_size;
}

int queueIsEmpty(Queue q) {
    return q->size == 0;
}


int queueGetSize(Queue q) {
    return q->size;
}










