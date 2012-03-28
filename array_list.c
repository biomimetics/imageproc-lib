/*
 * Copyright (c) 2012, Regents of the University of California
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
 * Array-based List
 *
 * by Humphrey Hu
 *
 * v 0.1
 *
 * Revisions:
 *  Humphrey Hu         2012-02-04    Initial implementation
 *                      
 * Notes:
 *  - Index convention:
 *      items[head] is the first item in the queue
 *      items[tail] is the last item in the queue
 *  - Exception:    
 *      items[head] == items[tail] == NULL when the queue is empty
 */
 
#include "array_list.h"
#include <stdlib.h>
#include "utils.h"

// ================ Function Stubs ============================================
static inline unsigned int alistNextIndex(ArrayList alist, unsigned int i);
static inline unsigned int alistPrevIndex(ArrayList alist, unsigned int i);

// ================ Public Functions ==========================================
ArrayList alistInit(unsigned int max_size) {

    ArrayList alist = (ArrayList) malloc(sizeof(ArrayListStruct));
    
    if(alist == NULL) { return alist; }   // Check for allocation failure

    alist->items = (ArrayListItem*) calloc(max_size, sizeof(ArrayListItem)); // Init to zeros
    
    if(alist->items == NULL) {
        alistDelete(alist);
        return NULL;
    }
    
    alist->max_size = max_size;
    alist->size = 0;
    alist->head = 0;
    alist->tail = 0;
    return alist;
    
}

void alistDelete(ArrayList alist) {

    if(alist != NULL) {
    
        if(alist->items != NULL) {
            free(alist->items);
        }
        free(alist);
        
    }
    
}

unsigned int alistAddTail(ArrayList alist, ArrayListItem item) {

    // Can't add NULL items since NULL return is reserved
    if(item == NULL) { 
        return 0; 
    }
    
    CRITICAL_SECTION_START
    // Can't add if the queue is full
    if(alistIsFull(alist)) {
        CRITICAL_SECTION_END
        return 0; 
    }
    
    if(!alistIsEmpty(alist)) {
        alist->tail = alistNextIndex(alist, alist->tail);  // Find the new tail
    }
    alist->items[alist->tail] = item;   // Assign the item
    alist->size++;         // Update the size
    CRITICAL_SECTION_END
    return 1;
    
}

unsigned int alistAddHead(ArrayList alist, ArrayListItem item) {

    // Can't add NULL items since NULL return is reserved
    if(item == NULL) { 
        return 0; 
    }
    
    CRITICAL_SECTION_START
    // Can't add if the queue is full
    if(alistIsFull(alist)) {
        CRITICAL_SECTION_END
        return 0; 
    }
    
    if(!alistIsEmpty(alist)) {
        alist->head = alistPrevIndex(alist, alist->head);  // Find the new head
    }
    alist->items[alist->head] = item;   // Assign the item
    alist->size++;         // Update the size
    CRITICAL_SECTION_END
    return -1;
    
}

ArrayListItem alistPopTail(ArrayList alist) {

    ArrayListItem item;
    
    CRITICAL_SECTION_START
    if(alistIsEmpty(alist)) {
        CRITICAL_SECTION_END
        return NULL; 
    }
    
    item = alist->items[alist->tail]; // Retrieve item
    alist->items[alist->tail] = NULL; // Clear entry
    
    if(alist->tail != alist->head) {  // Size > 1
        alist->tail = alistPrevIndex(alist, alist->tail); // Update tail
    }
    alist->size--;
    CRITICAL_SECTION_END
    return item;
    
}

ArrayListItem alistPopHead(ArrayList alist) {

    ArrayListItem item;
    
    CRITICAL_SECTION_START
    if(alistIsEmpty(alist)) {
        CRITICAL_SECTION_END
        return NULL; 
    }
    
    item = alist->items[alist->head]; // Retrieve item
    alist->items[alist->head] = NULL; // Clear entry
    
    if(alist->head != alist->tail) {  // Size > 1
        alist->head = alistNextIndex(alist, alist->head); // Update head
    }
    alist->size--;
    CRITICAL_SECTION_END
    return item;
    
}

ArrayListItem alistPeekTail(ArrayList alist) {

    ArrayListItem item;
    
    CRITICAL_SECTION_START

    if(alistIsEmpty(alist)) {
        CRITICAL_SECTION_END
        return NULL; 
    }        
    
    item = alist->items[alist->tail];

    CRITICAL_SECTION_END
    
    return item;
    
}

ArrayListItem alistPeekHead(ArrayList alist) {

    ArrayListItem item;

    CRITICAL_SECTION_START
    
    if(alistIsEmpty(alist)) {
        CRITICAL_SECTION_END
        return NULL; 
    }
    
    item = alist->items[alist->head];

    CRITICAL_SECTION_END
    
    return item;
    
}

unsigned int alistIsEmpty(ArrayList alist) {
    
    return alist->size == 0;

}

unsigned int alistIsFull(ArrayList alist) {

    return alist->size == alist->max_size;

}

unsigned int alistGetSize(ArrayList alist) {

    return alist->size;

}    

// ================== PRIVATE FUNCTIONS ==========================
/**
 * Returns the next valid index with wraparound in a queue from a position
 *
 * @param alist FastQueue to search in
 * @param i Position to start from
 * @return next valid index
 */
static inline unsigned int alistNextIndex(ArrayList alist, unsigned int i) {

    if(i == (alist->max_size - 1)) {
        return 0;
    }
    return i + 1;

}

/**
 * Returns the previous valid index with wraparound in a queue from a position
 *
 * @param alist FastQueue to search in
 * @param i Position to start from
 * @return previous valid index
 */
static inline unsigned int alistPrevIndex(ArrayList alist, unsigned int i) {

    if(i == 0) {
        return alist->max_size - 1;
    }
    return i - 1;

}
