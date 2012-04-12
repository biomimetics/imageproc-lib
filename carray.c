/**
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
 * Circular Array Data Structure
 *
 * by Humphrey Hu
 *
 * v.0.1
 *
 * Revisions:
 *  Humphrey Hu         2012-02-04    Initial implementation
 *
 * Notes:
 *  - Index convention:
 *      items[head] is the first item in the queue
 *      items[tail] is the last item in the queue
 *  - Empty convention:
 *      items[head] == items[tail] == NULL when the queue is empty
 */

#include "carray.h"
#include <stdlib.h>
#include "utils.h"

// ================ Function Stubs ============================================
static inline unsigned int carrayNextIndex(CircArray carray, unsigned int i);
static inline unsigned int carrayPrevIndex(CircArray carray, unsigned int i);

// ================ Public Functions ==========================================
CircArray carrayCreate(unsigned int max_size) {

    CircArray carray = (CircArray) malloc(sizeof(CircArrayStruct));

    if(carray == NULL) { return carray; }   // Check for allocation failure

    carray->items = (CircArrayItem*) calloc(max_size, sizeof(CircArrayItem)); // Init to zeros

    if(carray->items == NULL) {
        carrayDelete(carray);
        return NULL;
    }

    carray->max_size = max_size;
    carray->size = 0;
    carray->head = 0;
    carray->tail = 0;
    return carray;

}

void carrayDelete(CircArray carray) {

    if(carray != NULL) {

        if(carray->items != NULL) {
            free(carray->items);
        }
        free(carray);

    }

}

unsigned int carrayAddTail(CircArray carray, CircArrayItem item) {

    // Can't add NULL items since NULL return is reserved
    if(item == NULL) {
        return 0;
    }

    CRITICAL_SECTION_START
    // Can't add if the queue is full
    if(carrayIsFull(carray)) {
        CRITICAL_SECTION_END
        return 0;
    }

    if(!carrayIsEmpty(carray)) {
        carray->tail = carrayNextIndex(carray, carray->tail);  // Find the new tail
    }
    carray->items[carray->tail] = item;   // Assign the item
    carray->size++;         // Update the size
    CRITICAL_SECTION_END
    return 1;

}

unsigned int carrayAddHead(CircArray carray, CircArrayItem item) {

    // Can't add NULL items since NULL return is reserved
    if(item == NULL) {
        return 0;
    }

    CRITICAL_SECTION_START
    // Can't add if the queue is full
    if(carrayIsFull(carray)) {
        CRITICAL_SECTION_END
        return 0;
    }

    if(!carrayIsEmpty(carray)) {
        carray->head = carrayPrevIndex(carray, carray->head);  // Find the new head
    }
    carray->items[carray->head] = item;   // Assign the item
    carray->size++;         // Update the size
    CRITICAL_SECTION_END
    return -1;

}

CircArrayItem carrayPopTail(CircArray carray) {

    CircArrayItem item;

    CRITICAL_SECTION_START
    if(carrayIsEmpty(carray)) {
        CRITICAL_SECTION_END
        return NULL;
    }

    item = carray->items[carray->tail]; // Retrieve item
    carray->items[carray->tail] = NULL; // Clear entry

    if(carray->tail != carray->head) {  // Size > 1
        carray->tail = carrayPrevIndex(carray, carray->tail); // Update tail
    }
    carray->size--;
    CRITICAL_SECTION_END
    return item;

}

CircArrayItem carrayPopHead(CircArray carray) {

    CircArrayItem item;

    CRITICAL_SECTION_START
    if(carrayIsEmpty(carray)) {
        CRITICAL_SECTION_END
        return NULL;
    }

    item = carray->items[carray->head]; // Retrieve item
    carray->items[carray->head] = NULL; // Clear entry

    if(carray->head != carray->tail) {  // Size > 1
        carray->head = carrayNextIndex(carray, carray->head); // Update head
    }
    carray->size--;
    CRITICAL_SECTION_END
    return item;

}

CircArrayItem carrayPeekTail(CircArray carray) {

    CircArrayItem item;

    CRITICAL_SECTION_START

    if(carrayIsEmpty(carray)) {
        CRITICAL_SECTION_END
        return NULL;
    }

    item = carray->items[carray->tail];

    CRITICAL_SECTION_END

    return item;

}

CircArrayItem carrayPeekHead(CircArray carray) {

    CircArrayItem item;

    CRITICAL_SECTION_START

    if(carrayIsEmpty(carray)) {
        CRITICAL_SECTION_END
        return NULL;
    }

    item = carray->items[carray->head];

    CRITICAL_SECTION_END

    return item;

}

unsigned int carrayIsEmpty(CircArray carray) {

    return carray->size == 0;

}

unsigned int carrayIsFull(CircArray carray) {

    return carray->size == carray->max_size;

}

unsigned int carrayGetSize(CircArray carray) {

    return carray->size;

}

// ================== PRIVATE FUNCTIONS ==========================
/**
 * Returns the next valid index with wraparound in a queue from a position
 *
 * @param carray FastQueue to search in
 * @param i Position to start from
 * @return next valid index
 */
static inline unsigned int carrayNextIndex(CircArray carray, unsigned int i) {

    if(i == (carray->max_size - 1)) {
        return 0;
    }
    return i + 1;

}

/**
 * Returns the previous valid index with wraparound in a queue from a position
 *
 * @param carray FastQueue to search in
 * @param i Position to start from
 * @return previous valid index
 */
static inline unsigned int carrayPrevIndex(CircArray carray, unsigned int i) {

    if(i == 0) {
        return carray->max_size - 1;
    }
    return i - 1;

}
