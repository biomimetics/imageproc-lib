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
 */

#ifndef __CARRAY_H
#define __CARRAY_H

typedef void* CircArrayItem;

typedef struct {
    CircArrayItem* items;
    unsigned int head;
    unsigned int tail;
    unsigned int size;
    unsigned int max_size;
} CircArrayStruct;

typedef CircArrayStruct *CircArray;

// Create a queue/stack
CircArray carrayCreate(unsigned int max_size);
void carrayDelete(CircArray fq);

// ========== Adding ============
// Add an object to the back
unsigned int carrayAddTail(CircArray fq, CircArrayItem item);
// Add an object to the front
unsigned int carrayAddHead(CircArray fq, CircArrayItem item);

// ========== Removing ============
// Remove an object from the back
CircArrayItem carrayPopTail(CircArray fq);
// Remove an object from the front
CircArrayItem carrayPopHead(CircArray fq);

// ========== Querying ============
// Retrieve an object from the back
CircArrayItem carrayPeekTail(CircArray fq);
// Retrieve an object from the front
CircArrayItem carrayPeekHead(CircArray fq);

// Check if the queue is empty
unsigned int carrayIsEmpty(CircArray fq);
// Check if the queue is full
unsigned int carrayIsFull(CircArray fq);
// Get number of items in the queue
unsigned int carrayGetSize(CircArray fq);


#endif // __ALIST_H
