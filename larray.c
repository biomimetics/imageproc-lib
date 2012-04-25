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
* Managed Array Data Structure
*
* by Humphrey Hu
*
* v 0.2
*
* Revisions:
*  Humphrey Hu         2012-02-04    Initial implementation
*  Humphrey Hu         2012-02-06    Added size querying                
*/

#include "larray.h"
#include <stdlib.h>

// ==== Function Prototypes ===================================================
unsigned int findEmpty(LinArrayItem item, void *args);

// ==== Public Functions ======================================================
LinArray larrayCreate(unsigned int size) {

    LinArray larray;
    
    larray = (LinArray) malloc(sizeof(LinArrayStruct));
    if(larray == NULL) { return NULL; }

    larray->items = (LinArrayItem*) calloc(size, sizeof(LinArrayItem));
    if(larray->items == NULL) {
        free(larray);
        return NULL;
    }
    
    larray->max_size = size;
    larray->size = 0;
    return larray;
    
}

LinArrayItem larrayReplace(LinArray larray, unsigned int index, LinArrayItem item) {

    LinArrayItem orig;

    if(index >= larray->max_size) { return NULL; }
    
    orig = larray->items[index];
    larray->items[index] = item;

    if(item == NULL) { larray->size--; }
    if(orig == NULL) { larray->size++; }

    return orig;

}

LinArrayItem larrayRetrieve(LinArray larray, unsigned int index) {

    if(index >= larray->max_size) { return NULL; }
    
    return larray->items[index];

}

unsigned int larrayFindFirst(LinArray larray, LinArrayItemTest equals, 
                            void *args, unsigned int *index,
                            LinArrayItem *item) {

    return larrayFindN(larray, equals, args, index, item, 1);
    
}

unsigned int larrayFindN(LinArray larray, LinArrayItemTest equals,
                        void *args, unsigned int *indices, 
                        LinArrayItem *items, unsigned int n) {
                        
    unsigned int i, j;
    LinArrayItem curr;
    
    j = 0;
    
    for(i = 0; i < larray->max_size; i++) {
        curr = larray->items[i];
        if(equals(curr, args)) {
            indices[j] = i;
            items[j] = curr;
            j++;
        }
        if(j >= n) {
            break;
        }    
    }
    return j;
                        
}

unsigned int larrayFindEmpty(LinArray larray, unsigned int *index) {

    LinArrayItem dummy;

    return larrayFindFirst(larray, &findEmpty, NULL, index, &dummy);
    
}

unsigned int larrayIsEmpty(LinArray larray) {

    return larray->size == 0;

}

unsigned int larrayIsFull(LinArray larray) {

    return larray->size == larray->max_size;

}

unsigned int larrayGetSize(LinArray larray) {

    return larray->size;

}

unsigned int larrayGetMaxSize(LinArray larray) {

    return larray->max_size;
    
}

// ==== Private Functions =====================================================

unsigned int findEmpty(LinArrayItem item, void *args) {

    return item == NULL;

}


