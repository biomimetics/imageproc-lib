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
* v 0.1
*
* Revisions:
*  Humphrey Hu         2012-04-04    Initial implementation
*                      
*/

#ifndef __LARRAY_H
#define __LARRAY_H

typedef void* LinArrayItem;

typedef struct {
    unsigned int max_size;
    unsigned int size;
    LinArrayItem* items;
} LinArrayStruct;

typedef LinArrayStruct* LinArray;

/**
 * Item comparison typedef. The method should handle NULL pointers gracefully.
 */
typedef unsigned int (*LinArrayItemCompare)(LinArrayItem i1, LinArrayItem i2);

/**
 * Item test typedef. The method should handle NULL pointers gracefully.
 */
typedef unsigned int (*LinArrayItemTest)(LinArrayItem item, void *args);

/**
 * Create a linear array.
 * @param size - Maximum capacity of linear array
 * @return Linear array object
 */
LinArray larrayCreate(unsigned int size);

/**
 * Delete a linear array, freeing up its resources.
 * @param larray - Linear array object to delete
 */
void larrayDelete(LinArray larray);

/**
 * Swap current item at index with argument item.
 * @param larray - Linear array object to operate on
 * @param index - Position to swap at
 * @param item - Item to insert into array at position
 * @return - Previous item at index position
 */
LinArrayItem larrayReplace(LinArray larray, unsigned int index, 
                                LinArrayItem item);

/**
 * Return current item at index. Does not modify linear array.
 * @param larray - Linear array object to operate on
 * @param index - Position to retrieve from
 * @return - Item currently at index position
 */
LinArrayItem larrayRetrieve(LinArray larray, unsigned int index);

/**
 * Find first item meeting specified criteria.
 * @param larray - Linear array object to operate on
 * @param equals - Pointer to item test function
 * @param index - Pointer to store index of matching item in
 * @param item - Pointer to store matching item in
 * @return - 1 if match found, else 0
 */
unsigned int larrayFindFirst(LinArray larray, LinArrayItemTest equals,
                            void *args, unsigned int *index, 
                            LinArrayItem *item);

/**
 * Find first N items meeting specified criteria
 * @param larray - Linear array object to operate on
 * @param equals - Pointer to item test function
 * @param indices - Pointer to array to store index of matching items in
 * @param items - Pointer to array to store matching items in
 * @param n - Number of matches to try and find
 * @return - Number of matches found
 */
unsigned int larrayFindN(LinArray larray, LinArrayItemTest equals,
                        void *args, unsigned int *indices, 
                        LinArrayItem *items, unsigned int n);

/**
 * Find first item 
 * @param larray - Linear array object to operate on
 * @param index - Pointer to store matching item in
 */ 
unsigned int larrayFindEmpty(LinArray larray, unsigned int *index);                            

unsigned int larrayIsEmpty(LinArray larray);

unsigned int larrayIsFull(LinArray larray);

unsigned int larrayGetSize(LinArray larray);

unsigned int larrayGetMaxSize(LinArray larray);

#endif
