/*
 * Copyright (c) 2010-2013, Regents of the University of California
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
 * Payload module
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek     2010-06-05    Initial release
 */

#include "payload.h"
#include "utils.h"
#include <stdlib.h>     // for malloc
#include <stdarg.h>     // variable number of arguments
#include <string.h>     // for memcpy

#define STATUS_POS              0
#define TYPE_POS                1

Payload payCreate(unsigned char data_length, unsigned char *data,
                  unsigned char status, unsigned char type) {

    Payload pld = payCreateEmpty(data_length);
    paySetStatus(pld, status);
    paySetType(pld, type);
    payAppendData(pld, 0, data_length, data);
    return pld;
}


Payload payCreateEmpty(unsigned char data_length)
{
    Payload pld = (Payload)malloc(sizeof(PayloadStruct));

    if ( pld == NULL ) return NULL;

    unsigned char* data = (unsigned char*)malloc(data_length + PAYLOAD_HEADER_LENGTH);

    if ( data == NULL )
    {
      free(pld);
      return NULL;
    }

    pld->pld_data = data;
    pld->data_length = data_length;
    pld->iter_index = 0;
    return pld;
}


Payload payClone(Payload pld) {
    return payCreate(payGetDataLength(pld), payGetData(pld), payGetStatus(pld), payGetType(pld));
}

unsigned char payNextElement(Payload pld) {
    if (pld->iter_index >= pld->data_length + PAYLOAD_HEADER_LENGTH) {
        return -1;
    }
    return pld->pld_data[pld->iter_index++];
}

void payInitIterator(Payload pld) {
    pld->iter_index = 0;
}

unsigned char payReadByte(Payload pld, unsigned char loc) {
    return pld->pld_data[loc+PAYLOAD_HEADER_LENGTH];
}

unsigned char* payToString(Payload pld) {
    return pld->pld_data;
}

void payAppendData(Payload pld, char loc,
                unsigned char data_length, unsigned char *data)
{
    memcpy(pld->pld_data + PAYLOAD_HEADER_LENGTH + loc, data, data_length);

    // TODO (apullin) : Shouldn't iter_index keep track of "loc"?
    pld->iter_index += data_length;
}

void payWriteByte(Payload pld, unsigned char loc, unsigned char data) {

    pld->pld_data[loc+PAYLOAD_HEADER_LENGTH] = data;

}

unsigned char* payGetData(Payload pld) {
    return pld->pld_data + PAYLOAD_HEADER_LENGTH;
}


void paySetData(Payload pld, unsigned char data_length, unsigned char *data) {
    payAppendData(pld, 0, data_length, data);
}


unsigned char payGetPayloadLength(Payload pld) {
    return pld->data_length + PAYLOAD_HEADER_LENGTH;
}

unsigned char payGetDataLength(Payload pld) {
    return pld->data_length;
}

void paySetType(Payload pld, unsigned char type) {
    pld->pld_data[TYPE_POS] = type;
}

unsigned char payGetType(Payload pld) {
    return pld->pld_data[TYPE_POS];
}

void paySetStatus(Payload pld, unsigned char status){
    pld->pld_data[STATUS_POS] = status;
}

unsigned char payGetStatus(Payload pld) {
    return pld->pld_data[STATUS_POS];
}

void payDelete(Payload pld) {
    free(pld->pld_data);
    free(pld);
}
