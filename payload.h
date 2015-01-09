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
 */

#ifndef __PAYLOAD_H
#define __PAYLOAD_H


#define PAYLOAD_HEADER_LENGTH   2

// Payload structure
typedef struct {
    unsigned char data_length;
    unsigned char* pld_data;
    unsigned char iter_index;
    unsigned char (*test)(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned int);
} PayloadStruct;

typedef PayloadStruct* Payload;

/*******************************************************************************
* Function Name : payCreate
* Description   :
* Parameters    :
* Return Value  :
*******************************************************************************/
Payload payCreate(unsigned char data_length, unsigned char *data,
                  unsigned char status, unsigned char type);

Payload payCreateEmpty(unsigned char data_length);

Payload payClone(Payload pld);

/*******************************************************************************
* Function Name : payNextElement
* Description   : Returns the next element from the give payload. The returned
*                 elements include headers and data of the payload. It is
*                 recommended to call payInitIterator before this function call.
*                 Users must be careful not to call this function more than the
*                 length of payload since the payInitIterator is called.
* Parameters    : Payload
* Return Value  : a character of the next element
*******************************************************************************/
unsigned char payNextElement(Payload pld);

void payInitIterator(Payload pld);

/*******************************************************************************
* Function Name : payReadByte
* Description   : Read a data character located at loc. This function does not
*                 return the contents of payload header.
* Parameters    : loc is the location of the character to be read. The first
*                 data is located at loc = 0.
* Return Value  : A character data
*******************************************************************************/
unsigned char payReadByte(Payload pld, unsigned char loc);

/*******************************************************************************
* Function Name : payToString
* Description   : Returns the string representation of the given payload
* Parameters    : Payload
* Return Value  : A pointer of the string representation of the payload
*******************************************************************************/
unsigned char* payToString(Payload pld);

void payAppendData(Payload pld, char loc,
                unsigned char data_length, unsigned char *data);

void payWriteByte(Payload pld, unsigned char loc, unsigned char data);

/*******************************************************************************
* Function Name : payGetData
* Description   : Returns the pointer of the begining of the data
* Parameters    : Payload
* Return Value  : A character pointer of the data for the given payload.
*******************************************************************************/
unsigned char* payGetData(Payload pld);

void paySetData(Payload pld, unsigned char data_length, unsigned char *data);

unsigned char payGetPayloadLength(Payload pld);

unsigned char payGetDataLength(Payload pld);

void paySetType(Payload pld, unsigned char type);

unsigned char payGetType(Payload pld);

void paySetStatus(Payload pld, unsigned char status);

unsigned char payGetStatus(Payload pld);

void payDelete(Payload pld);


#endif // __PAYLOAD_H
