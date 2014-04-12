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
 * Program memory access module for dsPIC33F.
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek 2010-11-08    Initial release
 *                      
 * Notes:
 *  - This module is based on Microchip's bootloader for dsPIC30F/33F and 
 *    PIC24F/24H Devices from Microchip (https://www.microchip.com/stellent/
 *    idcplg?IdcService=SS_GET_PAGE&nodeId=1824&appnote=en530200)
 *
 */

#include <xc.h>
#include "progmem.h"

//#include "lcd.h"
//#include <stdio.h>

#define PM_ROW_ERASE 		0x4042
#define PM_ROW_WRITE 		0x4001
#define CONFIG_WORD_WRITE	0X4000


/*-----------------------------------------------------------------------------
 *          Type Define
-----------------------------------------------------------------------------*/
typedef union tuReg32 {
    unsigned long Val32;
    struct {
	unsigned short LW;
	unsigned short HW;
    } Word;
    char Val[4];
} uReg32;

/*-----------------------------------------------------------------------------
 *          Declaration of external functions from memory.s
-----------------------------------------------------------------------------*/
extern unsigned long ReadLatch(unsigned short, unsigned short);
extern void ResetDevice(void);
extern void WriteLatch(unsigned short, unsigned short, unsigned short, unsigned short);
extern void WriteMem(unsigned short);
extern void Erase(unsigned short, unsigned short, unsigned short);


/*-----------------------------------------------------------------------------
 *          Variables
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void pmErasePage(unsigned long addr) {
    uReg32 src_addr = {addr};
    Erase(src_addr.Word.HW, src_addr.Word.LW, PM_ROW_ERASE);
}

unsigned long pmReadMem(unsigned long addr) {
    uReg32 src_addr = {addr};
    return ReadLatch(src_addr.Word.HW, src_addr.Word.LW);
}

void pmReadPage(char *data, unsigned long addr) {

    int i;
    
    uReg32 src_addr = {addr};
    uReg32 temp;

    for(i = 0; i < PM_ROW_SIZE; ++i){
	temp.Val32 = pmReadMem(src_addr.Val32);
	data[0] = temp.Val[0];
	data[1] = temp.Val[1];
	data[2] = temp.Val[2];
	data = data + 3;
	src_addr.Val32 = src_addr.Val32 + 2;
    }
}

void pmWritePage(char *data, unsigned long addr) {

    uReg32 src_addr = {addr};

    uReg32 temp;
    uReg32 temp_addr;
    uReg32 temp_data;

    int i, j;

    temp.Val[3] = 0;
    
    for(i = 0, j = 0; i < PM_ROW_SIZE; ++i){
		
	temp.Val[0] = data[j];
    	temp.Val[1] = data[j+1];
	temp.Val[2] = data[j+2];
    	j += 3;

	WriteLatch(src_addr.Word.HW, src_addr.Word.LW, temp.Word.HW, temp.Word.LW);

	/* Device ID errata workaround: Save data at any address that has LSB 0x18 */
	if((src_addr.Val32 & 0x0000001F) == 0x18) {
	    temp_addr.Val32 = src_addr.Val32;
    	    temp_data.Val32 = temp.Val32;
	}

	if((i != 0) && (((i + 1) % 64) == 0)){
	/* Device ID errata workaround: Reload data at address with LSB of 0x18 */
            WriteLatch(temp_addr.Word.HW, temp_addr.Word.LW, temp_data.Word.HW, temp_data.Word.LW);
	    WriteMem(PM_ROW_WRITE);
    	}

	src_addr.Val32 = src_addr.Val32 + 2;
    }
}

void pmReadId(char *data) {

    uReg32 src_addr, temp;

    src_addr.Val32 = 0xFF0000;
    temp.Val32 = pmReadMem(src_addr.Val32);
    data[0] = temp.Val[0];
    data[1] = temp.Val[1];
    data[2] = temp.Val[2];
		
    src_addr.Val32 = 0xFF0002;
    temp.Val32 = pmReadMem(src_addr.Val32);
    data[3] = temp.Val[0];
    data[4] = temp.Val[1];
    data[5] = temp.Val[2];

}

void pmReadGoto(char *data) {

    uReg32 src_addr, temp;
    src_addr.Val32 = 0x000000;
    temp.Val32 = pmReadMem(src_addr.Val32);
    data[0] = temp.Val[0];
    data[1] = temp.Val[1];
    data[2] = temp.Val[2];
		
    src_addr.Val32 = 0x000002;
    temp.Val32 = pmReadMem(src_addr.Val32);
    data[3] = temp.Val[0];
    data[4] = temp.Val[1];
    data[5] = temp.Val[2];
}

void pmReset(void) {
    ResetDevice();
}
