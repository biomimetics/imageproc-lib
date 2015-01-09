/*
 * Copyright (c) 2010-2014, Regents of the University of California
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
 * Generalized interface for telemetry recording and readback.
 *
 * by Andrew Pullin
 *
 * v0.1
 *
 * Revisions:
 *  Andrew Pullin       2014       Initial publish to imageproc-lib
 *
 * Usage:
 * There are three requed macros to be defined, in settings.h:
 *       #define TELEM_TYPE myTelemStruct_t
 *       #define TELEM_INCLUDE "my_telem.h"
 *       #define TELEMPACKFUNC(x) myTelemGetData(x)
 *
 * The header file referenced by TELEM_INCLUDE must be defined.
 * See https://github.com/biomimetics/roach/blob/master/lib/vr_telem.h for an
 * example of how to set up this file.
 * Similarly, the matching C file that implements TELEMPACKFUNC(x) must exist.
 * See https://github.com/biomimetics/roach/blob/master/lib/vr_telem.c for an
 * example implementation matching he header above.
 */

#include "utils.h"
#include "settings.h"
#include "dfmem.h"
#include "telem.h"
#include "radio.h"
#include "at86rf231_driver.h"
#include "sclock.h"
//#include "sys_service.h"
#include "cmd.h" //for CMD codes
#include <string.h> //for memcpy

//This is a terrible hack to avoid bus conflicts.
//TODO: find source of bus problems and fix
#include <timer.h>

//Timer parameters
#define TIMER_FREQUENCY     1000.0                // 1000 Hz
#define TIMER_PERIOD        1/TIMER_FREQUENCY
#define DEFAULT_SKIP_NUM    1


#if defined(__RADIO_HIGH_DATA_RATE)
#define READBACK_DELAY_TIME_MS 3
#else
#define READBACK_DELAY_TIME_MS 7
#endif

telemStruct_t telemBuffer;
unsigned int telemDataSize;
unsigned int telemPacketSize;

#define TELEM_HEADER_SIZE   sizeof(telemBuffer.sampleIndex) + sizeof(telemBuffer.timestamp)

////////   Private variables   ////////////////
static unsigned long samplesToSave = 0;
//Skip counter for dividing the 300hz timer into lower telemetry rates
static unsigned int telemSkipNum = DEFAULT_SKIP_NUM;
static unsigned int skipcounter = DEFAULT_SKIP_NUM;
static unsigned long sampIdx = 0;

//static unsigned long samplesToStream = 0;
//static char telemStreamingFlag = TELEM_STREAM_OFF;
//static unsigned int streamSkipCounter = 0;
//static unsigned int streamSkipNum = 15;

//Offset for time value when recording samples
static unsigned long telemStartTime = 0;

static DfmemGeometryStruct mem_geo;

///////////// Private functions //////////////
//Function to be installed into T5, and setup function
//static void SetupTimer5(); // Might collide with setup in steering module!
//static void telemServiceRoutine(void); //To be installed with sysService
//The following local functions are called by the service routine:
static void telemISRHandler(void);

/////////        Telemtry ISR          ////////
////////  Installed to Timer5 @ 300hz  ////////
//void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {

//static void telemServiceRoutine(void) {
    //This intermediate function is used in case we want to tie other
    //sub-taks to the telemtry service routine.
    //TODO: Is this neccesary?

    // Section for saving telemetry data to flash
    // Uses telemSkip as a divisor to T5.
//    telemISRHandler();
//}

/*
static void SetupTimer5() {
    ///// Timer 5 setup, Steering ISR, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T5CON1value, T5PERvalue;
    // prescale 1:64
    T5CON1value = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_64 & T5_SOURCE_INT;
    // Period is set so that period = 5ms (200Hz), MIPS = 40
    //T5PERvalue = 2083; // ~300Hz
    T5PERvalue = 625; //1Khz
    int retval;
    //retval = sysServiceConfigT5(T5CON1value, T5PERvalue, T5_INT_PRIOR_4 & T5_INT_ON);
}
*/

void telemSetup() {

    dfmemGetGeometryParams(&mem_geo); // Read memory chip sizing

    //Telemetry packet size is set at startupt time.
    telemDataSize = sizeof (TELEM_TYPE);
    telemPacketSize = sizeof (telemStruct_t);
    Nop();
    Nop();

    //Install telemetry service handler
    // Lines removed before to use telemetry module in direct mode, does not 
    // run its own ISR. (pullin, 10/9/14)
    //int retval;
    //retval = sysServiceInstallT5(telemServiceRoutine);
    //SetupTimer5();
}

void telemSetSamplesToSave(unsigned long n) {
    samplesToSave = n;
    sampIdx = 0;
}

void telemReadbackSamples(unsigned long numSamples, unsigned int src_addr) {
    int delaytime_ms = READBACK_DELAY_TIME_MS;
    unsigned long i = 0; //will actually be the same as the sampleIndex

    telemStruct_t sampleData;

    //This is a terrible hack to avoid bus conflicts.
    //TODO: find source of bus problems and fix
    DisableIntT1;

    for (i = 0; i < numSamples; i++) {
        //Retireve data from flash
        telemGetSample(i, sizeof (sampleData), (unsigned char*) (&sampleData));
        //Reliable send, with linear backoff
        do {
            //debugpins1_set();
            telemSendDataDelay(&sampleData, delaytime_ms, src_addr);
            //Linear backoff
            delaytime_ms += 0;
            //debugpins1_clr();
        } while (trxGetLastACKd() == 0);
        
        delaytime_ms = READBACK_DELAY_TIME_MS;
    }

    //This is a terrible hack to avoid bus conflicts.
    //TODO: find source of bus problems and fix
    EnableIntT1;
}

void telemSendDataDelay(telemStruct_t* sample, int delaytime_ms, unsigned int src_addr) {
    radioSendData(src_addr, 0, CMD_FLASH_READBACK, telemPacketSize, (unsigned char *)sample, 0);
    delay_ms(delaytime_ms); // allow radio transmission time
}


//Saves telemetry data structure into flash memory, in order
//Position in flash memory is maintained by dfmem module
void telemSaveData(telemStruct_t * telemPkt) {
    
    //Write the packet header info to the DFMEM
    dfmemSave((unsigned char*) telemPkt, sizeof(telemStruct_t));
    samplesToSave--;

    //This is done here instead of the ISR because telemSaveData() will only be
    //executed if samplesToSave > 0 upon entry.
    if (samplesToSave == 0) {
        //Done sampling, commit last buffer
        dfmemSync();
    }
}

void telemErase(unsigned long numSamples) {
    //dfmemEraseSectorsForSamples(numSamples, sizeof (telemU));
    // TODO (apullin) : Add an explicit check to see if the number of saved
    //                  samples will fit into memory!

    //Green LED will be used as progress indicator

    //Horibble hack: Disable IMU while erasing flash
    _T4IE = 0;

    LED_GREEN = 1;
    unsigned int firstPageOfSector, i;

    //avoid trivial case
    if (numSamples == 0) {
        return;
    }

    //Saves to dfmem will NOT overlap page boundaries, so we need to do this level by level:
    unsigned int samplesPerPage = mem_geo.bytes_per_page / telemPacketSize; //round DOWN int division
    unsigned int numPages = (numSamples + samplesPerPage - 1) / samplesPerPage; //round UP int division
    unsigned int numSectors = (numPages + mem_geo.pages_per_sector - 1) / mem_geo.pages_per_sector;

    //This is a terrible hack to avoid bus conflicts.
    //TODO: find source of bus problems and fix
    DisableIntT1;

    //At this point, it is impossible for numSectors == 0
    //Sector 0a and 0b will be erased together always, for simplicity
    //Note that numSectors will be the actual number of sectors to erase,
    //   even though the sectors themselves are numbered starting at '0'
    dfmemEraseSector(0); //Erase Sector 0a
    LED_GREEN = ~LED_GREEN;
    dfmemEraseSector(8); //Erase Sector 0b
    LED_GREEN = ~LED_GREEN;

    //Start erasing the rest from Sector 1,
    // The (numsectors-1) here is because sectors are numbered from 0, whereas
    // numSectors is the actual count of sectors to erase; fencepost error.
    for (i = 1; i <= (numSectors-1); i++) {
        firstPageOfSector = mem_geo.pages_per_sector * i;
        //hold off until dfmem is ready for sector erase command
        //LED should blink indicating progress
        //Send actual erase command
        dfmemEraseSector(firstPageOfSector);
        LED_GREEN = ~LED_GREEN;
    }

    //Leadout flash, should blink faster than above, indicating the last sector
    while (!dfmemIsReady()) {
        LED_GREEN = ~LED_GREEN;
        delay_ms(50);
    }
    LED_GREEN = 0; //Green LED off

    //This is a terrible hack to avoid bus conflicts.
    //TODO: find source of bus problems and fix
    EnableIntT1;

    //Since we've erased, reset our place keeper vars
    dfmemZeroIndex();

    //Horibble hack: Disable IMU while erasing flash
    _T4IE = 1;
}


void telemGetSample(unsigned long sampNum, unsigned int sampLen, unsigned char *data)
{
    unsigned int samplesPerPage = mem_geo.bytes_per_page / sampLen; //round DOWN int division
    unsigned int pagenum = sampNum / samplesPerPage;
    unsigned int byteOffset = (sampNum - pagenum*samplesPerPage)*sampLen;

    dfmemRead(pagenum, byteOffset, sampLen, data);
}

//This only exists to allow an exteral module to cause a telemetry save
//immediately. (pullin, 10/9/14)
void telemSaveNow(){
    telemISRHandler();
}

////   Private functions
////////////////////////

static void telemISRHandler() {

    //skipcounter decrements to 0, triggering a telemetry save, and resets
    // value of skicounter
    if (skipcounter == 0) {
        if (samplesToSave > 0) {
            telemBuffer.timestamp = sclockGetTime() - telemStartTime;
            telemBuffer.sampleIndex = sampIdx;
            //Write telemetry data into packet
            //TELEMPACKFUNC((unsigned char*) &(telemBuffer.telemData));
            TELEMPACKFUNC( &(telemBuffer.telemData) );

            telemSaveData(&telemBuffer);
            sampIdx++;
        }
        //Reset value of skip counter
        skipcounter = telemSkipNum;
    }
    //Always decrement skip counter at every interrupt, at 300Hz
    //This way, if telemSkipNum = 1, a sample is saved at every interrupt.
    skipcounter--;

}

void telemSetSkip(unsigned int skipnum) {
    telemSkipNum = skipnum;
}

//This function is a setter for the telemStartTime variable,
//which is used to offset the recorded times for telemetry, such that
//they start at approx. 0, instead of reflecting the total number of
//sclock ticks.

void telemSetStartTime(void) {
    telemStartTime = sclockGetTime();
}
