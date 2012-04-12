/*
 * Copyright (c) 2010-2012, Regents of the University of California
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
 * Default command queue handlers
 *
 * by Stanley S. Baek
 *
 * v.alpha
 *
 * Revisions:
 *  Stanley S. Baek             2010-7-10   Initial release
 *  Ronald S. Fearing           2011-11-11  Added the ability to introduce/query
 *                                          a compiled-in version string.
 *                              2012-2-10   Introduced handling of invalid
 *                                          command codes.
 *  Fernando L. Garcia Bermudez 2012-2-11   Detached global commands from the
 *  w/Andrew Pullin                         user's cmd.c, introducing another
 *  & Humphrey Hu                           abstraction layer to the cmd queue.
 *                              2012-2-13   Introduced command to stop execution
 *                                          and exit with a status message.
 *
 * Notes:
 *  - CMD values of 0x00(0) - 0x3F(127) are defined here
 *  - for bootloader (0x00 - 0x1F)
 *  - CMD values of 0x80(128) - 0xEF(239) are available for user applications.
 *  - CMD values of 0xF0(240) - 0xFF(255) are reserved for future use
 */

#include "cmd_default.h"
#include "radio.h"
#include "payload.h"
#include "pwm.h"


// Command namespace
#define NAMESPACE_MIN_CMD    0x00
#define NAMESPACE_MAX_CMD    0xFF
#define NAMESPACE_DEFAULT    0
#define NAMESPACE_USER       1
#define NAMESPACES           2
#define LIMITS               2
#define LIMIT_MIN            0
#define LIMIT_MAX            1

// Commands
#define CMD_EXIT             0x00
#define CMD_RESET            0x01
#define CMD_ECHO             0x02
#define CMD_VERSION          0x03

// TODO (fgb) : Move this to payload.h?
#define MIN_PAYLOAD_LENGTH   0x00
#define MAX_PAYLOAD_LENGTH   0x80


/*-----------------------------------------------------------------------------
 *          Private variables
-----------------------------------------------------------------------------*/

typedef void (*cmd_func_type)(unsigned char, unsigned char, unsigned char*);

cmd_func_type cmd_func[NAMESPACE_MAX_CMD];
unsigned char cmd_pld_length[NAMESPACE_MAX_CMD];


/*----------------------------------------------------------------------------
 *          Declaration of private functions
 ---------------------------------------------------------------------------*/

static void cmdReset(unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEcho(unsigned char status, unsigned char length, unsigned char *frame);
static void cmdVersion(unsigned char status, unsigned char length, unsigned char *frame);

static void cmdSetCommand(unsigned char namespace, unsigned char command, cmd_func_type function, unsigned char pld_length);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void cmdSetupDefaultCommands(void)
{
    unsigned int i;

    for ( i = 0; i < NAMESPACE_MAX_CMD; ++i )
    {
        cmd_func[i] = NULL;
        cmd_pld_length[i] = MIN_PAYLOAD_LENGTH;
    }

    cmdSetCommand(NAMESPACE_DEFAULT, CMD_ECHO, cmdEcho, MAX_PAYLOAD_LENGTH);
    cmdSetCommand(NAMESPACE_DEFAULT, CMD_RESET, cmdReset, MIN_PAYLOAD_LENGTH);
    cmdSetCommand(NAMESPACE_DEFAULT, CMD_VERSION, cmdVersion, MIN_PAYLOAD_LENGTH);
}

void cmdSetUserCommand(unsigned char command, void *function, unsigned char pld_length)
{
    cmdSetCommand(NAMESPACE_USER, command, function, pld_length);
}

void cmdHandleRadioRxBuffer(void)
{
    Payload pld;
    unsigned char status, command, data_length;

    if ((pld = radioReceivePayload()) != NULL)
    {
        status = payGetStatus(pld);
        command = payGetType(pld);
        data_length = payGetDataLength(pld);

        if ((cmd_func[command] != NULL && cmd_pld_length[command] == data_length) || command == CMD_ECHO)
        {
            cmd_func[command](status, data_length, payGetData(pld));
        } else {
            cmdExit(status, "ERROR: The command received was invalid or its payload not the correct length.");
        }

        payDelete(pld);
    }

    return;
}

void cmdExit(unsigned char status, char *message)
{
    cmdEmergencyStop();

    radioSendPayload(macGetDestAddr(), payCreate(sizeof(message), (unsigned char *)message, status, CMD_EXIT));
}

void cmdEmergencyStop(void)
{
    unsigned char i;

    for (i=1; i<=4; i++) SetDCMCPWM(i, 0, 0);

    CloseMCPWM(); // will need a reset to recover mobility
}


/*-----------------------------------------------------------------------------
 *          Private functions
-----------------------------------------------------------------------------*/

static void cmdEcho(unsigned char status, unsigned char length, unsigned char *frame)
{
    radioSendPayload(macGetDestAddr(), payCreate(length, frame, status, CMD_ECHO));
}

static void cmdReset(unsigned char status, unsigned char length, unsigned char *frame)
{
    asm volatile("reset");
}

static void cmdVersion(unsigned char status, unsigned char length, unsigned char *frame)
{
    extern unsigned char version[];

    // Limit string length, to avoid packet size limit
    // TODO (fgb) : This should be done at python runtime to prevent unnecessary storage and clipping
    unsigned char string_length = 0;    //
    while(string_length < 127 && version[string_length] != '\n') string_length++;

    radioSendPayload(macGetDestAddr(), payCreate(string_length, version, status, CMD_VERSION));
}

static void cmdSetCommand(unsigned char namespace, unsigned char command, cmd_func_type function, unsigned char pld_length)
{
    unsigned char cmd_namespace[NAMESPACES][LIMITS] = {{NAMESPACE_MIN_CMD, 0x7F},
                                                      {0x80, NAMESPACE_MAX_CMD}};

    if (function != NULL || pld_length <= MAX_PAYLOAD_LENGTH)
    {
        if (command >= cmd_namespace[namespace][LIMIT_MIN] &&
            command <= cmd_namespace[namespace][LIMIT_MAX] )
        {
            cmd_func[command] = function;
            cmd_pld_length[command] = pld_length;
        }
    } else {
        cmdExit(0x01, "ERROR: A command function pointer is being set to NULL or its payload expected length is greater than the packet length [128].");
    }
}
