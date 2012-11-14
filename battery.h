/*
 * Copyright (c) 2007-2012, Regents of the University of California
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
 * Header for the battery supervisor module
 *
 * by Fernando L. Garcia Bermudez and Stanley S. Baek
 *
 * v.0.2
 *
 * Usage:
 *  #include "battery.h"
 *
 *  // Initialize battery supervisor
 *  batSetup();
 *
 *  // When the battery's voltage falls below the supervisor's threshold, the
 *  // interrupt will trip. If you'd like to stop all running motors when this
 *  // happens, please define __LOWBATT_STOPS_MOTORS in your project.
 *
 */

#ifndef __BATTERY_H
#define __BATTERY_H

typedef void (*BatteryEventISR)(void);

/**
 * Set up the battery supervisor module
 */
void batSetup(void);

/**
 * Specify a function to call on battery supervisor events
 * @param isr - Battery event callback function pointer
 */
void batSetCallback(BatteryEventISR isr);

#endif
