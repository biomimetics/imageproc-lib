/*
 * Copyright (c) 2007-2010, Regents of the University of California
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
 * OmniVision OV7660 Camera (ovcam) Configuration Interface (SCCB)
 *
 * by Fernando L. Garcia Bermudez
 *
 * v.beta
 *
 * Revisions:
 *  Fernando L. Garcia Bermudez     2007-9-10    Initial release
 *
 * Notes:
 *  - Uses an I2C(SCCB) port for configuring the camera.
 */

#include <xc.h>
#include "i2c.h"
#include "ovcam.h"
#include "utils.h"


// I2Cx Registers
#if (defined(__IMAGEPROC1) || defined(__IMAGEPROC2))

    #define I2C_TRN         I2C1TRN
    #define I2C_RCV         I2C1RCV
    #define I2C_BRG         I2C1BRG
    #define I2C_STATbits    I2C1STATbits
    #define I2C_CON         I2C1CON
    #define I2C_CONbits     I2C1CONbits

#endif

// Device Slave Addresses
#define IDADDR_W    0x42
#define IDADDR_R    0x43

/* OV7660 Control Registers */

// Common Control
#define COM1    0x04
#define COM2    0x09
#define COM3    0x0C
#define COM5    0x0E
#define COM6    0x0F
#define COM7    0x12
#define COM8    0x13
#define COM9    0x14
#define COM10   0x15
#define COM11   0x3B
#define COM12   0x3C
#define COM13   0x3D
#define COM15   0x40
#define COM16   0x41
#define COM17   0x42

// Data Format
#define CLKRC   0x11
#define PSHFT   0x1B
#define TSLB    0x3A
#define MVFP    0x1E

#define HREF    0x32
#define HSTART  0x17
#define HSTOP   0x18

#define VREF    0x03
#define VSTRT   0x19
#define VSTOP   0x1A

// Edge Enhancement
#define EDGE    0x3F
#define DSPC2   0xA0

// Banding Filter
#define BD50ST  0x9D
#define BD60ST  0x9E
#define HV      0x69

#define DBLV    0x6B

// Gain/Exposure
#define GAIN    0x00
#define BLUE    0x01
#define RED     0x02

#define AECH    0x10

#define AEW     0x24
#define AEB     0x25

// Dummy Line
#define DM_LNL  0x92
#define DM_LNH  0x93

// Array
#define CHLF    0x33
#define ARBLM   0x34

// Matrix Coefficients
#define MTX1    0x4F
#define MTX2    0x50
#define MTX3    0x51
#define MTX4    0x52
#define MTX5    0x53
#define MTX6    0x54
#define MTX7    0x55
#define MTX8    0x56
#define MTX9    0x57
#define MTXS    0x58

// ADC
#define ACOM    0x38
#define OFON    0x39


/*----------------------------------------------------------------------------
 *          Declaration of private functions
 ---------------------------------------------------------------------------*/
static void ovcamSetupPeripheral(void);
//static void ovcamSetupOV7660 (void);
static inline void ovcamWriteByte (unsigned char byte);
static inline unsigned char ovcamReadByte (void);
static inline void ovcamNACK (void);
static inline void ovcamStartTx (void);
static inline void ovcamStopTx (void);
static inline void ovcamWaitTillIdle (void);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void ovcamSetup(void)
{
    ovcamSetupPeripheral();
    ovcamTurnOn();
    delay_us(5000); // Wait for the camera to boot
    ovcamSetupOV7660();
}

void ovcamWriteRegister (unsigned char subaddr, unsigned char data)
{
    ovcamStartTx();
    ovcamWriteByte(IDADDR_W);
    ovcamWriteByte(subaddr);
    ovcamWriteByte(data);
    ovcamStopTx();
}

unsigned char ovcamReadRegister (unsigned char subaddr)
{
    unsigned char data;

    ovcamStartTx();
    ovcamWriteByte(IDADDR_W);
    ovcamWriteByte(subaddr);
    ovcamStopTx();

    ovcamStartTx();
    ovcamWriteByte(IDADDR_R);
    data = ovcamReadByte();
    ovcamNACK();
    ovcamStopTx();

    return data;
}

/*-----------------------------------------------------------------------------
 *          Private functions (for internal use only)
-----------------------------------------------------------------------------*/

// Initializes the I2Cx bus for configuring the camera.
static void ovcamSetupPeripheral(void)
{
    I2C_BRG = 363; // Fcy(1/Fscl - 1/1111111)-1
    I2C_CON = I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD & I2C1_IPMI_DIS &
              I2C1_7BIT_ADD & I2C1_SLW_DIS & I2C1_SM_DIS & I2C1_GCALL_DIS &
              I2C1_STR_DIS &  I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
              I2C1_STOP_DIS & I2C1_RESTART_DIS & I2C1_START_DIS;

    ovcamWaitTillIdle();
}

// Configures the OV7660 camera.
//
// The initial reset is to be followed by at least 5ms of settling time. After
// that, the contents of all important registers are set to the required
// values. It seems that even though one needs to change only a few values
// from their defaults, the rest of the rewrites are necessary for proper
// functioning. Since some of the registers are undocumented in the datasheet,
// they might be calibration data (provided by the manufacturer). More testing
// is needed to make sure this is the right interpretation.
void ovcamSetupOV7660 (void)
{
    // SCCB Register Reset
    ovcamWriteRegister(COM7,0x80);
    delay_us(5000); // Wait for the camera to boot

    // Pixel Clock, Banding Filter, Exposure, Gain
    ovcamWriteRegister(CLKRC,0x81); // PCLK = (XCLK/2)/2
    ovcamWriteRegister(DM_LNL,0x48);
    ovcamWriteRegister(DM_LNH,0x01);
    ovcamWriteRegister(BD50ST,0x62);
    ovcamWriteRegister(BD60ST,0x52);
    ovcamWriteRegister(COM11,0x02); // 60Hz Banding Filter
    ovcamWriteRegister(COM8,0xF2); // Banding Filter ON
    ovcamWriteRegister(AECH,0x00);
    ovcamWriteRegister(GAIN,0x00);
    ovcamWriteRegister(BLUE,0x80);
    ovcamWriteRegister(RED,0x80);
    ovcamWriteRegister(COM8,0xF7); // AGC Gain + AWB + AEC Exposure

    // Format
    ovcamWriteRegister(COM7,0x10); // QVGA & YUV
    ovcamWriteRegister(COM1,0x24); // QQVGA + HREF skip 3/4 rows (every other in QQVGA)
    ovcamWriteRegister(COM3,0x02); // Horizontal no pixel average + VarioPixel Off (extra)

    // Windowing
    ovcamWriteRegister(HSTART,0x23); // Horizontal Frame Start (8 MSb's)
    ovcamWriteRegister(HSTOP,0x4B);  // Horizontal Frame Stop (8 MSb's)
    ovcamWriteRegister(HREF,0xBF);   // 3 LSb's for Horizontal Frame Start/Stop
    ovcamWriteRegister(VSTRT,0x02);  // Vertical Frame Start (8 MSb's)
    ovcamWriteRegister(VSTOP,0x3E);  // Vertical Frame Stop (8 MSb's)
    ovcamWriteRegister(VREF,0x00);   // 2 LSb's for Vertical Frame Start/Stop

    ovcamWriteRegister(COM5,0x84); // Enable higher frame rates
    ovcamWriteRegister(COM6,0x62); // Reset all timing on format change + BLC input
    ovcamWriteRegister(COM10,0x02); // VSYNC Negative
    ovcamWriteRegister(0x16,0x02);
    ovcamWriteRegister(PSHFT,0x01);
    ovcamWriteRegister(MVFP,0x39); // Mirror On and VFLIP On
    ovcamWriteRegister(0x29,0x3C); // 20 for internal regulator
    ovcamWriteRegister(CHLF,0x00);
    ovcamWriteRegister(ARBLM,0x07);
    ovcamWriteRegister(0x35,0x84);
    ovcamWriteRegister(0x36,0x00);
    ovcamWriteRegister(ACOM,0x13);
    ovcamWriteRegister(OFON,0x43);
    ovcamWriteRegister(TSLB,0x00); // YUYV output
    ovcamWriteRegister(COM12,0x6C);
    ovcamWriteRegister(COM13,0x90); // Gamma used for raw data before interpolation
    ovcamWriteRegister(EDGE,0x29);
    ovcamWriteRegister(COM15,0xC1); // Output Range 00-FF
    ovcamWriteRegister(COM16,0x20); // Edge Enhancement for YUV
    ovcamWriteRegister(DBLV,0x0A);
    ovcamWriteRegister(0xA1,0xC8);

    ovcamWriteRegister(HV,0x40);
    ovcamWriteRegister(0x43,0xF0);
    ovcamWriteRegister(0x44,0x10);
    ovcamWriteRegister(0x45,0x78);
    ovcamWriteRegister(0x46,0xA8);
    ovcamWriteRegister(0x47,0x60);
    ovcamWriteRegister(0x48,0x80);
    ovcamWriteRegister(0x59,0xBA);
    ovcamWriteRegister(0x5A,0x9A);
    ovcamWriteRegister(0x5B,0x22);
    ovcamWriteRegister(0x5C,0xB9);
    ovcamWriteRegister(0x5D,0x9B);
    ovcamWriteRegister(0x5E,0x10);
    ovcamWriteRegister(0x5F,0xE0);
    ovcamWriteRegister(0x60,0x85); // 05 for advanced AWB
    ovcamWriteRegister(0x61,0x60);
    ovcamWriteRegister(0x9F,0x9D);
    ovcamWriteRegister(DSPC2,0xA0);

    // Matrix Coefficients
    ovcamWriteRegister(MTX1,0x66);
    ovcamWriteRegister(MTX2,0x6B);
    ovcamWriteRegister(MTX3,0x05);
    ovcamWriteRegister(MTX4,0x19);
    ovcamWriteRegister(MTX5,0x40);
    ovcamWriteRegister(MTX6,0x59);
    ovcamWriteRegister(MTX7,0x40);
    ovcamWriteRegister(MTX8,0x40);
    ovcamWriteRegister(MTX9,0x40);
    ovcamWriteRegister(MTXS,0x0D);

    ovcamWriteRegister(0x8B,0xCC);
    ovcamWriteRegister(0x8C,0xCC);
    ovcamWriteRegister(0x8D,0xCF);

    // Gamma Curve
    // GSP
    ovcamWriteRegister(0x6C,0x40);
    ovcamWriteRegister(0x6D,0x30);
    ovcamWriteRegister(0x6E,0x4B);
    ovcamWriteRegister(0x6F,0x60);
    ovcamWriteRegister(0x70,0x70);
    ovcamWriteRegister(0x71,0x70);
    ovcamWriteRegister(0x72,0x70);
    ovcamWriteRegister(0x73,0x70);
    ovcamWriteRegister(0x74,0x60);
    ovcamWriteRegister(0x75,0x60);
    ovcamWriteRegister(0x76,0x50);
    ovcamWriteRegister(0x77,0x48);
    ovcamWriteRegister(0x78,0x3A);
    ovcamWriteRegister(0x79,0x2E);
    ovcamWriteRegister(0x7A,0x28);
    ovcamWriteRegister(0x7B,0x22);
    // GST
    ovcamWriteRegister(0x7C,0x04);
    ovcamWriteRegister(0x7D,0x07);
    ovcamWriteRegister(0x7E,0x10);
    ovcamWriteRegister(0x7F,0x28);
    ovcamWriteRegister(0x80,0x36);
    ovcamWriteRegister(0x81,0x44);
    ovcamWriteRegister(0x82,0x52);
    ovcamWriteRegister(0x83,0x60);
    ovcamWriteRegister(0x84,0x6C);
    ovcamWriteRegister(0x85,0x78);
    ovcamWriteRegister(0x86,0x8C);
    ovcamWriteRegister(0x87,0x9E);
    ovcamWriteRegister(0x88,0xBB);
    ovcamWriteRegister(0x89,0xD2);
    ovcamWriteRegister(0x8A,0xE6);

    ovcamWriteRegister(COM9,0x2E); // Automatic Gain Ceiling (AGC) is 4x + Drop frame policy
    ovcamWriteRegister(AEW,0x68);
    ovcamWriteRegister(AEB,0x58);
}

// Sends a byte to the camera.
//
// Parameters : byte to send.
static inline void ovcamWriteByte (unsigned char byte)
{
    I2C_TRN = byte;
    while(I2C_STATbits.TRSTAT);
    ovcamWaitTillIdle();
    while(I2C_STATbits.ACKSTAT);
}

// Receives a byte from the camera.
//
// Returns : received byte.
static inline unsigned char ovcamReadByte (void)
{
    I2C_CONbits.RCEN = 1;
    while(I2C_CONbits.RCEN);
    I2C_STATbits.I2COV = 0;
    return(I2C_RCV);
}

// Sends NACK to the camera.
static inline void ovcamNACK (void)
{
    I2C_CONbits.ACKDT = 1;
    I2C_CONbits.ACKEN = 1;
    while(I2C_CONbits.ACKEN);
}

// Starts communication with the camera.
static inline void ovcamStartTx (void)
{
    I2C_CONbits.SEN = 1;
    while(I2C_CONbits.SEN);
}

// Stops communication with the camera.
static inline void ovcamStopTx (void)
{
    I2C_CONbits.PEN = 1;
    while(I2C_CONbits.PEN);
}

// Wait until I2C sus is inactive
static inline void ovcamWaitTillIdle (void)
{
    while(I2C_CONbits.SEN || I2C_CONbits.PEN || I2C_CONbits.RCEN ||
          I2C_CONbits.RSEN || I2C_CONbits.ACKEN || I2C_STATbits.TRSTAT);
}
