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
 * LCD Module for MikroElectronika/Exp16 Dev Board
 *
 * by Stanley S. Baek
 *
 * v.beta
 *
 * Revisions:
 *  Stanley S. Baek      2010-05-26    Initial release
 *                      
 * Notes:
 *  - MCU resources required for this module:
 *   A 4-bit LCD for Mikroelektronika dev board (40 MIPS) is connected to
*       D4:D7 - RB4:RB7
*       EN - RB3
*       RS - RB2
*    A 8-bit LCD for Explore 16 Dev board (40 MIPS) is connected to
*       D0:D7 - RE0:RE7
*       RW - RD5
*       EN - RD4
*       RS - B15
*       
*/

/******************************************************************************

*
* Usage:
*   1. Run lcdSetup first
*   2. The print(str) function will print the first 32 characters in str.
*      of str on the LCD. It takes about 1ms for 2*16 LCD (2.5ms fro 4*20 LCD)
*      to finish the function. Very slow....
*   3. Here is an example.
*
*   #include <stdio.h>
*   #include  "lcd.h"
*
*   char foo[32];
*   int age = 32;
*
*   lcdSetup(2,16);     // 2 rows and 16 cols
*   sprintf(foo, "Aaron is %o years old.", age); // print age in octal.    
*   print(foo)  
*   
*   It will print on your LCD the following
*   "Aaron is 40 year"
*   "s old.          "
*
*   4. lcdSendString can be used alternatively for print.
*   5. Important Note: If you would like to have more advanced output 
*      behaviors such as automatic shifting, scrolling up and down, etc, 
*      please please please implement them by yourself and share them 
*      with me. :D Thanks.
*
******************************************************************************/

#include "ports.h"
#include "lcd.h"
#include "utils.h"


/*****	LCD COMMAND FUCNTION PROTOTYPES  *****/
//#define cursor_right()        lcdSendCmd( 0x14 )
//#define cursor_left()         lcdSendCmd( 0x10 )
//#define display_shift()       lcdSendCmd( 0x1C )
//#define home_clr()            lcdSendCmd( 0x01 ) 
//#define home_it()             lcdSendCmd( 0x02 ) 
//#define go_to_line_2()        lcdSendCmd( 0xC0 ) 


#if defined(__MIKRO)
    //LCD Registers addresses
    #define LCD_EN      0x08
    #define LCD_RS      0x04

    // We want not to change RB8 - RB15 in this module.
    // By casting LATB in (unsigned char*), we only use lower
    // eight bits of LATB ports.
    unsigned char* lcd_port_ = (unsigned char*)&LATB;
#elif defined(__EXP16DEV)
    #define  LCD_RW  _LATD5       // LCD R/W signal
    #define  LCD_RS  _LATB15      // LCD RS signal
    #define  LCD_EN  _LATD4 

    // We want not to change RB8 - RB15 in this module.
    // By casting LATB in (unsigned char*), we only use lower
    // eight bits of LATB ports.
    unsigned char* lcd_port_ = (unsigned char*)&LATE;
#endif

#define LCD_SEND_DELAY      40      // microseconds
#define LCD_EN_HOLD()       Nop(); Nop(); Nop() // to hold EN signal 



/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/
    
static unsigned char lcd_rows_ = 2; 
static unsigned char lcd_cols_ = 16;


/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

static void lcdReset(void);
static void lcdSendCmd (char cmd);
static void lcdSendData (char data);


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

void lcdSetup(unsigned char rows, unsigned char cols) {

    lcd_rows_ = rows;
    lcd_cols_ = cols;

    #if defined(__MIKRO)
        lcdReset();             // Call LCD reset
        lcdSendCmd(0x28);       // 4-bit mode - 2 (or 4) lines - 5x7 font.
        lcdSendCmd(0x0C);       // Display no cursor - no blink.
        lcdSendCmd(0x06);       // Automatic Increment - No Display shift.
        lcdSendCmd(0x80);       // Address DDRAM with 0 offset 80h.
    #elif defined(__EXP16DEV)
        lcdReset();
        lcdSendCmd(0x38);       // 8-bit mode - 2 (or 4) lines - 5x7 font.
        lcdSendCmd(0x0C);       // Display no cursor - no blink.
        lcdSendCmd(0x06);       // Automatic Increment - No Display shift.
        lcdSendCmd(0x80);       // Address DDRAM with 0 offset 80h.
        lcd_rows_ = 2;
        lcd_cols_ = 16;
    #endif


}


void lcdSendString(char *var) {
    #if (defined(__MIKRO) || defined(__EXP16DEV))
    unsigned char count = 0;
  
    lcdSendCmd(0x80);           // move to first row & first col
    while (*var) {              //till string ends
        lcdSendData(*var++);    //send characters one by one
        count++;
        if (count == lcd_cols_) {
            lcdSendCmd(0xc0);   // move to second row
        } else if (count == lcd_cols_*2) {
            lcdSendCmd(0x94);   // move to third row
        } else if (count == lcd_cols_*3) {
            lcdSendCmd(0xd4);   // move to fourth row
        } else if (count == lcd_cols_*4) {
            break;
        }
    }

    while (count < lcd_rows_ * lcd_cols_) {
        lcdSendData(' ');       // put space for the rest 
        count++;
        if (count == lcd_cols_) {
            lcdSendCmd(0xc0);
        } else if (count == lcd_cols_*2) {
            lcdSendCmd(0x94);
        } else if (count == lcd_cols_*3) {
            lcdSendCmd(0xd4);
        }
    }
    #endif
}


/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*****************************************************************************
* Function Name : lcdReset
* Description   : Reset LCD.  Use this funtion to initialize LCD.
* Parameters    : None
* Return Value  : None                                                     
*****************************************************************************/
static void lcdReset(void) {
#if defined(__MIKRO)
    *lcd_port_ = 0xff;      // may not need this line
    delay_ms(20);           // may not need this line
    *lcd_port_ = 0x30 | LCD_EN;
    *lcd_port_ = 0x30;
    delay_ms(10);           // 5ms may work
    *lcd_port_ = 0x30 | LCD_EN;
    *lcd_port_ = 0x30;
    delay_ms(1);            // 200us may work
    *lcd_port_ = 0x30 | LCD_EN;
    *lcd_port_ = 0x30;
    delay_ms(1);            // 200us may work
    *lcd_port_ = 0x20 | LCD_EN;
    *lcd_port_ = 0x20;
    delay_ms(1);            // 200us may work

#elif defined(__EXP16DEV)
    //*lcd_port_ = 0x00;
    delay_ms(1);
    *lcd_port_ = 0x38;	
    LCD_EN = 1;	
    LCD_EN_HOLD();
    LCD_EN = 0;          // toggle E signal
    delay_ms(5);         // 5ms delay
    *lcd_port_ = 0x38;	    
    LCD_EN = 1;	
    LCD_EN_HOLD();
    LCD_EN = 0;         // toggle E signal
    delay_us(200);      // 200uS delay
    *lcd_port_ = 0x38;	    
    LCD_EN = 1;	
    LCD_EN_HOLD();
    LCD_EN = 0;         // toggle E signal
    delay_us(200);      // 200uS delay
    *lcd_port_ = 0x38;	    
    LCD_EN = 1;	
    LCD_EN_HOLD();
    LCD_EN = 0;         // toggle E signal
    delay_us(200);      // 200uS delay    

#endif    

}

/*****************************************************************************
* Function Name : lcdSendCmd
* Description   : Send a command to control LCD. 
* Parameters    : cmd is a command character.
* Return Value  : None                                                     
*****************************************************************************/
static void lcdSendCmd (char cmd) {
#if defined(__MIKRO)        
    *lcd_port_ = (cmd & 0xf0) | LCD_EN;
    *lcd_port_ = (cmd & 0xf0);
    *lcd_port_ = ((cmd<<4) & 0xf0) | LCD_EN;
    *lcd_port_ = ((cmd<<4) & 0xf0);
    delay_us(LCD_SEND_DELAY);
#elif defined(__EXP16DEV)
    *lcd_port_ = cmd;           // command byte to lcd
    LCD_RW = 0;                 // ensure RW is 0
    LCD_EN = 1;                 // toggle E line
    LCD_RS = 0;
    LCD_EN_HOLD();
    LCD_EN = 0;                 // toggle E line
    delay_us(LCD_SEND_DELAY); 
#endif    
}


/******************************************************************************
* Function Name : lcdSendData
* Description   : Send a data to LCD to display.  Address automatically 
*                 increases for the next data to be sent.
* Parameters    : data is a character to be displayed on LCD.
* Return Value  : None                                                     
*******************************************************************************/
static void lcdSendData(char data) {
#if defined(__MIKRO)     
    *lcd_port_ = (data & 0xf0) | LCD_EN | LCD_RS;
    *lcd_port_ = (data & 0xf0) | LCD_RS;
    *lcd_port_ = ((data << 4) & 0xf0) | LCD_EN | LCD_RS;
    *lcd_port_ = ((data << 4) & 0xf0) | LCD_RS;
    delay_us(LCD_SEND_DELAY);
#elif defined(__EXP16DEV)     
    LCD_RW = 0;       	    // ensure RW is 0, may not need
    LCD_RS = 1;             // assert register select to 1
    *lcd_port_ = data;      // data byte to lcd
    LCD_EN = 1;	
    LCD_EN_HOLD(); 
    LCD_RS = 0;             // negate register select to 0
    LCD_EN = 0;             // toggle E signal
    delay_us(1000);
#endif
}

