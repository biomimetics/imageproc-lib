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
 * Usage:
 *   1. Run lcdSetup first
 *   2. The print(str) function will print the first 32 characters in str.
 *      of str on the LCD. It takes about 1ms for 2*16 LCD (2.5ms fro 4*20 LCD)
 *      to finish the function. Very slow....
 *   3. Here is an example.
 *
 *   #include <stdio.h>  // for sprintf()
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
 */

#ifndef __LCD_H
#define __LCD_H

#define print(x)   lcdSendString(x)


/*****************************************************************************
* Function Name : lcdSetup
* Description   : Reset and Initialized LCD
* Parameters    : rows - nubber of rows in LCD (usually 2 or 4)
*                 cols - number of cols in LCD (usually 16 or 20) 
* Return Value  : None                                                     
*****************************************************************************/
void lcdSetup(unsigned char rows, unsigned char cols); 


/*****************************************************************************
* Function Name : lcdSendString
* Description   : Send a string to LCD. If the length of string is less than
*                 the lcd_rows_ * lcd_cols_, spaces will be padded at the end
*                 of the string.
* Parameters    : None
* Return Value  : None                                                     
*****************************************************************************/
void lcdSendString(char *var);

#endif // __LCD_H
