/* 
 * File:   bsp-ip25.h
 * Author: apullin
 *
 * Created on January 15, 2014, 5:05 PM
 */

#ifndef BSP_IP25_H
#define	BSP_IP25_H

//SPI setup registers; possible should be in a chip support include
// SPIx Registers
#define SPI_BUF         SPI2BUF
#define SPI_CON1        SPI2CON1
#define SPI_CON2        SPI2CON2
#define SPI_STAT        SPI2STAT
#define SPI_STATbits    SPI2STATbits
#define SPI_CON1bits    SPI2CON1bits
#define SLPTR               (_LATB15) // Radio Sleep/Transmit Pin

//TRX defines
#define TRX_CS      0 //On SPI channel 1
//DFMEM defines
#define DFMEM_CS    0 //On SPI channel 2
//MPU defines
#define MPU_CS      1 //On SPI channel 2

//These should probably be in a chip support package include, rather than BSP
#define SPI1_CS             (_LATB2)    // Radio Chip Select
#define SPI2_CS1            (_LATG9)    // Flash Chip Select
#define SPI2_CS2            (_LATC15)   // MPU6000 Chip Select

//LED colors
//TODO (): Should this be here? Since any color LED could end up installed
//  on the board, which could be very misleading
#define LED_1   _LATB12
#define LED_2   _LATB13
#define LED_3   _LATB14

#define LED_RED				LED_1
#define LED_GREEN			LED_2
#define LED_YELLOW			LED_3

#endif	/* BSP_IP25_H */

