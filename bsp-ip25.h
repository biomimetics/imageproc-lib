/* 
 * File:   bsp-ip25.h
 * Author: apullin
 *
 * Created on January 15, 2014, 5:05 PM
 */

#ifndef BSP_IP25_H
#define	BSP_IP25_H

//TRX defines
#define TRX_CS      0
//DFMEM defines
#define DFMEM_CS    0
//MPU defines
#define MPU_CS      1

//These should probably be in a chip support package include
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

