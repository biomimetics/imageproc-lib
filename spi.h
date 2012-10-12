/********************************************************************/
/*              Header for SPI module library functions             */
/********************************************************************/

#if defined(__dsPIC30F__)
#include <p30Fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#elif defined (__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined (__PIC24E__)
#include <p24Exxxx.h>
#endif

#ifndef __SPI_H
#define __SPI_H

/* List of SFRs for SPI */
/* This list contains the SFRs with default (POR) values to be used for configuring SPI */
/* The user can modify this based on the requirement */

#define SPI1STAT_VALUE         0x0000
#define SPI2STAT_VALUE         0x0000
#define SPI1BUF_VALUE          0x0000
#define SPI2BUF_VALUE          0x0000
 
#if defined(__dsPIC33F__) || defined(__PIC24H__) || defined(__dsPIC33E__) || defined (__PIC24E__)\
    || defined(__dsPIC30F1010__) || defined(__dsPIC30F2020__) || defined(__dsPIC30F2023__)

#define _SPI_V2

#define SPI1CON1_VALUE          0x0000
#define SPI2CON1_VALUE          0x0000
#define SPI1CON2_VALUE          0x0000
#define SPI2CON2_VALUE          0x0000

#elif defined(__dsPIC30F__)

#define _SPI_V1

#define SPI1CON_VALUE          0x0000
#define SPI2CON_VALUE          0x0000

#endif

#if defined(_SPI_V2)

/* SPIXCON1 REGISTER bits differing from 30F devices*/

#define DISABLE_SCK_PIN         0xffff  /* Internal SPI clock is diabled, pin functions as I/O */
#define ENABLE_SCK_PIN          0xefff  /*Internal SPI clock is enabled */

/* SPIXCON2 REGISTER */

#define  FRAME_ENABLE_ON        0xffff  /* Frame SPI support enable        */
#define  FRAME_ENABLE_OFF       0x7fff  /* Frame SPI support Disable       */

#define  FRAME_SYNC_INPUT       0xffff  /* Frame sync pulse Input (slave)  */
#define  FRAME_SYNC_OUTPUT      0xbfff  /* Frame sync pulse Output (master)*/

#define FRAME_POL_ACTIVE_HIGH   0xffff  /* Frame sync pulse is active-high*/
#define FRAME_POL_ACTIVE_LOW    0xdfff  /* Frame sync pulse is active-low */

#define FRAME_SYNC_EDGE_COINCIDE 0xffff  /* Frame sync pulse coincides with first bit clock */
#define FRAME_SYNC_EDGE_PRECEDE  0xfffd  /* Frame sync pulse precedes first bit clock */

#define FIFO_BUFFER_ENABLE      0xffff  /* FIFO buffer enabled */
#define FIFO_BUFFER_DISABLE     0xfffe  /* FIFO buffer disabled */


#elif defined(_SPI_V1)

/* SPIxCON REGISTER bits in 30F (non-SMPS) devices differing from 33F and 24H devices */

#define  FRAME_ENABLE_ON        0xffff  /* Frame SPI support enable        */
#define  FRAME_ENABLE_OFF       0xbfff  /* Frame SPI support Disable       */

#define  FRAME_SYNC_INPUT       0xffff  /* Frame sync pulse Input (slave)  */
#define  FRAME_SYNC_OUTPUT      0xdfff  /* Frame sync pulse Output (master)*/

#endif

#define  DISABLE_SDO_PIN        0xffff  /* SDO pin is not used by module   */
#define  ENABLE_SDO_PIN         0xf7ff  /* SDO pin is  used by module      */

#define  SPI_MODE16_ON          0xffff  /* Communication is word wide      */
#define  SPI_MODE16_OFF         0xfbff  /* Communication is byte wide      */

#define  SPI_SMP_ON             0xffff  /* Input data sampled at end of data output time */
#define  SPI_SMP_OFF            0xfdff  /* Input data sampled at middle of data output time */

#define  SPI_CKE_ON             0xffff  /* Transmit happens from active clock 
                                           state to idle clock state*/
#define  SPI_CKE_OFF            0xfeff  /* Transmit happens on transition from
                                           idle clock state to active clock state */

#define  SLAVE_ENABLE_ON        0xffff  /* Slave Select enbale               */
#define  SLAVE_ENABLE_OFF       0xff7f  /* Slave Select not used by module   */

#define  CLK_POL_ACTIVE_LOW     0xffff  /* Idle state for clock is high, active is low */
#define  CLK_POL_ACTIVE_HIGH    0xffbf  /* Idle state for clock is low, active is high */

#define  MASTER_ENABLE_ON       0xffff  /* Master Mode              */
#define  MASTER_ENABLE_OFF      0xffdf  /* Slave Mode               */

#define  SEC_PRESCAL_1_1        0xffff  /* Secondary Prescale 1:1   */
#define  SEC_PRESCAL_2_1        0xfffb  /* Secondary Prescale 2:1   */
#define  SEC_PRESCAL_3_1        0xfff7  /* Secondary Prescale 3:1   */
#define  SEC_PRESCAL_4_1        0xfff3  /* Secondary Prescale 4:1   */
#define  SEC_PRESCAL_5_1        0xffef  /* Secondary Prescale 5:1   */
#define  SEC_PRESCAL_6_1        0xffeb  /* Secondary Prescale 6:1   */
#define  SEC_PRESCAL_7_1        0xffe7  /* Secondary Prescale 7:1   */
#define  SEC_PRESCAL_8_1        0xffe3  /* Secondary Prescale 8:1   */

#define  PRI_PRESCAL_1_1        0xffff  /* Primary Prescale 1:1     */
#define  PRI_PRESCAL_4_1        0xfffe  /* Primary Prescale 4:1     */
#define  PRI_PRESCAL_16_1       0xfffd  /* Primary Prescale 16:1    */
#define  PRI_PRESCAL_64_1       0xfffc  /* Primary Prescale 64:1    */

/* SPIxSTAT REGISTER */

#define  SPI_ENABLE             0xffff  /* Enable module */
#define  SPI_DISABLE            0x7fff  /* Disable module */

#define  SPI_IDLE_CON           0xdfff  /* Continue module operation in idle mode */
#define  SPI_IDLE_STOP          0xffff  /* Discontinue module operation in idle mode */ 

#define  SPI_RX_OVFLOW_CLR     0xffbf   /* Clear receive overflow bit.*/
#define  SPI_RX_OVFLOW         0xffff   /* Buffer overflow has occured.*/

#if defined (__dsPIC33E__) || defined(__PIC24E__)

#define  SPI_SHFTREG_EMPTY       0xffff /* Shift register is empty and ready to send or receive*/
#define  SPI_SHFTREG_NOT_EMPTY   0xff7f /* Shift register is not empty */

#define  RX_FIFO_EMPTY           0xffff /* Receive FIFO is empty */
#define  RX_FIFO_NOT_EMPTY       0xffdf /* Receive FIFO is not empty */

#define  BUF_INT_SEL_7           0xffff /* Interrupt when transmit buffer is full */
#define  BUF_INT_SEL_6           0xfffb /* Interrupt when last bit is shifted into SPIxSR and 
                                           as a result TX FIFO is empty */
#define  BUF_INT_SEL_5           0xfff7 /* Interrupt when the last bit is shifted out of SPIxSR
                                           and the transmit is complete */
#define  BUF_INT_SEL_4           0xfff3 /* Interrupt when one data is shifted into SPIxSR and as a result TX FIFO
                                           has one open memory location */
#define  BUF_INT_SEL_3           0xffef /* Interrupt when the receive buffer is full */
#define  BUF_INT_SEL_2           0xffeb /* Interrupt when the receive buffer is 2/4 or more full */
#define  BUF_INT_SEL_1           0xffe7 /* Interrupt when data is received in the receive buffer */
#define  BUF_INT_SEL_0           0xffe3 /* Interrupt when the last data in the receive buffer is
                                           read,as a result the buffer is empty */

#define  BUF_ELE_COUNT_7         0xffff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_6         0xfeff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_5         0xfdff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_4         0xfcff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_3         0xfbff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_2         0xfaff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_1         0xf9ff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#define  BUF_ELE_COUNT_0         0xf8ff /* Number of SPIx transfers pending( master mode )
                                           Number of SPIx transfers unread( slave mode )*/
#endif

/* SPI Interrupt defines */

#define  SPI_INT_EN             0xffff  /* SPI Interrupt Enable     */
#define  SPI_INT_DIS            0xfff7  /* SPI Interrupt Disable    */

#define  SPI_INT_PRI_0          0xfff8  /* SPI Interrupt Prior Level_0 */
#define  SPI_INT_PRI_1          0xfff9  /* SPI Interrupt Prior Level_1 */
#define  SPI_INT_PRI_2          0xfffa  /* SPI Interrupt Prior Level_2 */
#define  SPI_INT_PRI_3          0xfffb  /* SPI Interrupt Prior Level_2 */
#define  SPI_INT_PRI_4          0xfffc  /* SPI Interrupt Prior Level_4 */
#define  SPI_INT_PRI_5          0xfffd  /* SPI Interrupt Prior Level_5 */
#define  SPI_INT_PRI_6          0xfffe  /* SPI Interrupt Prior Level_6 */
#define  SPI_INT_PRI_7          0xffff  /* SPI Interrupt Prior Level_7 */

/* Macros to  Enable/Disable interrupts and set Interrupt priority of SPI1 in 22F*/
#define EnableIntSPI1                    _SPI1IE = 1
#define DisableIntSPI1                   _SPI1IE = 0
#define SetPriorityIntSPI1(priority)     _SPI1IP = priority

/* CloseSPI. Disables SPI module */
void  CloseSPI1() __attribute__ ((section (".libperi")));

/* ConfigINtSPI1. Configure Interrupt enable and priorities */
void ConfigIntSPI1(unsigned int config)  __attribute__ ((section(".libperi")));

/* DataRdySPI */
 
char DataRdySPI1() __attribute__ ((section (".libperi")));

/* getcSPI. Read byte from SPIBUF register */
#define  getcSPI1    ReadSPI1

/* getsSPI.Write string to SPIBUF */
unsigned int getsSPI1(unsigned int length, unsigned int *rdptr, unsigned int spi_data_wait)
__attribute__ ((section (".libperi")));

/* OpenSPI1 */
#if defined(_SPI_V2)
void OpenSPI1(unsigned int config1,unsigned int config2, unsigned int config3 )__attribute__ ((section(".libperi")));
#elif defined(_SPI_V1)
void OpenSPI1(unsigned int config1,unsigned int config2 ) __attribute__ ((section (".libperi")));
#endif

/* putcSPI.Write byte/word to SPIBUF register */
#define  putcSPI1    WriteSPI1

/* putsSPI Read string from SPIBUF */
void putsSPI1(unsigned int length, unsigned int *wrptr)__attribute__ ((section (".libperi")));

/* ReadSPI.Read byte/word from SPIBUF register */
unsigned int ReadSPI1() __attribute__ ((section (".libperi")));

/* WriteSPI. Write byte/word to SPIBUF register */
void WriteSPI1(unsigned int data_out) __attribute__ ((section (".libperi")));

#if defined (__dsPIC33E__) || defined(__PIC24E__)

/* BufEleCountSPI1. Returns the number of valid words/bytes in the FIFO */
unsigned int BufEleCountSPI1(void) __attribute__ ((section (".libperi")));

/*IntSelSPI1. Selects the frequency of interrupt generation */
void IntSelSPI1( unsigned int) __attribute__ ((section (".libperi")));

#endif

#ifdef _SPI2IF

/* Macros to  Enable/Disable interrupts and set Interrupt priority of SPI2 */
#define EnableIntSPI2                    _SPI2IE = 1
#define DisableIntSPI2                   _SPI2IE = 0
#define SetPriorityIntSPI2(priority)     _SPI2IP = priority

/* CloseSPI2.Disables SPI module */
void  CloseSPI2()  __attribute__ ((section (".libperi")));

/* ConfigINtSPI2. Configures Interrupt enable and priorities */
void ConfigIntSPI2(unsigned int config)  __attribute__ ((section(".libperi")));

/* OpenSPI2 */
#if defined(_SPI_V2)
void OpenSPI2(unsigned int config1,unsigned int config2, unsigned int config3 )__attribute__ ((section(".libperi")));
#elif defined(_SPI_V1)
void OpenSPI2(unsigned int config1,unsigned int config2 ) __attribute__ ((section (".libperi")));
#endif

/* DataRdySPI2. Test if SPIBUF register is full */
char DataRdySPI2()  __attribute__ ((section (".libperi")));

/* getcSPI2.Read byte from SPIBUF register */
#define  getcSPI2    ReadSPI2

/* getsSPI2.Write string to SPIBUF */
unsigned int getsSPI2(unsigned int length, unsigned int *rdptr, unsigned int spi_data_wait)
 __attribute__ ((section(".libperi")));

/* putcSPI2.Write byte/word to SPIBUF register */
#define  putcSPI2    WriteSPI2

/* putsSPI2. Read string from SPIBUF */
void putsSPI2(unsigned int length, unsigned int *wrptr)__attribute__ ((section(".libperi")));

/* ReadSPI2.Read byte/word from SPIBUF register */
unsigned int ReadSPI2() __attribute__ ((section (".libperi")));

/* WriteSPI2. Write byte/word to SPIBUF register */
void WriteSPI2( unsigned int data_out) __attribute__ ((section(".libperi")));

#if defined (__dsPIC33E__) || defined(__PIC24E__)

/* BufEleCountSPI2. Returns the number of valid words/bytes in the FIFO */
unsigned int BufEleCountSPI2(void) __attribute__ ((section (".libperi")));

/*IntSelSPI2. Selects the frequency of interrupt generation */
void IntSelSPI2( unsigned int) __attribute__ ((section (".libperi")));

#endif

#endif

#ifdef _SPI3IF

/* Macros to  Enable/Disable interrupts and set Interrupt priority of SPI3 */
#define EnableIntSPI3                    _SPI3IE = 1
#define DisableIntSPI3                   _SPI3IE = 0
#define SetPriorityIntSPI3(priority)     _SPI3IP = priority

/* CloseSPI3.Disables SPI module */
void  CloseSPI3()  __attribute__ ((section (".libperi")));

/* ConfigINtSPI3. Configures Interrupt enable and priorities */
void ConfigIntSPI3(unsigned int config)  __attribute__ ((section(".libperi")));

/* OpenSPI3 */
#if defined(_SPI_V2)
void OpenSPI3(unsigned int config1,unsigned int config2, unsigned int config3 )__attribute__ ((section(".libperi")));
#elif defined(_SPI_V1)
void OpenSPI3(unsigned int config1,unsigned int config2 ) __attribute__ ((section (".libperi")));
#endif

/* DataRdySPI3. Test if SPIBUF register is full */
char DataRdySPI3()  __attribute__ ((section (".libperi")));

/* getcSPI3.Read byte from SPIBUF register */
#define  getcSPI3    ReadSPI3

/* getsSPI3.Write string to SPIBUF */
unsigned int getsSPI3(unsigned int length, unsigned int *rdptr, unsigned int spi_data_wait)
 __attribute__ ((section(".libperi")));

/* putcSPI3.Write byte/word to SPIBUF register */
#define  putcSPI3    WriteSPI3

/* putsSPI3. Read string from SPIBUF */
void putsSPI3(unsigned int length, unsigned int *wrptr)__attribute__ ((section(".libperi")));

/* ReadSPI3.Read byte/word from SPIBUF register */
unsigned int ReadSPI3() __attribute__ ((section (".libperi")));

/* WriteSPI3. Write byte/word to SPIBUF register */
void WriteSPI3( unsigned int data_out) __attribute__ ((section(".libperi")));

#if defined (__dsPIC33E__) || defined(__PIC24E__)

/* BufEleCountSPI3. Returns the number of valid words/bytes in the FIFO */
unsigned int BufEleCountSPI3(void) __attribute__ ((section (".libperi")));

/*IntSelSPI3. Selects the frequency of interrupt generation */
void IntSelSPI3( unsigned int) __attribute__ ((section (".libperi")));

#endif

#endif

#ifdef _SPI4IF

/* Macros to  Enable/Disable interrupts and set Interrupt priority of SPI2 */
#define EnableIntSPI4                    _SPI4IE = 1
#define DisableIntSPI4                   _SPI4IE = 0
#define SetPriorityIntSPI4(priority)     _SPI4IP = priority

/* CloseSPI4.Disables SPI module */
void  CloseSPI4()  __attribute__ ((section (".libperi")));

/* ConfigINtSPI4. Configures Interrupt enable and priorities */
void ConfigIntSPI4(unsigned int config)  __attribute__ ((section(".libperi")));

/* OpenSPI4 */
#if defined(_SPI_V2)
void OpenSPI4(unsigned int config1,unsigned int config2, unsigned int config3 )__attribute__ ((section(".libperi")));
#elif defined(_SPI_V1)
void OpenSPI4(unsigned int config1,unsigned int config2 ) __attribute__ ((section (".libperi")));
#endif

/* DataRdySPI4. Test if SPIBUF register is full */
char DataRdySPI4()  __attribute__ ((section (".libperi")));

/* getcSPI4.Read byte from SPIBUF register */
#define  getcSPI4    ReadSPI4

/* getsSPI4.Write string to SPIBUF */
unsigned int getsSPI4(unsigned int length, unsigned int *rdptr, unsigned int spi_data_wait)
 __attribute__ ((section(".libperi")));

/* putcSPI4.Write byte/word to SPIBUF register */
#define  putcSPI4    WriteSPI4

/* putsSPI4. Read string from SPIBUF */
void putsSPI4(unsigned int length, unsigned int *wrptr)__attribute__ ((section(".libperi")));

/* ReadSPI4.Read byte/word from SPIBUF register */
unsigned int ReadSPI4() __attribute__ ((section (".libperi")));

/* WriteSPI4. Write byte/word to SPIBUF register */
void WriteSPI4( unsigned int data_out) __attribute__ ((section(".libperi")));

#if defined (__dsPIC33E__) || defined(__PIC24E__)

/* BufEleCountSPI4. Returns the number of valid words/bytes in the FIFO */
unsigned int BufEleCountSPI4(void) __attribute__ ((section (".libperi")));

/*IntSelSPI4. Selects the frequency of interrupt generation */
void IntSelSPI4( unsigned int) __attribute__ ((section (".libperi")));

#endif

#endif

#endif
