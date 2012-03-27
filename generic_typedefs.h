#ifndef __GENERIC_TYPE_DEFS_H_
#define __GENERIC_TYPE_DEFS_H_


#if !defined(__PACKED)
    #define __PACKED
#endif

/* get compiler defined type definitions (NULL, size_t, etc) */
#include <stddef.h> 


#define PUBLIC                                  /* Function attributes */
#define PROTECTED
#define PRIVATE     static
#define _PERSISTENT __attribute__((persistent))

/***********************************************************************************/

/* Alternate definitions */
typedef unsigned char           byte;                           /* 8-bit unsigned  */
typedef unsigned short int      word;                           /* 16-bit unsigned */

typedef union
{
    byte val;
    struct __PACKED
    {
         byte b0:1;
         byte b1:1;
         byte b2:1;
         byte b3:1;
         byte b4:1;
         byte b5:1;
         byte b6:1;
         byte b7:1;
    } bits;
} ByteVal, ByteBits;

typedef union
{
    word val;
    byte v[2] __PACKED;
    struct __PACKED
    {
        byte LB;
        byte HB;
    } byte;
    struct __PACKED
    {
         byte b0:1;
         byte b1:1;
         byte b2:1;
         byte b3:1;
         byte b4:1;
         byte b5:1;
         byte b6:1;
         byte b7:1;
         byte b8:1;
         byte b9:1;
         byte b10:1;
         byte b11:1;
         byte b12:1;
         byte b13:1;
         byte b14:1;
         byte b15:1;
    } bits;
} WordVal, WordBits;

#endif /* __GENERIC_TYPE_DEFS_H_ */
