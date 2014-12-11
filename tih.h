/*********************************************
* Name: tih.h
* Desc: Ti DRV8833 Hbridge driver
* Date: 2012-8-18
* Author: apullin
*********************************************/

#ifndef __TIH_H
#define __TIH_H


//#include "pwm.h"

#define PWM_FREQ            20000 //10 Khz
#define ADC_TRIG_POINT      0.97  //Trigger at 99% of complete PWM period

typedef enum {
    TIH_MODE_COAST = 0,
    TIH_MODE_BRAKE
} tiHDriveMode;

typedef enum {
    TIH_FWD = 0,
    TIH_REV
} tiHDriveDir;

void tiHSetup(void);
void tiHSetOutputInt(unsigned int channel, int dutycycle);
void tiHSetOutputPercent(unsigned int channel, float percent);
void tiHChangeDirection(unsigned int channel, tiHDriveDir dir);
void tiHChangeMode(unsigned int channel, tiHDriveMode mode);
void tiHSetDC(unsigned int channel, int dutycycle);
void tiHSetFloat(unsigned int channel, float percent);
int tiHGetPWMMax();
int tiHGetPWMPeriod();
int tiHGetSignedDC(unsigned int channel);

typedef struct {
    float throt_f; // [-100.0 , 100.0]
    int throt_i; // [-pwm_period*2 , pwm_period*2]
    tiHDriveMode mode;
    tiHDriveDir dir;
} tiHDriver;


#endif  // __TIH_H
