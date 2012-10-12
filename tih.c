/*********************************************
 * Name: tih.c
 * Desc: Ti DRV8833 Hbridge driver
 * Date: 2012-8-18
 * Author: apullin
 *********************************************/

#include "tih.h"
#include "pwm.h"
#include "ports.h"
#include "led.h"
#include "init_default.h"

#define NUM_PWM 4

static tiHDriver outputs[4];
static int pwm_period;

static void tiHSetupPeripheral(void);
static void tiHConfigure(unsigned int channel);

void tiHSetup(void) {

    //This setup is unique to PWM_OP_SCALE4
    //If the clock scaler is changed, this MUST be changed too!
    //pwm_period = (int)((float)FCY/((float)PWM_FREQ * (float)4))- 1;
    pwm_period = 32494;  //magic number found by doug that tends to make things work
    tiHSetupPeripheral();
    int i;
    for(i=0; i< NUM_PWM;i++){
        outputs[i].throt_f = 0.0;
        outputs[i].throt_i = 0;
        outputs[i].mode = TIH_MODE_COAST;
        outputs[i].dir = TIH_FWD;
    }

    //Start all channels at 0 throttle in Forward / Coast mode
    tiHSetDC(1,0);
    tiHSetDC(2,0);
    tiHSetDC(3,0);
    tiHSetDC(4,0);
    

}


static void tiHSetupPeripheral(void) {

    unsigned int PTPERvalue = pwm_period;
    unsigned int SEVTCMPvalue, PTCONvalue, PWMCON1value, PWMCON2value;
    //SEVTCMPvalue = 1988;
    //PWM Special Event Trigger is set to
    SEVTCMPvalue = (int)(ADC_TRIG_POINT * (float)pwm_period);
    PTCONvalue = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE4 &
                 PWM_IPCLK_SCALE1 & PWM_MOD_FREE;
    PWMCON1value = PWM_MOD1_IND & PWM_PEN1L & PWM_MOD2_IND & PWM_PEN2L &
                 PWM_MOD3_IND & PWM_PEN3L & PWM_MOD4_IND & PWM_PEN4L;
    PWMCON2value = PWM_SEVOPS4 & PWM_OSYNC_TCY & PWM_UEN;
    ConfigIntMCPWM(PWM_INT_DIS & PWM_FLTA_DIS_INT & PWM_FLTB_DIS_INT);

    //Call Microchip library setup function
    OpenMCPWM(PTPERvalue, SEVTCMPvalue, PTCONvalue, PWMCON1value, PWMCON2value);
   
}

void tiHSetFloat(unsigned int channel, float percent){
    unsigned int idx = channel - 1;

    unsigned int pdc_value;
    pdc_value = (unsigned int) (2 * percent / 100 * (10000));

    outputs[idx].throt_f = percent;
    outputs[idx].throt_i = pdc_value;

    if (pdc_value < 0){
        outputs[idx].dir = TIH_REV;
        pdc_value = -pdc_value;
    }

    //Select correct PWM output and GPIO level for dir and mode
    tiHConfigure(channel);

    //Set duty cycle
    SetDCMCPWM(channel, pdc_value, 0);
}

void tiHSetDC(unsigned int channel, int dutycycle){
    unsigned int idx = channel - 1;

    outputs[idx].throt_f = -666.0; //TODO: not a solution; have to update float every time?
    outputs[idx].throt_i = dutycycle;

    if (dutycycle < 0){
        outputs[idx].dir = TIH_REV;
        dutycycle = -dutycycle;
    }

    //Select correct PWM output and GPIO level for dir and mode
    tiHConfigure(channel);

    //Set duty cycle
    SetDCMCPWM(channel, dutycycle, 0);
}

void tiHChangeDirection(unsigned int channel, tiHDriveDir dir){
    outputs[channel].dir = dir;
    tiHConfigure(channel);
}

void tiHChangeMode(unsigned int channel, tiHDriveMode mode){
    outputs[channel].mode = mode;
    tiHConfigure(channel);
}

#define OUTPUT_PWM  1
#define OUTPUT_GPIO 0

void tiHConfigure(unsigned int channel) {
    unsigned int idx = channel - 1;
    unsigned char lbit, hbit;

    if (outputs[idx].dir == TIH_FWD) {
        hbit = OUTPUT_PWM;
        lbit = OUTPUT_GPIO;
    } else { //reverse
        hbit = OUTPUT_GPIO;
        lbit = OUTPUT_PWM;
    }

    /*
    LATEBITS LATEval = *((LATEBITS*)&LATE);
    PWMCON1BITS PWMCON1val = *((PWMCON1BITS*)&PWMCON1);

    //Setup which pin recieves GPIO / PWM
    if (channel == 1) {
        //Set up PWM1L/H
        LATEval.LATE0 = outputs[idx].mode;
        LATEval.LATE1 = outputs[idx].mode;
        PWMCON1val.PEN1H = hbit;
        PWMCON1val.PEN1L = lbit;
    } else if (channel == 2) {
        //Set up PWM1L/H
        LATEval.LATE2 = outputs[idx].mode;
        LATEval.LATE3 = outputs[idx].mode;
        PWMCON1val.PEN2H = hbit;
        PWMCON1val.PEN2L = lbit;
    } else if (channel == 3) {
        //Set up PWM1L/H
        LATEval.LATE4 = outputs[idx].mode;
        LATEval.LATE5 = outputs[idx].mode;
        PWMCON1val.PEN3H = hbit;
        PWMCON1val.PEN3L = lbit;
    } else if (channel == 4) {
        //Set up PWM1L/H
        LATEval.LATE6 = outputs[idx].mode;
        LATEval.LATE7 = outputs[idx].mode;
        PWMCON1val.PEN4H = hbit;
        PWMCON1val.PEN4L = lbit;
    }
    LATE = (unsigned int)LATEval;
    PWMCON1 = (unsigned int)PWMCON1val;
     
*/
    
    //Alternate methods?
    /*
    PWMCON1 = PWM_MOD4_IND & PWM_MOD3_IND & PWM_MOD2_IND & PWM_MOD1_IND &
            (outputs[0].dir << 1) | (outputs[0].dir << 0) |
            (outputs[0].dir << 3) | (outputs[0].dir << 2) |
            (outputs[0].dir << 5) | (outputs[0].dir << 3) |
            (outputs[0].dir << 7) | (outputs[0].dir << 6);

    LATE &= (outputs[0].mode << 0) | (outputs[0].mode << 1) |
            (outputs[1].mode << 2) | (outputs[1].mode << 3) |
            (outputs[2].mode << 4) | (outputs[2].mode << 5) |
            (outputs[3].mode << 6) | (outputs[3].mode << 7);
            */

    //Clear and set ONLY pertinent bits
    unsigned int PWMCON1val = PWMCON1;
    unsigned int LATEval = LATE;

    unsigned char bidx = 2*idx; //0,2,4,or 6

    //Direction
    PWMCON1val &= ~( 0b11 << bidx ); //clear
    PWMCON1val |=  ( (hbit << 1) | lbit ) << bidx; //set hbit/lbit
    //Mode
    LATEval &= ~( 0b11 << bidx ); //clear
    LATEval |= ((outputs[idx].mode << 1) | outputs[idx].mode) << bidx; //set
    
    LATE = LATEval; //Mode
    PWMCON1 = PWMCON1val; //Direction
}


void tiHTest(void)
{
    //this is a test to see if the H-Bridge chips work.  It will do a hacky setup
    //on the H-bridge, then run all of them alternatively forward and backwards,
    //1 second each, forever.

    //This setup is unique to PWM_OP_SCALE4
    //If the clock scaler is changed, this MUST be changed too!
    pwm_period = 32494;  //magic number found by doug that tends to make things work
    tiHSetupPeripheral();
    int i;
    for(i=0; i< NUM_PWM;i++)
        {
        outputs[i].throt_f = 0.0;
        outputs[i].throt_i = 0;
        outputs[i].mode = TIH_MODE_COAST;
        outputs[i].dir = TIH_FWD;
        }

    while(1)
    {

         PTPER = 0x7EEE;  //initalizes time components of the PWM
         PTCON = 0x8000;
         PWMCON1 = 0x00FF; //should have the lows do the inverses

         PDC1 = 0xA6B0; //sets duty cycle for #1.
         PDC2 = 0xA6B0; //sets duty cycle for #2.
         PDC3 = 0xA6B0; //sets duty cycle for #3.
         PDC4 = 0xA6B0; //sets duty cycle for #4.
        _PEN1L = 0;
        _PEN2L = 0;
        _PEN3L = 0;
        _PEN4L = 0;

    //all 4 motors should be running now.

        delay_ms(1000);

         PTPER = 0x7EEE;  //initalizes time components of the PWM
         PTCON = 0x8000;
         PWMCON1 = 0x00FF; //should have the lows do the inverses

         PDC1 = 0xA6B0; //sets duty cycle for #1.
         PDC2 = 0xA6B0; //sets duty cycle for #2.
         PDC3 = 0xA6B0; //sets duty cycle for #3.
         PDC4 = 0xA6B0; //sets duty cycle for #4.
        _PEN1H = 0;
        _PEN2H = 0;
        _PEN3H = 0;
        _PEN4H = 0;
        delay_ms(1000);
    }


    //Start all channels at 0 throttle in Forward / Coast mode
    tiHSetDC(1,0);
    tiHSetDC(2,0);
    tiHSetDC(3,0);
    tiHSetDC(4,0);
}