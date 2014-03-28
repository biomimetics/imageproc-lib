/*********************************************
 * Name: tih.c
 * Desc: Ti DRV8833 Hbridge driver
 * Date: 2012-8-18
 * Author: apullin
 *********************************************/

#include "tih.h"
#include "pwm.h"
#include "ports.h"
#include "init_default.h"

#define NUM_PWM 4
#define OUTPUT_PWM  1
#define OUTPUT_GPIO 0
#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

static tiHDriver outputs[4];
static int pwm_period;  //calculated at init, PWM module period register
static int max_pwm;     //calculated at init, special event compare reg for ADC
static int sevtcmp_pwm;

static void tiHSetupPeripheral(void);
static void tiHConfigure(unsigned int channel);

void tiHSetup(void) {

    //This setup is unique to PWM_IPCLK_SCALE1
    //If the clock scaler is changed, this MUST be changed too!
    pwm_period = (int)((float)FCY/((float)PWM_FREQ * 1.0))- 1;

    sevtcmp_pwm = (int)(ADC_TRIG_POINT * (float)pwm_period);

    max_pwm = 2*sevtcmp_pwm;

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

    SEVTCMPvalue = sevtcmp_pwm;
    PTCONvalue = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 &
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

    int pdc_value;
    pdc_value = (int) (2 * percent / 100 * (PTPER));

    outputs[idx].throt_f = percent;
    outputs[idx].throt_i = ABS(pdc_value);

    if (pdc_value < 0){
        outputs[idx].dir = TIH_REV;
        pdc_value = ABS(pdc_value);
    }
    else{
        outputs[idx].dir = TIH_FWD;
    }

    //Select correct PWM output and GPIO level for dir and mode
    tiHConfigure(channel);

    //Set duty cycle
    SetDCMCPWM(channel, pdc_value, 0);
}

void tiHSetDC(unsigned int channel, int dutycycle){
    unsigned int idx = channel - 1;
    if (dutycycle > max_pwm) dutycycle = max_pwm;
    if (dutycycle < -max_pwm) dutycycle = -max_pwm;
    outputs[idx].throt_f = -666.0; //TODO: not a solution; have to update float every time?
    outputs[idx].throt_i = dutycycle;

    if (dutycycle < 0){
        outputs[idx].dir = TIH_REV;
        dutycycle = -dutycycle;
    } else{outputs[idx].dir = TIH_FWD;}

    //Select correct PWM output and GPIO level for dir and mode
    tiHConfigure(channel);

    //Set duty cycle max = 0xfff
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


    if(outputs[idx].dir == TIH_FWD){
        if(outputs[idx].mode == TIH_MODE_BRAKE){
            hbit = OUTPUT_GPIO;
            lbit = OUTPUT_PWM;
        }
        else if(outputs[idx].mode == TIH_MODE_COAST){
            hbit = OUTPUT_PWM;
            lbit = OUTPUT_GPIO;
        }
    }
    else if(outputs[idx].dir == TIH_REV){
        if(outputs[idx].mode == TIH_MODE_BRAKE){
            hbit = OUTPUT_PWM;
            lbit = OUTPUT_GPIO;
        }
        else if(outputs[idx].mode == TIH_MODE_COAST){
            hbit = OUTPUT_GPIO;
            lbit = OUTPUT_PWM;
        }
    }

    unsigned int gpio_val = outputs[idx].mode;


    //Clear and set ONLY pertinent bits
    unsigned int PWMCON1val = PWMCON1;
    unsigned int LATEval = LATE;

    unsigned int bidx = 2*idx; //0,2,4, or 6

    //Direction
    PWMCON1val &= ~( 0b10001 << idx ); //clear PWMxH/L
    PWMCON1val |=  ( lbit << idx);     //set lbit
    PWMCON1val |=  ( hbit << (idx+4)); //set hbit
    //Mode
    LATEval &= ~( 0b11 << bidx ); //clear
    LATEval |= ((gpio_val << 1) | gpio_val) << bidx; //set

    LATE = LATEval; //Mode
    PWMCON1 = PWMCON1val; //Direction
}

//Getter for the maximum allowable throttle value
int tiHGetPWMMax(){
    return max_pwm;
}

int tiHGetPWMPeriod(){
    return pwm_period;
}

int tiHGetSignedDC(unsigned int channel){
    unsigned int idx = channel - 1;
    return outputs[idx].throt_i;
}