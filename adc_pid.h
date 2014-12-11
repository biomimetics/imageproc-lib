#ifndef __ADC_PID_H
#define __ADC_PID_H

void adcSetup(void); //Top level config function, to be called from main

//Getters for other modules to access values, by useful name
unsigned int adcGetVbatt();
unsigned int adcGetMotorA();
unsigned int adcGetMotorB();
unsigned int adcGetMotorC();
unsigned int adcGetMotorD();

//Getters for other modules to access values, by AN pin name
unsigned int adcGetAN0();
unsigned int adcGetAN8();
unsigned int adcGetAN9();
unsigned int adcGetAN10();
unsigned int adcGetAN11();

#ifndef ADC_MAX
#define ADC_MAX             853
#endif

#endif //__ADC_PID_H
