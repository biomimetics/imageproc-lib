#ifndef __PID_OBJ_H
#define __PID_OBJ_H

//Structures and enums
//PID Continer structure

typedef struct {
    int input;
    long dState, iState, preSat, p, i, d;
    int Kp, Ki, Kd, Kaw, y_old, output;
    unsigned char N;
    char onoff; //boolean
    long error;
    unsigned long start_time;
    int inputOffset;
    int feedforward;
    //char OUTPUT_CHANNEL;
} tPID_Controller;

//Functions
void pidObjSetup();
void pidObjInit(tPID_Controller* pidObj, int Kp, int Ki, int Kd, int Kaw, int ff);
void pidObjUpdate(tPID_Controller* pidObj, int feedback);
void pidObjSetInput(tPID_Controller* pidObj, int input_val);
void pidObjSetGains(tPID_Controller* pidObj, int Kp, int Ki, int Kd, int Kaw, int ff);
void pidObjOn(tPID_Controller* pidObj);
void pidObjOff(tPID_Controller* pidObj);

#endif