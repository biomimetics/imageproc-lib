#ifdef PID_SOFTWARE
#define AMS_DEFAULT_KP  200
#define AMS_DEFAULT_KI  5
#define AMS_DEFAULT_KD  0
#define AMS_DEFAULT_KAW 5
#define AMS_DEFAULT_KFF  0
#define SOFT_GAIN_SCALER 512
#elif defined PID_HARDWARE
#define AMS_DEFAULT_KP  500
#define AMS_DEFAULT_KI  50
#define AMS_DEFAULT_KD  0
#define AMS_DEFAULT_KAW 0
#define AMS_DEFAULT_KFF  0
#define AMS_PID_SCALER 8
#endif

// PID object with long type input
typedef struct {
    long input;
    long dState, iState, preSat, p, i, d;
    int Kp, Ki, Kd, Kaw, y_old, output;
    unsigned char N;
    char onoff; //boolean
    long error;
    unsigned long run_time;
    unsigned long start_time;
    int inputOffset;
    int Kff;
    int maxVal, minVal;
    int satValPos, satValNeg;
#ifdef PID_HARDWARE
    tPID dspPID;
#endif
} longpidObj;


//Setup PID for ams encoders
void amsPIDSetup(void)
void amsCtrlSetInput(void)