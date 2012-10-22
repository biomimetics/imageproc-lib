#ifdef PID_SOFTWARE
#define AMS_DEFAULT_KP  200
#define AMS_DEFAULT_KI  5
#define AMS_DEFAULT_KD  0
#define AMS_DEFAULT_KAW 5
#define AMS_DEFAULT_KFF  0
#define SOFT_GAIN_SCALER 512
#elif defined PID_HARDWARE
#define AMS_DEFAULT_KP  15000
#define AMS_DEFAULT_KI  100
#define AMS_DEFAULT_KD  150
#define AMS_DEFAULT_KAW 0
#define AMS_DEFAULT_KFF  0
#define AMS_PID_SCALER 32
#endif

#define nPIDS  2


//Setup PID for ams encoders
void amsPIDSetup(void);

void amsCtrlSetInput(unsigned char num, int state);

void amsCtrlPIDUpdate(unsigned char num, int state);

void amsCtrlSetGains(unsigned char num, int Kp, int Ki, int Kd, int Kaw, int ff);