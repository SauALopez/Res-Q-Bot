/* INTERRUPT PINS FOR ENCODER */

#define ENC_RFW 18      //right front wheel encoder
#define ENC_LFW 19      //left front wheel encoder
#define ENC_RRW 20      //right rear wheel encoder
#define ENC_LRW 21      //left rear wheel encoder

/*PWM PINS FOR CONTROL OF WHEEL */

#define RFW 13   // right front wheel
#define LFW 12   // left front wheel
#define RRW 11   // right rear wheel
#define LRW 10   // left rear wheel

/* ENABLE PINS, FORDWARD & BACKWARD MOVEMENT */

#define FW_RFW 9
#define BW_RFW 8

#define FW_LFW 7
#define BW_LFW 6

#define FW_RRW 5
#define BW_RRW 4

#define FW_LRW 3
#define BW_LRW 2

/* VOLATILE VARIABLES FOR INTERUPTS */

//RPM VARAIBLES
volatile int RPM_RFW =0;
volatile int RPM_LFW =0;
volatile int RPM_RRW =0;
volatile int RPM_LRW =0;
//LAST MILLIS VARIABLES
volatile long last_RFW=0;
volatile long last_LFW=0;
volatile long last_RRW=0;
volatile long last_LRW=0;

/* PID - CONTROL SYSTEM */
double kp = 1;
double ki = 1;
double kd = 1;
