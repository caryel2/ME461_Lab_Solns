#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c28/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "f28377sCoecsl.h"
#include "f28377sEqep.h"
#include "f28377sSerial.h"
#include "f28377sEPWM3A.h"
#include "f28377sDAC.h"
#include "f28377sADC.h"
#include "lcd.h"
#include "math.h"
#include "MadgwickAHRS.h"

void initSpibGpio(void);

//#define USEONLYBEMF
#define PRINTDEBUG_NOSIMULINK
//#define USE_SCIC_OLD_VERSION

#define PI        3.1415926535897932384626433832795
#define TWO_PI    6.2831853071795864769252867665590
#define HALF_PI   1.5707963267948966192313216916398

/***************************************************************************/
/***************************************************************************/
void matlab_serialRX(serial_t *s, char data);
void simulink_serialRX(serial_t *s, char data);
void setupSpib(void);
void setupSpia(void);
void initAngles (void);
void serialRXA(serial_t *s, char data);


#ifdef _FLASH
// These are defined by the linker (see device linker command file)
extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadSize;
extern unsigned int RamfuncsRunStart;
#endif

int adcb0result = 0;
int adcb1result = 0;

#define NUM_SEND_QUEUES 120
#define MAX_SEND_LENGTH 1600
#define MAX_VAR_NUM 10

#pragma DATA_SECTION(matlabLock, ".my_vars")
float matlabLock = 0;
float matlabLockshadow = 0;

// UART 1 GLOBAL VARIABLES
int UARTsensordatatimeouterror = 0; // Initialize timeout error count
int UARTtransmissionerror = 0;          // Initialize transmission error count

int UARTbeginnewdata = 0;
int UARTdatacollect = 0;
char UARTMessageArray[101];
int UARTreceivelength = 0;

char Main_sendingarray = 0;  // Flag to Stop terminal prints when using matlab commands
//  Only way to clear this flag

union mem_add {
    float f;
    long i;
    char c[2];
}memloc;

union ptrmem_add {
    float* f;
    long* i;
    char c[2];
}ptrmemloc;

long* Main_address[MAX_VAR_NUM];
float Main_value[MAX_VAR_NUM];
char Main_SendArray[128];
char Main_SendArray2[128];
float Main_tempf=0;
int Main_i = 0;
int Main_j = 0;
int Main_memcount = 0;

char SIMU_databyte1 = 0;
int SIMU_Var1_fromSIMU_16bit = 0;
int SIMU_Var2_fromSIMU_16bit = 0;
int SIMU_Var3_fromSIMU_16bit = 0;
int SIMU_Var4_fromSIMU_16bit = 0;
int SIMU_Var5_fromSIMU_16bit = 0;
int SIMU_Var6_fromSIMU_16bit = 0;
int SIMU_Var7_fromSIMU_16bit = 0;

char SIMU_TXrawbytes[12];

long SIMU_Var1_toSIMU_32bit = 0;
long SIMU_Var2_toSIMU_32bit = 0;
long SIMU_Var3_toSIMU_32bit = 0;
long SIMU_Var4_toSIMU_32bit = 0;
long SIMU_Var5_toSIMU_32bit = 0;
long SIMU_Var6_toSIMU_32bit = 0;
long SIMU_Var7_toSIMU_32bit = 0;

long SIMU_Var1_toSIMU_16bit = 0;
long SIMU_Var2_toSIMU_16bit = 0;
long SIMU_Var3_toSIMU_16bit = 0;
long SIMU_Var4_toSIMU_16bit = 0;
long SIMU_Var5_toSIMU_16bit = 0;
long SIMU_Var6_toSIMU_16bit = 0;
long SIMU_Var7_toSIMU_16bit = 0;

int SIMU_beginnewdata = 0;
int SIMU_datacollect = 0;
int SIMU_Tranaction_Type = 0;
int SIMU_checkfirstcommandbyte = 0;


/* Swi handle defined in swi.cfg */
extern const Swi_Handle swi_print;
extern const Swi_Handle swi_balanceControl;

//pwm clock = sysclk/2 by default = 100Mhz
//time based clock = pwmclock /(clkdiv*hspclkdiv) = 100Mhz/(1*2) = 50Mhz
//want 20khz so pwmCountMax = 50Mhz/20Khz = 2500;=
int pwmCountMax = 2500;     //want a 20khz signal.
int pwmIncr = 125;          //increment value, set to pwmCountMax/20 so that steps b/w -10 and +10
long pwmVal = 0;


/***************************************************************************/
/***************************************************************************/
//IMU variables
//note: MPU9250 connected to SPIB

volatile float accelx=0;
volatile float accely=0;
volatile float accelz=0;
volatile float gyrox=0;
volatile float gyroy=0;
volatile float gyroz=0;
volatile float mx=0;
volatile float my=0;
volatile float mz=0;
volatile float accelx_offset=0;
volatile float accely_offset=0;
volatile float accelz_offset=0;
volatile float gyrox_offset=0;
volatile float gyroy_offset=0;
volatile float gyroz_offset=0;
volatile float roll_offset=0;
volatile float pitch_offset=0;
volatile float compass_offset=0;
float accelxSide = 0; //1.485;//1.446;//1.4322; //1.4435;//0.7180;       //accel x offset for when starting on the side
float accelxSideStart = 0; //1.485;//1.4322; //1.4435;//0.7180;
float gravitySide = 1.0; // 0.70;       //gravity offset for accely when starting on the side.
float dummyVar = 0;
char buffer[100];
int16 IMU_data[9];
float offset = 0;

// Variables related to Madgwick algorithm
float Quaternions[4];
float DotQuaternions[4];
float mdg_beta = 0.1;

Uint16 count=0;
Uint16 newdata=0;
Uint16 read=0;
Uint16 calc_offset=0;

// Joey added constants
Uint16 XG_OFFSET_H = 0x1300;
Uint16 XG_OFFSET_L = 0x1400;
Uint16 YG_OFFSET_H = 0x1500;
Uint16 YG_OFFSET_L = 0x1600;
Uint16 ZG_OFFSET_H = 0x1700;
Uint16 ZG_OFFSET_L = 0x1800;
Uint16 SMPLRT_DIV = 0x1900;
Uint16 CONFIG = 0x1A00;
Uint16 GYRO_CONFIG = 0x1B00;
Uint16 ACCEL_CONFIG = 0x1C00;
Uint16 ACCEL_CONFIG_2 = 0x1D00;
Uint16 LP_ACCEL_ODR = 0x1E00;
Uint16 WOM_THR = 0x1F00;
Uint16 FIFO_EN = 0x2300;
Uint16 I2C_MST_CTRL = 0x2400;
Uint16 I2C_SLV0_ADDR = 0x2500;
Uint16 I2C_SLV0_REG = 0x2600;
Uint16 I2C_SLV0_CTRL = 0x2700;
Uint16 I2C_SLV1_ADDR = 0x2800;
Uint16 I2C_SLV1_REG = 0x2900;
Uint16 I2C_SLV1_CTRL = 0x2A00;
Uint16 INT_ENABLE = 0x3800;
Uint16 INT_STATUS = 0x3A00;
Uint16 I2C_SLV1_DO = 0x6400;
Uint16 I2C_MST_DELAY_CTRL = 0x6700;
Uint16 USER_CTRL = 0x6A00;
Uint16 PWR_MGMT_1 = 0x6B00;
Uint16 WHO_AM_I = 0x7500;
Uint16 XA_OFFSET_H = 0x7700;
Uint16 XA_OFFSET_L = 0x7800;
Uint16 YA_OFFSET_H = 0x7A00;
Uint16 YA_OFFSET_L = 0x7B00;
Uint16 ZA_OFFSET_H = 0x7D00;
Uint16 ZA_OFFSET_L = 0x7E00;
Uint16 temp=0;
Uint16 senddata = 0;
Uint16 readdata[25];  // received data

Uint16 acc_config2=0;
float x_A_offs_add = 0.0; //.100;
float y_A_offs_add = 0.0; //-.07;
float z_A_offs_add = 0.0; //.070;

float x_G_offs = 0.0; //1;
float y_G_offs = 0.0; //6;
float z_G_offs = 0.0; //1;

float compass_angle = 0.0;

float verr = 0;
float vel2_ref = 0;
float ik = 0;
float ik_old = 0;
float verr_old = 0;
float ki = 10;
float Kp = .1;
float Kd = 0;
float thetaRef = 0;
float vel2 = 0;


float max_mx = 40;
float min_mx = 40;
float max_my = 10;
float min_my = 10;
float max_mz = -30;
float min_mz = -30;

float mx_offset = (-53+51)/2; //(74.0 + (-2.0))/2.0;
float my_offset = (0+90)/2; //(58.0 + (-40.0))/2.0;
float mz_offset = (-2-99)/2; //(21.0 + (-84.0))/2.0;

float Kpcenter = -0.0003;
float Kon = 1.0;
float velf = 0;
float vel_orig = 0;
float velfold = 0;
float accelxSideAdjust = 0.0;
float SideAdjust_error = 0.0;
float SideAdjust_I = 0.0;
float SideAdjust_Iold = 0.0;
float SideAdjust_errorOld = 0.0;
float Kicenter = -.0001;

float ADC1V;
float ADC0V;
float ADC0V_scaled;

typedef struct {
    char  first;   // Is first measurement?
    int   initial; // Initial value
    int   d;       // Digital value
    int   d_old;   // Previous digital value
    float f;       // Float value
    float f_old;   // Previous float value
    long  l;       // Long value
    float cos;     // Cosine approximation
} Angle;

Angle theta;

unsigned int spiaRxData[4] = {0, 0, 0, 0};
unsigned int spiaTxData = 0xAA;
//char spiaDone = 1;          //start at 1 so that first transmission happens
//char spibDone = 1;

int numOverReadA = 0;

/***************************************************************************/
//control variables
float u = 0;        //control effort, u = ref-actual
float K[4] = {-695, -45, 0, -0.045};     // Control coefficients
//float K[4] = {-700, -90.2, 0, -0.045};     // Control coefficients
//float K[4] = {-572.2306, -68.1386, 0, -0.2156};    // Upside down Control coefficients
float frictK = 0.5;

float vel = 0;      //calculated velocity
float thetaR = 0;       //rotar angle = phiP + phiR, phiR = rotar angle = theta.f
float thetaRold = 0;
float targetAngle = 0;
float gyroOff = 0.0;
char doneCal = 0;


// filter variables
float tiltfK[3] = {0};
float tiltK[2] = {0};
float gyrofK[3] = {0};
float gyroK[2] = {0};
float tilt = 0;
float T = 0.001;        //sample rate, 1ms
float tau = 2;

// Variables for Pitch Kalman Filter
float pitch_Q = 0.01; // made global to enable changing in runtime
float pitch_R = 500; //50000;
float pitch_kalman_tilt = 0;
float pitch_kalman_P = 22.365;

// Variables for Roll Kalman Filter
float roll_Q = 0.01; // made global to enable changing in runtime
float roll_R = 5000; //50000;
float roll_kalman_tilt = 0;
float roll_kalman_P = 22.365;

// Variables for Roll Kalman Filter
float yaw_Q = 0.01; // made global to enable changing in runtime
float yaw_R = 5000; //50000;
float yaw_kalman_tilt = 0;
float yaw_kalman_P = 22.365;

float th_array[4] = {0,0,0,0};
int SpiaNumCalls = 0;
float th_value = 0;
float th_old = 0;
int SpibNumCalls = 0;

float pitch_tilt_value = 0;
float pitch_set_point = 0;
float pitch_tilt_array[4] = {0,0,0,0};
float pitch_gyro_value = 0;
float pitch_gyro_array[4] = {0,0,0,0};

float roll_tilt_value = 0;
float roll_set_point = 0;
float roll_tilt_array[4] = {0,0,0,0};
float roll_gyro_value = 0;
float roll_gyro_array[4] = {0,0,0,0};

float yaw_tilt_value = 0;
float yaw_set_point = 0.0;
float yaw_tilt_array[4] = {0,0,0,0};
float yaw_gyro_value = 0;
float yaw_gyro_array[4] = {0,0,0,0};

long numSpibposts_debug = 0;
long numSpiaposts_debug = 0;

/***************************************************************************/
/***************************************************************************/

unsigned long timeVar = 0;
unsigned long countVar = 0;
unsigned int countFlag = 0;

void main(void)
{
    // Copy time critical code and Flash setup code to RAM
    // This includes InitFlash(), Flash API functions and any functions that are
    // assigned to ramfuncs section.
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the device .cmd file.
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
#ifdef _FLASH
    InitFlash_Bank0();
    InitFlash_Bank1();
#endif
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xS_SysCtrl.c file.
    InitSysCtrl();

    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xS_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    InitGpio();

    init_EQEPs();

    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);                     //toggle led setup
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);       //toggle led setup
    GpioDataRegs.GPASET.bit.GPIO12 = 1;

    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);                     //toggle led setup
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_PUSHPULL);       //toggle led setup
    GpioDataRegs.GPASET.bit.GPIO13 = 1;

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 0);                     //toggle led setup
    GPIO_SetupPinOptions(63, GPIO_OUTPUT, GPIO_PUSHPULL);       //toggle led setup
    GpioDataRegs.GPBSET.bit.GPIO63 = 1;
    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);                     //toggle led setup
    GPIO_SetupPinOptions(41, GPIO_OUTPUT, GPIO_PUSHPULL);       //toggle led setup
    GpioDataRegs.GPBSET.bit.GPIO41 = 1;

    //ANDREW    GPIO78  Motor Enable
    GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(78, GPIO_OUTPUT, GPIO_PUSHPULL);

    //ANDREW    GPIO92  Pulse
    GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);

    InitEPwm3Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

#ifdef USE_SCIC_OLD_VERSION
    init_serial(&SerialA,115200,NULL);
#else
    // RAMYA: for tuning the parameters
    init_serial(&SerialA,115200,serialRXA);
#endif

    init_serial(&SerialB,115200,NULL);

#ifdef USE_SCIC_OLD_VERSION
    init_serial(&SerialC,115200,simulink_serialRX);
#else
    init_serial(&SerialC,19200,NULL);
#endif

    //init_lcd(90);
    InitSpiaGpio();     //for as5145h (rotary encoder)
    initSpibGpio();     //for mpu9250 (imu)
    DELAY_US(200);

    setupSpia();        //set up spi for as5145h (must be before b because pie enables are at end of b)
    DELAY_US(200000);

    setupSpib();        //set up spi for mpu9250
    DELAY_US(200);

    initAngles();       //clear angles variable

    //Configure the ADCs and power them up
    ConfigureADC();

    //Setup the ADCs for software conversions
    SetupADCSoftware();

    initEPwm3A();
    initEPwm7();
    initEPwm8();
    setEPWM3A(0);
    initDACs();

// ************ your Init code here  *******************


//    UART_printfLine(1, "Digital Control");
//    UART_printfLine(2, "of Dynamic Systems");
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_FREEZE; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0000; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 6; // divide by 2^6=64 50Mhz divide by 64 is 781250 Hz
    EPwm5Regs.TBPRD = 390; // 50ms sample actually 781.25 is 1000Hz.
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; //unfreeze, and enter up count mode
    EDIS;

// ************ End init code *************************

    // !!!!!!!!!!!!!!!!!  NO CODE below here in Main() !!!!!!!!!!!!!!!!!!!!!
    // !!!!!!!!!!!!!!!!!  All inits above !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // Disable CPU interrupts
    DINT;
    // Clear all CPU interrupt flags:
    IFR = 0x0000;
    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

#warn Maybe move above DINT?
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP6 | PIEACK_GROUP8 | PIEACK_GROUP9);  // ACKnowledge any SCI interrupt requests

    BIOS_start();

}

//clock0
//should happen every millisecond
void clkFunc(void)
{
    timeVar++;

//    if((timeVar%4) == 0)
//    {
//        if(spiaDone == 1 && spibDone == 1)
//        {
//            if(doneCal == 1)
//            {
//                Swi_post(swi_balanceControl);
//                GpioDataRegs.GPBSET.bit.GPIO41 = 1;         //blink blue led d10 to show cpu running
//            }
//            if(timeVar%200 == 0) {
//                Swi_post(swi_print);
//            }
//            spiaDone = 0;
//            spibDone = 0;
//        }
//    }


    if((timeVar%200) == 0)
    {

        Swi_post(swi_print);

        if(doneCal == 0)
            GpioDataRegs.GPBTOGGLE.bit.GPIO41 = 1;          //blink blue led d10 to show cpu running
    }

    if((timeVar%1000) == 0)
    {
        //GpioDataRegs.GPBTOGGLE.bit.GPIO41 = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO63 = 1;
    }


    int i;
    //com with rotary encoder
    SpiaNumCalls++;
    SpiaRegs.SPIFFRX.bit.RXFFIL = 3;                //Going to R/W 3 entries
    GpioDataRegs.GPCCLEAR.bit.GPIO73 = 1;           //pull cs low to get ready to tx/rx
    for(i = 0; i<3; i++)
        SpiaRegs.SPITXBUF = (unsigned) 0xAA<<8;     //write to spia, need to issue 3
    //left shift 8 to left the 8 bits

    SpibNumCalls++;
    //com with imu
    SpibRegs.SPIFFRX.bit.RXFFIL = 7;                    //Going to R/W 7 (16 bit) entries
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;               //Pull CS low to get ready to transmit/receive
    if(count<3000) count++;
    else
    {
        if(calc_offset==0)
        {
            calc_offset = 1;
            count = 0;
        }
        else if(calc_offset == 1)
            calc_offset = 2;
    }

    senddata = ((0x8000)|(0x3B00));                     //"Start Data" Register to start reading data from - ACCEL_XOUT_H
    read = 1;                                           //Data to be read will be accel data
    for(i=0;i<7;i++)
    {
        SpibRegs.SPITXBUF = senddata+i*256;             //!!!! Transmit 7 read commands to read in all (16 bit) parts of accelerometer
    }
}



void spiaRxFifoIsr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO73 = 1;     //pull cs high as done r/w
    spiaRxData[0] = SpiaRegs.SPIRXBUF & 0xFF;   //received data right justified
    spiaRxData[1] = SpiaRegs.SPIRXBUF & 0xFF;
    spiaRxData[2] = SpiaRegs.SPIRXBUF & 0xFF;

    //note: weird overflow 4x
    while(SpiaRegs.SPIFFRX.bit.RXFFST != 0)
    {
        spiaRxData[3] = SpiaRegs.SPIRXBUF & 0xFF;
        numOverReadA++;
    }

    theta.d = (((unsigned int) (spiaRxData[0] & 0x7F)) << 5) + (((unsigned int) (spiaRxData[1] & 0xF8)) >> 3);

    if (theta.first == 1)
    {
        theta.first = 0;
        theta.l = theta.d;
        theta.d_old = theta.d;
        theta.initial = theta.d;
    }
    else
    {
        if (abs(theta.d - theta.d_old) > 1000)
        {  // 1000 number figured for 2 ms sample rate and max speed
            // We have roll over
            if (theta.d > theta.d_old)
                theta.l = theta.l + (theta.d - theta.d_old) - 4096;

            else
                theta.l = theta.l + (theta.d - theta.d_old) + 4096;
        }

        else // No roll over
            theta.l = theta.l + (theta.d - theta.d_old);

        theta.d_old = theta.d;
    }

    theta.f = ((theta.l-theta.initial)*TWO_PI)/(4096.0);

    th_array[SpiaNumCalls] = theta.f;
    if (SpiaNumCalls >= 3) {
        th_value = (th_array[0] + th_array[1] + th_array[2] + th_array[3])/4.0;
        SpiaNumCalls = 0;
        if (SpibNumCalls == 0) {  // If SpibNumCalls happens to be done already Post Balance Control swi
            if (doneCal == 1) {
                numSpiaposts_debug++;
                Swi_post(swi_balanceControl);
            }
        }
    }
    //    spiaDone = 1;

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

long timecount = 0;

// Variables for Pitch Kalman Filter
float pitch_K = 0.0;
float pitch_tiltrate = 0.0;
float pitch_pred_tilt = 0.0;
float pitch_pred_P = 0.0;

// Variables for Roll Kalman Filter
float roll_K = 0.0;
float roll_tiltrate = 0.0;
float roll_pred_tilt = 0.0;
float roll_pred_P = 0.0;

// Variables for Yaw Kalman Filter
float yaw_K = 0.0;
float yaw_tiltrate = 0.0;
float yaw_pred_tilt = 0.0;
float yaw_pred_P = 0.0;

//float mpu_pitch = 0.0; float mpu_roll = 0.0;
float mdg_pitch = 0.0;
float mdg_roll = 0.0;
float mdg_yaw = 0.0;

//spibrx pie interrupt
void spibRxFifoIsr(void)
{
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;                 //Pull CS high as done R/W
    Uint16 i;

    if(read==1){
        for(i=0;i<7;i++){
            readdata[i] = SpibRegs.SPIRXBUF;            //!!!! Store the 6 read commands of the accelerometer
        }
        SpibRegs.SPIFFRX.bit.RXFFIL = 15;   //17            //Going to R/W 17 (16 bit) entries
        GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;           //Pull CS low to get ready to transmit/receive
        for(i=7;i<22;i++){ //25
            SpibRegs.SPITXBUF = senddata+i*256;         //!!!! Transmit 17 read commands to read in all gyro and magn measurements
        }
        read = 2;                                       //Accel data has to be read first, then other data can be read
    }
    else if(read == 2){
        newdata = 1;                                    //Signal new data to be read
        for(i=7;i<22;i++){  // 24
            readdata[i] = SpibRegs.SPIRXBUF;            //!!! Store the 17 read commands of the gyro and magn
        }

        for(i=0;i<3;i++){
            IMU_data[i] = readdata[2*i+1]<<8;               //Combine high and low bits of the accelerometer
            IMU_data[i]+=(readdata[2*i+2] & 0x00FF);
        }
        for(i=3;i<6;i++){
            IMU_data[i] = readdata[2*i+3]<<8;               //Combine high and low bits of the gyro
            IMU_data[i]+=(readdata[2*i+4] & 0x00FF);
        }
        for(i=6;i<9;i++){
            IMU_data[i] = readdata[2*i+5]<<8;               //Combine high and low bits of compass
            IMU_data[i]+=(readdata[2*i+4] & 0x00FF);
        }

        //Perform operations on sensor data to normal form.
        accelx = (((float)(IMU_data[0]))*4.0/32767.0)-x_A_offs_add; //19.6133
        accely = (((float)(IMU_data[1]))*4.0/32767.0)-y_A_offs_add;
        accelz = (((float)(IMU_data[2]))*4.0/32767.0)-z_A_offs_add;

        //mpu_roll = atan2(accely, accelz);
        //mpu_pitch = atan2(-accelx, sqrt(accely*accely + accelz*accelz) );

        gyrox  = (((float)(IMU_data[3]))*250.0/32767.0)-x_G_offs;
        gyroy  = (((float)(IMU_data[4]))*250.0/32767.0)-y_G_offs;
        gyroz  = (((float)(IMU_data[5]))*250.0/32767.0)-z_G_offs;

        mx     = (((float)(IMU_data[6]))*1.0/1.0);
        my     = (((float)(IMU_data[7]))*1.0/1.0);
        mz     = (((float)(IMU_data[8]))*1.0/1.0);

        mx = mx - mx_offset;
        my = my - my_offset;
        mz = mz - mz_offset;


        if (timecount > 1000) {
            if (mx > max_mx) {
                if (mx < 250) max_mx = mx;
            }
            if (mx < min_mx) {
                if (mx > -250) min_mx = mx;
            }
            if (my > max_my) {
                if (my < 250) max_my = my;
            }
            if (my < min_my) {
                if (my > -250) min_my = my;
            }
            if (mz > max_mz) {
                if (mz < 250) max_mz = mz;
            }
            if (mz < min_mz) {
                if (mz > -250) min_mz = mz;
            }
        }

        if (mx>0){
            compass_angle = (((float)atan((my/mx)))*180.0/PI);
            if (compass_angle <0) {
                compass_angle = 360 + compass_angle;
            }
        }
        else if (mx<0) {
            compass_angle = 180 + (((float)atan((my/mx)))*180.0/PI);
        }

        if(calc_offset == 0) {
        }
        if(calc_offset==1){
            accelx_offset+=accelx; accely_offset+=accely; accelz_offset+=accelz;
            //roll_offset+=mpu_roll; pitch_offset+=mpu_pitch;
            gyrox_offset+=gyrox; gyroy_offset+=gyroy; gyroz_offset+=gyroz;
            compass_offset+=compass_angle;
        }
        else if(calc_offset==2){
            accelx_offset/=3000.0;accely_offset/=3000.0; accelz_offset/=3000.0;
            //roll_offset/=3000; pitch_offset/=3000;
            gyrox_offset/=3000.0;gyroy_offset/=3000.0;gyroz_offset/=3000.0;
            compass_offset/=3000;
            calc_offset = 3;
        }
        else if(calc_offset==3){
            doneCal = 1;
            accelx -= accelx_offset; accely -=accely_offset; accelz -=(accelz_offset-gravitySide); //-accelxSide
            gyrox -= gyrox_offset;gyroy -= gyroy_offset;gyroz -= gyroz_offset;
            /*mpu_roll -=roll_offset; mpu_pitch -=pitch_offset; compass_angle -=compass_offset; */

            // Adding new stuff related to Madgwick algorithm -> Ramya
            MadgwickAHRSupdateIMU(Quaternions, DotQuaternions, mdg_beta,
                                  (PI/180.0)*gyrox,
                                  (PI/180.0)*gyroy,
                                  (PI/180.0)*gyroz,
                                  accelx, accely, accelz);

            double q0 = Quaternions[0];
            double q1 = Quaternions[1];
            double q2 = Quaternions[2];
            double q3 = Quaternions[3];

            double R11 = 2.0*q0*q0 -1 + 2.0*q1*q1;
            double R21 = 2.0*(q1*q2 - q0*q3);
            double R31 = 2.0*(q1*q3 + q0*q2);
            double R32 = 2.0*(q2*q3 - q0*q1);
            double R33 = 2.0*q0*q0 -1 + 2.0*q3*q3;

            mdg_roll = atan2( R32, R33 );
            mdg_pitch = -atan( R31 / sqrt(1-R31*R31) );
            mdg_yaw = atan2( R21, R11 );


            /*--------------Pitch Kalman Filtering code start---------------------------------------------------------------------*/
            float pitch_z, pitch_S;

            //Measurements from accelerometer-x and gyroscope-y
            pitch_tiltrate = (gyroy*PI)/180.0; //z-axis: (gyroz*PI)/180.0; //y-axis: (gyroy*PI)/180.0; //x-axis: (gyrox*PI)/180.0; // rad/s
            pitch_z = (accelz*PI)/2.0;

            // Prediction Step for pitch
            pitch_pred_tilt = pitch_kalman_tilt + T*pitch_tiltrate;
            pitch_pred_P = pitch_kalman_P + pitch_Q;

            // Update Step  for pitch
            pitch_S = (pitch_pred_P + pitch_R);
            pitch_K = pitch_pred_P/pitch_S;
            pitch_kalman_tilt = (1-pitch_K)*pitch_pred_tilt + pitch_K*pitch_z;
            pitch_kalman_P = (1 - pitch_K)*pitch_pred_P;

            // Store Kalman Estimates for Pitch
            pitch_tilt_array[SpibNumCalls] = pitch_kalman_tilt;
            pitch_gyro_array[SpibNumCalls] = pitch_tiltrate;

            /*--------------Roll Kalman Filtering code start---------------------------------------------------------------------*/
            float roll_z, roll_S;

            //Measurements from accelerometer-y and gyroscope-x
            roll_tiltrate = (gyrox*PI)/180.0; //z-axis: (gyroz*PI)/180.0; //y-axis: (gyroy*PI)/180.0; //x-axis: (gyrox*PI)/180.0; // rad/s
            roll_z = (accely*PI)/2.0;

            // Prediction Step for roll
            roll_pred_tilt = roll_kalman_tilt + T*roll_tiltrate;
            roll_pred_P = roll_kalman_P + roll_Q;

            // Update Step  for roll
            roll_S = (roll_pred_P + roll_R);
            roll_K = roll_pred_P/roll_S;
            roll_kalman_tilt = (1-roll_K)*roll_pred_tilt + roll_K*roll_z;
            roll_kalman_P = (1 - roll_K)*roll_pred_P;

            // Store Kalman Estimates for Roll
            roll_tilt_array[SpibNumCalls] = roll_kalman_tilt;
            roll_gyro_array[SpibNumCalls] = roll_tiltrate;


            /*--------------Yaw Kalman Filtering code start---------------------------------------------------------------------*/
            float yaw_z, yaw_S;

            //Measurements from compass_angle and gyroscope-z
            yaw_tiltrate = (gyroz*PI)/180.0; //z-axis: (gyroz*PI)/180.0; //y-axis: (gyroy*PI)/180.0; //x-axis: (gyrox*PI)/180.0; // rad/s
            yaw_z = (compass_angle*PI)/180;

            // Prediction Step for yaw
            yaw_pred_tilt = yaw_kalman_tilt + T*yaw_tiltrate;
            yaw_pred_P = yaw_kalman_P + yaw_Q;

            // Update Step  for yaw
            yaw_S = (yaw_pred_P + yaw_R);
            yaw_K = yaw_pred_P/yaw_S;
            yaw_kalman_tilt = (1-yaw_K)*yaw_pred_tilt + yaw_K*yaw_z;
            yaw_kalman_P = (1 - yaw_K)*yaw_pred_P;

            // Store Kalman Estimates for Roll
            yaw_tilt_array[SpibNumCalls] = yaw_kalman_tilt;
            yaw_gyro_array[SpibNumCalls] = yaw_tiltrate;

            if (SpibNumCalls >= 3) {  // should never be greater than 3
                pitch_tilt_value = (pitch_tilt_array[0] + pitch_tilt_array[1] + pitch_tilt_array[2] + pitch_tilt_array[3])/4.0;
                pitch_gyro_value = (pitch_gyro_array[0] + pitch_gyro_array[1] + pitch_gyro_array[2] + pitch_gyro_array[3])/4.0;

                roll_tilt_value = (roll_tilt_array[0] + roll_tilt_array[1] + roll_tilt_array[2] + roll_tilt_array[3])/4.0;
                roll_gyro_value = (roll_gyro_array[0] + roll_gyro_array[1] + roll_gyro_array[2] + roll_gyro_array[3])/4.0;

                yaw_tilt_value = (yaw_tilt_array[0] + yaw_tilt_array[1] + yaw_tilt_array[2] + yaw_tilt_array[3])/4.0;
                yaw_gyro_value = (yaw_gyro_array[0] + yaw_gyro_array[1] + yaw_gyro_array[2] + yaw_gyro_array[3])/4.0;

                SpibNumCalls = 0;

                if (SpiaNumCalls == 0) {  // If SpiaNumCalls is zero then Spia also done so Post Balance Control swi
                    if (doneCal == 1) {
                        numSpibposts_debug++;
                        Swi_post(swi_balanceControl);
                    }
                }
            }


        }
        timecount++;
//        spibDone = 1;
    }



    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

/*if(my > 0) {
    compass_angle = 90.0-(((float)atan((mz/my)))*180.0/PI);
}
else if(my < 0) {
    compass_angle = 270.0-(((float)atan((mz/my)))*180.0/PI);
}
else if((my == 0) && (mz < 0)) {
    compass_angle = 180.0;
}
if((my == 0) && (mz > 0)) {
    compass_angle = 0.0;
}
compass_angle = -compass_angle; */

/*pitch_tilt_value = 0.0; pitch_gyro_value = 0.0;
roll_tilt_value = 0.0; roll_gyro_value = 0.0;
yaw_tilt_value = 0.0; yaw_gyro_value = 0.0;

for (jj=0; jj<avg_value; jj++) {
    pitch_tilt_value += pitch_tilt_array[jj];
    pitch_gyro_value += pitch_gyro_array[jj];

    roll_tilt_value += roll_tilt_array[jj];
    roll_gyro_value += roll_gyro_array[jj];

    yaw_tilt_value += yaw_tilt_array[jj];
    yaw_gyro_value += yaw_gyro_array[jj];
}

pitch_tilt_value /= avg_value; pitch_gyro_value /= avg_value;
roll_tilt_value /= avg_value; roll_gyro_value /= avg_value;
yaw_tilt_value /= avg_value; yaw_gyro_value /= avg_value; */




// ANDREW adcb1 pie interrupt
void Hw_ADCB(void)
{
    countVar++;

    if((countVar%9) == 0)
    {
#ifdef USEONLYBEMF
        GpioDataRegs.GPCSET.bit.GPIO78 = 1;
#endif
        countFlag = 1;
    }
    else
    {
        if(1==countFlag)
        {
            adcb0result = AdcbResultRegs.ADCRESULT0;
            adcb1result = AdcbResultRegs.ADCRESULT1;
            ADC0V = (adcb0result)*(3.0/4096);
            ADC1V = (adcb1result)*(3.0/4096);
#ifdef USEONLYBEMF
            GpioDataRegs.GPCTOGGLE.bit.GPIO92 = 1;
            GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;
#endif
            countFlag = 0;
        }
        else
        {
            GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;

        }



        ////// DOUBLE SAMPLE TIME
        // if((countVar%9) == 0)
        // {
        //     GpioDataRegs.GPCSET.bit.GPIO78 = 1;
        //     countFlag = 1;
        // }
        // else
        // {
        //     if(1==countFlag)
        //     {
        //         adcb0result = AdcbResultRegs.ADCRESULT0;
        ////         adcb1result = AdcbResultRegs.ADCRESULT1;
        //         ADC0V = (adcb0result)*(3.0/4096);
        ////         ADC1V = (adcb1result)*(3.0/4096);
        //
        //         GpioDataRegs.GPCTOGGLE.bit.GPIO92 = 1;
        //
        //         countFlag = 2;
        //     }
        //     else if(2==countFlag)
        //     {
        //         GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;
        //
        //         countFlag = 0;
        //     }
        //     else
        //     {
        //         GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;
        //
        //     }

        //// WORKING?
        //     if((countVar%9) == 0)
        //  {
        //      GpioDataRegs.GPCSET.bit.GPIO78 = 1;
        //      countFlag = 1;
        //  }
        //  else
        //  {
        //      if(1==countFlag)
        //      {
        //          adcb0result = AdcbResultRegs.ADCRESULT0;
        // //         adcb1result = AdcbResultRegs.ADCRESULT1;
        //          ADC0V = (adcb0result)*(3.0/4096);
        // //         ADC1V = (adcb1result)*(3.0/4096);
        //
        //          GpioDataRegs.GPCTOGGLE.bit.GPIO92 = 1;
        //
        //          countFlag = 2;
        //      }
        //      else if(2==countFlag)
        //      {
        //          GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;
        //
        //          countFlag = 0;
        //      }
        //      else
        //      {
        //          GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;
        //
        //      }

    }



    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
}

int sign(float x) {
    return (x > 0) - (x < 0);
}

int compare(float x) {
    return (x > 0.015) - (x <= 0.015);
}

long timecnt = 0;
float prop_u = 0;

float prop_l = 0.11;
float kF = 1.3275e-07;

float Ky[6] = {0.0, 0.0, 0.06, 0.008, 0.0, 0.0};
//{0.05, 0.0198, 0.06, 0.008, 0.0, 0.0};

float cnt_u[4] = {0.0};
float Winv[4*4];
float omega_sqr[4] = {0.0};
float omega[4] = {0.0};
float MAX_OMEGA = 1050.0*1050.0; // rad/s
float MIN_OMEGA = 110.0*110.0;  // rad/s
float prop_cnt[4] = {0.0};
float mass = 0.15;
float kmass = 0.25;

void balanceControl(void)
{
    /////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    float kM = 0.0150867824*kF;
    float twolkF = 1.0/(2.0*prop_l*kF);
    float fourkF = 1.0/(4.0*kF);
    float fourkM = 1.0/(4.0*kM);

    cnt_u[0] =  0.0; //-Ky[2]*(roll_tilt_value-roll_set_point)-Ky[3]*roll_gyro_value; //
    cnt_u[1] = -Ky[0]*(pitch_tilt_value-pitch_set_point)-Ky[1]*pitch_gyro_value; //0.0; //
    cnt_u[2] =  0.0; //-Ky[4]*(yaw_tilt_value-yaw_set_point)-Ky[5]*yaw_gyro_value; // 0.0; //
    cnt_u[3] = kmass*mass*9.81;

    /* This is for plus configuration */
    Winv[0] = 0.0;
    Winv[1] = twolkF;
    Winv[2] = -fourkM;
    Winv[3] = fourkF;

    Winv[1*4+0] = -twolkF;
    Winv[1*4+1] = 0.0;
    Winv[1*4+2] = fourkM;
    Winv[1*4+3] = fourkF;

    Winv[2*4+0] = 0.0;
    Winv[2*4+1] = -twolkF;
    Winv[2*4+2] = -fourkM;
    Winv[2*4+3] = fourkF;

    Winv[3*4+0] = twolkF;
    Winv[3*4+1] = 0.0;
    Winv[3*4+2] = fourkM;
    Winv[3*4+3] = fourkF;

    int i;
    for (i=0; i<4; i++){
        omega_sqr[i] = Winv[i*4+0]*cnt_u[0] + Winv[i*4+1]*cnt_u[1] + Winv[i*4+2]*cnt_u[2] + Winv[i*4+3]*cnt_u[3];

        if (omega_sqr[i] > MAX_OMEGA) {
            omega_sqr[i] = MAX_OMEGA;
        }
        else if (omega_sqr[i] < MIN_OMEGA) {
            omega_sqr[i] = MIN_OMEGA;
        }

        omega[i] = sqrt(omega_sqr[i]);
        prop_cnt[i] = 9.8664e-03*omega[i]-1.1277;
    }

    /* Debugging/testing
    prop_u = 5*sin(.1*2*PI*timecnt*.004) + 5;
    setEPWM7A(prop_u);
    setEPWM7B(prop_u);
    setEPWM8A(prop_u);
    setEPWM8B(prop_u); */
    setEPWM8A(0.0); //prop_cnt[1]); // 0.0); //
    setEPWM8B(0.0); //prop_cnt[3]); //
    setEPWM7A(prop_cnt[0]); //
    setEPWM7B(prop_cnt[2]); //


    /*

    omega_sqr[0] = Winv[0]*cnt_u[0]     + Winv[1]*cnt_u[1]     + Winv[2]*cnt_u[2]     + Winv[3]*cnt_u[3];
    omega_sqr[1] = Winv[1*4+0]*cnt_u[0] + Winv[1*4+1]*cnt_u[1] + Winv[1*4+2]*cnt_u[2] + Winv[1*4+3]*cnt_u[3];
    omega_sqr[2] = Winv[2*4+0]*cnt_u[0] + Winv[2*4+1]*cnt_u[1] + Winv[2*4+2]*cnt_u[2] + Winv[2*4+3]*cnt_u[3];
    omega_sqr[3] = Winv[3*4+0]*cnt_u[0] + Winv[3*4+1]*cnt_u[1] + Winv[3*4+2]*cnt_u[2] + Winv[3*4+3]*cnt_u[3];

    omega[0] = sqrt(omega_sqr[0]);
    omega[1] = sqrt(omega_sqr[1]);
    omega[2] = sqrt(omega_sqr[2]);
    omega[3] = sqrt(omega_sqr[3]);


    for (int i=0; i<4; i++) {
        prop_u[i] = 9.866432210280869e-03*omega[i]-1.127716052136025;
    }*/


    /*--------------------Old Untouched Code for Cheapli control -----------------------------
    // This is 350s / (s+350) Tustin and 0.004
    thetaR = th_value + tilt_value;        //phiP+phiR

    vel_orig = 1.764705882352941e-01 * vel_orig + 2.058823529411765e+02 * (thetaR - thetaRold);
//  vel = 1.764705882352941e-01 * vel_orig + 2.058823529411765e+02 * (thetaR - thetaRold);


    /// FIX ME DERIVATIVE FILTER //////////////////////
//    vel = 1.764705882352941e-01 * vel + 2.058823529411765e+02 * (sign(vel_orig)*ADC0V);
    if(ADC0V<=-0.002)
    {
        ADC0V_scaled = 249.51*ADC0V-19.391;
    }
    else if(ADC0V>=0.002)
    {
        ADC0V_scaled = 246.67*ADC0V+20.06;
    }
    else if(ADC0V<0.002 && ADC0V>-0.002)
    {
        ADC0V_scaled = 8545.4*ADC0V-1.21;
    }
    else
    {
        ADC0V_scaled = 273.6*ADC0V-2.3303;
    }

    //vel = 1.764705882352941e-01 * vel + 4.118e-01 * (sign(vel_orig)*ADC0V_scaled+vel);
#ifdef USEONLYBEMF
    vel = 1.764705882352941e-01 * vel + 4.118e-01 * (compare(ADC1V)*fabs(ADC0V_scaled)+vel);
    vel2 = vel;
#else
    vel = vel_orig;
    vel2 = 1.764705882352941e-01 * vel2 + 2.058823529411765e+02 * (th_value - th_old);

    //    vel = -vel;
    //    vel2 = -vel2;

#endif

    //velf = 0.05*vel + 0.95*velfold;
    velf = 0.4*vel + 0.6*velfold;
    velfold = velf;

    SideAdjust_error = 0.2-velf;
    SideAdjust_I = SideAdjust_Iold + (SideAdjust_error+SideAdjust_errorOld)*0.002;  // 0.004/2

    accelxSideAdjust = (Kpcenter)*Kon*SideAdjust_error + Kicenter*Kon*SideAdjust_I;
    if (accelxSideAdjust > 0.08) {  // was limited to 0.05
        accelxSideAdjust = 0.08;
        SideAdjust_I = SideAdjust_Iold;  // stop integrating
    }
    if (accelxSideAdjust < -0.08) {
        accelxSideAdjust = -0.08;
        SideAdjust_I = SideAdjust_Iold;  // stop integrating
    }
    if (fabs(vel) > 450) {
        accelxSideAdjust = 0;
        SideAdjust_I = 0;
    }
    SideAdjust_Iold = SideAdjust_I;

    accelxSide = accelxSideStart + accelxSideAdjust;

    //      if(updateData == 0)
    //      {

    u = -K[0]*tilt_value-K[1]*gyro_value-K[2]*thetaR-K[3]*vel;

    //          updateData = 1;
    //      }

    if (vel2 > 0)
        u = u + (.8*(.0247 * vel2 + 1.1805));

    else
        u = u + (.8*(0.0233 * vel2 - 1.15046));



    if (fabs(tilt_value) > 0.2) {
        u = 0;
    }


    //saturation
    if (u >  18) u =  18;
    if (u < -18) u = -18;

    pwmVal = -u * (pwmCountMax / 40) + pwmCountMax / 2;

    //// PWM_setCmpA(myPwm, upwm);
    EPwm3Regs.CMPA.bit.CMPA = (int)pwmVal;

    th_old = th_value;
    thetaRold = thetaR;*/


    timecnt++;
}

void print200ms(void)
{

#ifdef PRINTDEBUG_NOSIMULINK
#ifdef USE_SCIC_OLD_VERSION
    serial_printf(&SerialC,"%.2f %.2f %.2f  %.4f    %.2f %.2f   %.4f %.4f %.4f\n\r",
#else
    serial_printf(&SerialA,"M:  %.4f %.4f %.4f %.4f %.4f %.4f %.1f  MA:  %.4f %.4f %.4f  Q:  %.4f %.4f %.4f %.4f  Qd:  %.4f %.4f %.4f %.4f  || %.2f\n\r",
#endif
                  accelx,
                  accely,
                  accelz,
                  gyrox,
                  gyroy,
                  gyroz,
                  compass_angle,
                  (mdg_roll*180)/PI,
                  (mdg_pitch*180)/PI,
                  (mdg_yaw*180)/PI,
                  Quaternions[0],
                  Quaternions[1],
                  Quaternions[2],
                  Quaternions[3],
                  DotQuaternions[0],
                  DotQuaternions[1],
                  DotQuaternions[2],
                  DotQuaternions[3],
                  mdg_beta);
#endif


}

/* The actual one
void print200ms(void)
{

#ifdef PRINTDEBUG_NOSIMULINK
#ifdef USE_SCIC_OLD_VERSION
    serial_printf(&SerialC,"%.2f %.2f %.2f  %.4f    %.2f %.2f   %.4f %.4f %.4f\n\r",
#else
    serial_printf(&SerialA,"M:  %.4f %.4f %.4f %.4f %.4f %.4f %.1f %.4f %.4f   ((G:  %.4f %.4f %.4f %.4f %.2f %.2f %.2f   %.1f %.1f %.1f ||Pi:  %.4f %.4f   ||Rl:  %.4f %.4f   ||Yw:  %.4f %.4f   ||CL:  %.4f %.4f    %.4f %.4f \n\r",
#endif
                  accelx,
                  accely,
                  accelz,
                  gyrox,
                  gyroy,
                  gyroz,
                  compass_angle,
                  (mpu_roll*180)/PI,
                  (mpu_pitch*180)/PI,
                  Ky[0],
                  Ky[1],
                  Ky[2],
                  Ky[3],
                  pitch_set_point,
                  roll_set_point,
                  yaw_set_point,
                  pitch_R,
                  roll_R,
                  yaw_R,
                  pitch_tilt_value,
                  pitch_gyro_value,
                  roll_tilt_value,
                  roll_gyro_value,
                  yaw_tilt_value,
                  yaw_gyro_value,
                  prop_cnt[0],
                  prop_cnt[2],
                  prop_cnt[1],
                  prop_cnt[3]);
#endif


} */

// tilt_value, gyro_value, accelxSide, vel, velf u, pred_P, kalman_K, K[3]

/*                  prop_cnt[0],
                  prop_cnt[2],
                  prop_cnt[1],
                  prop_cnt[3] */

void serialRXA(serial_t *s, char data)
{
#ifdef USE_SCIC_OLD_VERSION
    if (s == &SerialC ){
#else
    if (s == &SerialA ){
#endif
        // Pitch axis
        if(data == 'p')
            Ky[0]+= 0.001;
        else if(data == 'o')
            Ky[0]-= 0.001;

        else if(data == 'l')
            Ky[1]+=0.0001;
        else if(data == 'k')
            Ky[1]-=0.0001;

        else if(data == 'm')
            pitch_R*=10;
        else if(data == 'n')
            pitch_R*=0.1;

        else if(data == 'j')
            pitch_set_point+=0.01;
        else if(data == 'h')
            pitch_set_point-=0.01;

        // Roll axis
        else if(data == 'w')
            Ky[2]+= 0.001;
        else if(data == 'q')
            Ky[2]-= 0.001;

        else if(data == 's')
            Ky[3]+=0.0001;
        else if(data == 'a')
            Ky[3]-=0.0001;

        else if(data == 'x')
            roll_R*=10;
        else if(data == 'z')
            roll_R*=0.1;

        else if(data == 'f')
            roll_set_point+=0.01;
        else if(data == 'd')
            roll_set_point-=0.01;

        // Yaw axis
        else if(data == 'y')
            Ky[4]+= 0.001;
        else if(data == 't')
            Ky[4]-= 0.001;

        else if(data == 'g')
            Ky[5]+=0.0001;
        else if(data == 'b')
            Ky[5]-=0.0001;

        else if(data == 'i')
            yaw_set_point+=0.01;
        else if(data == 'u')
            yaw_set_point-=0.01;

        else if(data == 'v')
            yaw_R*=10;
        else if(data == 'c')
            yaw_R*=0.1;

        else if(data == 'r')
            mdg_beta += 0.05;
        else if(data == 'e')
            mdg_beta -= 0.05;
    }
}

/*********************/
//the code below is used to transmit data to MATLAB
void EchoSerialData(int memcount,char *buffer) {


    char sendmsg[256];
    int i;

    sendmsg[0] = 0x2A; // *
    sendmsg[2] = '0'; // 0
    for (i=3;i<(memcount*8+3);i++) {
        sendmsg[i] = buffer[i-3];
    }
    sendmsg[1] = i;
#ifdef USE_SCIC_OLD_VERSION
    serial_send(&SerialC, sendmsg, i);
#else
    serial_send(&SerialA, sendmsg, i);
#endif


}



void matlab_serialRX(serial_t *s, char data) {
    if (!UARTbeginnewdata) {// Only TRUE if have not yet begun a message
        if (42 == (unsigned char)data) {// Check for start char

            UARTdatacollect = 0;        // amount of data collected in message set to 0
            UARTbeginnewdata = 1;       // flag to indicate we are collecting a message
            Main_memcount = 0;
            Main_i = 0;
        }
    } else {    // Filling data
        if (0 == UARTdatacollect){
            UARTreceivelength = ((int)data)-1; // set receive length to value of char after start char
            UARTdatacollect++;
        }else if (UARTdatacollect < UARTreceivelength){
            UARTMessageArray[UARTdatacollect-1] = (char) data;
            // If sending out float value(s), save input memory locations and values at those addresses
            if (('0' == UARTMessageArray[0]) &&  (UARTdatacollect > 1)){

                if (Main_i == 0) {
                    ptrmemloc.c[1] = ((UARTMessageArray[UARTdatacollect-1] & 0xFF) << 8);
                }
                if (Main_i == 1) {
                    ptrmemloc.c[1] |= (UARTMessageArray[UARTdatacollect-1] & 0xFF);
                }
                if (Main_i == 2) {
                    ptrmemloc.c[0] = ((UARTMessageArray[UARTdatacollect-1] & 0xFF) << 8);
                }
                if (3 == Main_i){
                    ptrmemloc.c[0] |= (UARTMessageArray[UARTdatacollect-1] & 0xFF);

                    Main_address[Main_memcount]=ptrmemloc.i;
                    Main_value[Main_memcount]=*ptrmemloc.f;

                    Main_i = 0;
                    Main_memcount++;
                }else{
                    Main_i++;
                }
            }
            UARTdatacollect++;
        }
        if (UARTdatacollect == UARTreceivelength){  // If input receive length is reached
            UARTbeginnewdata = 0;   // Reset the flag
            UARTdatacollect = 0;    // Reset the number of chars collected

            // Case '0' : Sending data in endian format (big-endian address, big-endian value)
            if ('0' == UARTMessageArray[0]){
                for (Main_i = 0;Main_i<Main_memcount;Main_i++){
                    ptrmemloc.i=Main_address[Main_i];
                    Main_SendArray[0+8*Main_i]=((ptrmemloc.c[1]>>8)&0xFF);
                    Main_SendArray[1+8*Main_i]=ptrmemloc.c[1]&0xFF;
                    Main_SendArray[2+8*Main_i]=((ptrmemloc.c[0]>>8)&0xFF);
                    Main_SendArray[3+8*Main_i]=ptrmemloc.c[0]&0xFF;
                    memloc.f=Main_value[Main_i];
                    Main_SendArray[4+8*Main_i]=((memloc.c[1]>>8)&0xFF);
                    Main_SendArray[5+8*Main_i]=memloc.c[1]&0xFF;
                    Main_SendArray[6+8*Main_i]=((memloc.c[0]>>8)&0xFF);
                    Main_SendArray[7+8*Main_i]=memloc.c[0]&0xFF;
                }
                EchoSerialData(Main_memcount,Main_SendArray);   // Append header information to send data and transmit
                // Case '1' : Writing float value to memory address (big-endian received address / value)
            }else if ('1' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[5+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[6+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[7+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[8+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;


                }

                matlabLockshadow = matlabLock;
                // Case '2' : Sending array data in following format [char 1,char2,char3,...]
                // [*,3+input length of array,3 (code for array receiving in Matlab),...
                //      array(0) chars in little-endian, ... , array(memcount) chars in little-endian]
            }else if ('2' == UARTMessageArray[0]){
                Main_sendingarray = 1;
                matlabLock = 1.0;
                matlabLockshadow = matlabLock;
                memloc.c[1] = NULL;
                memloc.c[0] = ((UARTMessageArray[5]&0xFF)<<8);
                memloc.c[0] |= (UARTMessageArray[6]&0xFF);
                Main_memcount = memloc.i;
                ptrmemloc.c[1] = ((UARTMessageArray[1]&0xFF)<<8);
                ptrmemloc.c[1] |= (UARTMessageArray[2]&0xFF);
                ptrmemloc.c[0] = ((UARTMessageArray[3]&0xFF)<<8);
                ptrmemloc.c[0] |= (UARTMessageArray[4]&0xFF);
                Main_SendArray[0]='*';
                Main_SendArray[1]=3+Main_memcount;
                Main_SendArray[2]='3';

#ifdef USE_SCIC_OLD_VERSION
                serial_send(&SerialC, Main_SendArray, 3);
#else
                serial_send(&SerialA, Main_SendArray, 3);
#endif

                for (Main_i = 0; Main_i < Main_memcount;Main_i++){
                    Main_tempf = *ptrmemloc.f;
                    memloc.f = Main_tempf;
                    Main_SendArray2[0+Main_j*4] =  (memloc.c[0]&0xFF);
                    Main_SendArray2[1+Main_j*4] =  ((memloc.c[0]>>8)&0xFF);
                    Main_SendArray2[2+Main_j*4] =  (memloc.c[1]&0xFF);
                    Main_SendArray2[3+Main_j*4] =  ((memloc.c[1]>>8)&0xFF);
                    memloc.c[1] = ptrmemloc.c[1];
                    memloc.c[0] = ptrmemloc.c[0];
                    memloc.i+=2;  // was plus 4
                    ptrmemloc.c[1]=memloc.c[1];
                    ptrmemloc.c[0]=memloc.c[0];
                    Main_j++;
                    if (32 == Main_j){
                        memcpy(Main_SendArray,Main_SendArray2,128);
#ifdef USE_SCIC_OLD_VERSION
                        serial_send(&SerialC, Main_SendArray, 128);
#else
                        serial_send(&SerialA, Main_SendArray, 128);
#endif
                        Main_j = 0;
                    }
                }
                if (Main_j != 0){
#ifdef USE_SCIC_OLD_VERSION
                    serial_send(&SerialC, Main_SendArray2, (Main_memcount%32)*4);
#else
                    serial_send(&SerialA, Main_SendArray2, (Main_memcount%32)*4);
#endif
                    Main_j = 0;
                }
                Main_sendingarray = 0;
                // Case '3' : Write float value to memory address (big-endian received address,
                //      little-endian received value)
            }else if ('3' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[8+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[7+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[6+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[5+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;

                }

                matlabLockshadow = matlabLock;
            }
        }
    }
}


extern eqep_t eqep1;
extern eqep_t eqep3;
void simulink_serialRX(serial_t *s, char data) {
    //ANDREW changed output values, changed serial port to A from B
    //  if (savenumbytes < 400) {  // Just for Debug
    //      savebytes[savenumbytes] = data;
    //      savenumbytes++;
    //  }
    if (!SIMU_beginnewdata) {// Only true if have not yet begun a message
        if (SIMU_checkfirstcommandbyte == 1) {
            if (0xFF == (unsigned char)data) {// Check for start 2 bytes command = 32767 becuase assuming command will stay between -10000 and 10000
                SIMU_checkfirstcommandbyte = 0;
            }
        } else {
            SIMU_checkfirstcommandbyte = 1;
            if (0x7F == (unsigned char)data) {// Check for start char

                SIMU_datacollect = 0;       // amount of data collected in message set to 0
                SIMU_beginnewdata = 1;      // flag to indicate we are collecting a message

                SIMU_Tranaction_Type = 2;

                // Coould Start ADC and then ADC interrupt will read ENCs also and then send
                // but that is for Simulink control
                // For Simulink data collection just send most current ADC and ENCs
                // Simulink Sample rate needs to be at least 500HZ but 200Hz probably better

                SIMU_Var1_toSIMU_16bit = 1000*pitch_tilt_value;
                SIMU_Var2_toSIMU_16bit = 1000*pitch_gyro_value;
                SIMU_Var3_toSIMU_16bit = 1000*thetaR;
                SIMU_Var4_toSIMU_16bit = 1000*vel_orig;
                SIMU_Var5_toSIMU_16bit = 1000*u;
                SIMU_Var6_toSIMU_16bit = 1000*th_value;

                SIMU_TXrawbytes[3] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[2] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[0] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);

                SIMU_TXrawbytes[7] = (char)((SIMU_Var4_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[6] = (char)(SIMU_Var4_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[5] = (char)((SIMU_Var3_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[4] = (char)(SIMU_Var3_toSIMU_16bit & 0xFF);

                SIMU_TXrawbytes[11] = (char)((SIMU_Var6_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[10] = (char)(SIMU_Var6_toSIMU_16bit & 0xFF);
                SIMU_TXrawbytes[9] = (char)((SIMU_Var5_toSIMU_16bit >> 8) & 0xFF);
                SIMU_TXrawbytes[8] = (char)(SIMU_Var5_toSIMU_16bit & 0xFF);


//                SIMU_Var1_toSIMU_32bit = 10000*vel;
//                SIMU_Var2_toSIMU_32bit = 10000*vel_orig;
//                SIMU_Var1_toSIMU_16bit = 10000*tilt_value;
//                SIMU_Var2_toSIMU_16bit = 10000*gyro_value;
//                SIMU_TXrawbytes[3] = (char)((SIMU_Var1_toSIMU_32bit >> 24) & 0xFF);
//                SIMU_TXrawbytes[2] = (char)((SIMU_Var1_toSIMU_32bit >> 16) & 0xFF);
//                SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_32bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[0] = (char)((SIMU_Var1_toSIMU_32bit) & 0xFF);
//
//                SIMU_TXrawbytes[7] = (char)((SIMU_Var2_toSIMU_32bit >> 24) & 0xFF);
//                SIMU_TXrawbytes[6] = (char)((SIMU_Var2_toSIMU_32bit >> 16) & 0xFF);
//                SIMU_TXrawbytes[5] = (char)((SIMU_Var2_toSIMU_32bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[4] = (char)((SIMU_Var2_toSIMU_32bit) & 0xFF);
//
//                SIMU_TXrawbytes[8] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[9] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
//                SIMU_TXrawbytes[10] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
//                SIMU_TXrawbytes[11] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);

#ifdef USE_SCIC_OLD_VERSION
                serial_send(&SerialC,SIMU_TXrawbytes,12);
#else
                serial_send(&SerialA,SIMU_TXrawbytes,12);
#endif

            }
        }
    } else {    // Filling data
        if (SIMU_Tranaction_Type == 2) {
            if (SIMU_datacollect == 0){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 1){

                SIMU_Var1_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_datacollect++;
            } else if (SIMU_datacollect == 2){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 3){

                SIMU_Var2_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_datacollect++;
            } else if (SIMU_datacollect == 4){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 5){

                SIMU_Var3_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 6){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 7){

                SIMU_Var4_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;

            } else if (SIMU_datacollect == 8){
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 9){

                SIMU_Var5_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 10) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 11) {
                SIMU_Var6_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;
                SIMU_datacollect++;
            } else if (SIMU_datacollect == 12) {
                SIMU_databyte1 = data;
                SIMU_datacollect++;
            }else if (SIMU_datacollect == 13) {
                SIMU_Var7_fromSIMU_16bit = ((int)data)<<8 | SIMU_databyte1;

                SIMU_beginnewdata = 0;  // Reset the flag
                SIMU_datacollect = 0;   // Reset the number of chars collected
                SIMU_Tranaction_Type = 0;
            }

        }
    }
}

void initAngles (void)
{
    theta.first = true;
    theta.initial = 0;
    theta.d = 0;
    theta.d_old = 0;
    theta.f = 0.0;
    theta.f_old = 0.0;
    theta.l = 0;
    theta.cos = 0.0;
}

void setupSpia(void)        //for as5145h
{

    //chip select for rotary encoder = gpio73
    GPIO_SetupPinOptions(73, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(73, GPIO_MUX_CPU1, 0);
    GpioDataRegs.GPCSET.bit.GPIO73 = 1;   // Load output latch (set cs to initially high)


    //need clock polarity = 1 (falling edge)
    //collect data on falling edge -> phase = 1 (with delay)
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;     //put spi in reset (reset before changin configurations)

    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;    //falling edge clock (starts high)
    SpiaRegs.SPICCR.bit.SPICHAR = 7;        //shift length for one character is 8 bits
    SpiaRegs.SPICTL.all = 0x000E;           //0000000000001110->clkphase = 1 (with delay), master mode,
    //enable talk, spi int disabled

    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 49;  //set bit rate to 1Mhz so 50Mhz/50?
    SpiaRegs.SPIPRI.bit.FREE = 1;           //spi contunes even when emu halted
    SpiaRegs.SPISTS.all=0x0000;             //clear all flags
    SpiaRegs.SPIFFCT.all=0x00;              //set tx delay to no delays

    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;        //put tx fifo in reset and hold there
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;    //clear tx fifo interrupt flag (will not be using)
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   //put rx fifo in reset and hold there
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;      //spi fifo enhancements enable

    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    //clear receive fifo interrupt flag
    SpiaRegs.SPIFFRX.bit.RXFFIL = 3;        //interrupt level is one word
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;      //rx fifo interrupt enable
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 1;  //overrun interrupt enable
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;      //interrupt enable

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;          //release tx fifo to operate
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;     //release rx fifo to operate

    SpiaRegs.SPIFFTX.bit.SPIRST = 1;        //spi can resume tx or rx (pull out of reset)
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;     //pull spi out of reset (enable spi)
    //pie and cpu enables for group 6 done in setupSpib (6.1 and 6.2 for spia)
}

void setupSpib(void)    // for DWM1000      //for mpu9250
{

    //chip select for imu = gpio59
    GPIO_SetupPinOptions(59, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(59, GPIO_MUX_CPU1, 0);
    EALLOW;
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;   // Load output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;  // GPIO59 = GPIO59
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;   // GPIO59 = output
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset

    SpibRegs.SPICCR.all =0x000F;                 // Reset on, rising edge, 16-bit char bits
    SpibRegs.SPICTL.all =0x000E;                 // Enable master mode, normal phase,
    // enable talk, and SPI int disabled.
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE =0x0040;

    SpibRegs.SPIPRI.bit.FREE = 1;
    SpibRegs.SPIPRI.bit.SOFT = 0;

    SpibRegs.SPICCR.all =0x008F;                 // Relinquish SPI from Reset
    SpibRegs.SPICCR.bit.SPICHAR = 7;            // set to transmitt 8 bits

    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpibRegs.SPICTL.bit.TALK = 1;

    SpibRegs.SPICTL.bit.SPIINTENA = 0;

    SpibRegs.SPISTS.all=0x0000;

    SpibRegs.SPIFFTX.bit.SPIRST = 1;
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1;

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 2;
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;

    SpibRegs.SPIFFCT.all=0x00;

    SpibRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO=1;
    SpibRegs.SPIFFRX.bit.RXFIFORESET=1;

    SpibRegs.SPICTL.bit.SPIINTENA = 1;
    SpibRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;

    DELAY_US(200);

    // Step 5. User specific code:
    // Interrupts are not used in this example.
    senddata = ((0x8000)|(0x3B00)); //accelx high byte
    acc_config2 = 0x1D00;

    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = XG_OFFSET_H;
    SpibRegs.SPITXBUF = 0x0000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = XG_OFFSET_L;
    SpibRegs.SPITXBUF = 0x0000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = YG_OFFSET_H;
    SpibRegs.SPITXBUF = 0x0000;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = YG_OFFSET_L;
    SpibRegs.SPITXBUF = 0x0000;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = ZG_OFFSET_H;
    SpibRegs.SPITXBUF = 0x0000;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = ZG_OFFSET_L;
    SpibRegs.SPITXBUF = 0x0000;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = SMPLRT_DIV;
    SpibRegs.SPITXBUF = 0x1300;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = CONFIG;
    SpibRegs.SPITXBUF = 0x0200;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = GYRO_CONFIG;
    SpibRegs.SPITXBUF = 0x0000;                      //250 dps

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = ACCEL_CONFIG;
    SpibRegs.SPITXBUF = 0x0800;                      //+- 4g


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = ACCEL_CONFIG_2;
    SpibRegs.SPITXBUF = 0x0600;     // 5Hz


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = LP_ACCEL_ODR;
    SpibRegs.SPITXBUF = 0x0000;


    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = WOM_THR;
    SpibRegs.SPITXBUF = 0x0000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = FIFO_EN;
    SpibRegs.SPITXBUF = 0x0000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_MST_CTRL;
    SpibRegs.SPITXBUF = 0x4000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV0_ADDR;
    SpibRegs.SPITXBUF = 0x8C00;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV0_REG;
    SpibRegs.SPITXBUF = 0x0200;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    // check nack bit
    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV0_CTRL;
    SpibRegs.SPITXBUF = 0x8800;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV1_ADDR;
    SpibRegs.SPITXBUF = 0x0C00;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV1_REG;
    SpibRegs.SPITXBUF = 0x0A00;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV1_CTRL;
    SpibRegs.SPITXBUF = 0x8100;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = INT_ENABLE;
    SpibRegs.SPITXBUF = 0x0100;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = INT_STATUS;
    SpibRegs.SPITXBUF = 0x0100;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_SLV1_DO;
    SpibRegs.SPITXBUF = 0x0100;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = I2C_MST_DELAY_CTRL;
    SpibRegs.SPITXBUF = 0x0300;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = USER_CTRL;
    SpibRegs.SPITXBUF = 0x2000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = PWR_MGMT_1;
    SpibRegs.SPITXBUF = 0x0100;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = WHO_AM_I;
    SpibRegs.SPITXBUF = 0x7100;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = XA_OFFSET_H;
    SpibRegs.SPITXBUF = 0xEB00;                  //0xEE00

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = XA_OFFSET_L;
    SpibRegs.SPITXBUF = 0x1200;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = YA_OFFSET_H;
    SpibRegs.SPITXBUF = 0x1000;                  //0x1500

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = YA_OFFSET_L;
    SpibRegs.SPITXBUF = 0xFA00;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = ZA_OFFSET_H;
    SpibRegs.SPITXBUF = 0x2100;                  //0x2300

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    SpibRegs.SPITXBUF = ZA_OFFSET_L;
    SpibRegs.SPITXBUF = 0x5000;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=2) { }
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;   //Enable Pie Group 6, Int 1 (spia_rx)
    PieCtrlRegs.PIEIER6.bit.INTx2 = 1;   //enable pie gropu 6, int 2 (spia_tx)
    PieCtrlRegs.PIEIER6.bit.INTx3=1;     // Enable PIE Group 6, INT 3 (spib_rx)
    PieCtrlRegs.PIEIER6.bit.INTx4=1;     // Enable PIE Group 6, INT 4 (spib_tx)
    IER|= M_INT6;                         // Enable CPU INT6
}

