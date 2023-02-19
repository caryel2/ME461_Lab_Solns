//#############################################################################
// FILE:   LABstart_noteer_main.c
//
// TITLE:  Lab start_noteer
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <time.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"
//#include "temp.h"


#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// serial
extern float cx;
extern float cy;
extern float gx;
extern float gy;
extern float distance2marker;

//!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code to your global variable declarations
//*****************************************************************************
// the defines for FFT
//*****************************************************************************
#define RFFT_STAGES     10
#define RFFT_SIZE       (1 << RFFT_STAGES)

//*****************************************************************************
// the globals
//*****************************************************************************
#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(pwrSpec, "FFT_buffer_2")
#endif
float pwrSpec[(RFFT_SIZE/2)+1];
float maxpwr = 0;
int16_t maxpwrindex = 0;
float freq = 0;

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(test_output, "FFT_buffer_2")
#endif
float test_output[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(fft_input, "FFT_buffer_1")
#endif
float fft_input[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(RFFTF32Coef,"FFT_buffer_2")
#endif //__cplusplus
//! \brief Twiddle Factors
//!
float RFFTF32Coef[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(AdcPingBufRaw, "FFT_buffer_2")
#endif
uint16_t AdcPingBufRaw[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(AdcPongBufRaw, "FFT_buffer_2")
#endif
uint16_t AdcPongBufRaw[RFFT_SIZE];


//! \brief Object of the structure RFFT_F32_STRUCT
//!
RFFT_F32_STRUCT rfft;

//! \brief Handle to the RFFT_F32_STRUCT object
//!
RFFT_F32_STRUCT_Handle hnd_rfft = &rfft;
uint16_t pingFFT = 0;
uint16_t pongFFT = 0;
uint16_t pingpongFFT = 1;
uint16_t iPingPong = 0;
int16_t DMAcount = 0;
__interrupt void DMA_ISR(void);
void InitDma(void);
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

// SONG

#define C4NOTE ((uint16_t)(((50000000/2)/2)/261.63))
#define D4NOTE ((uint16_t)(((50000000/2)/2)/293.66))
#define E4NOTE ((uint16_t)(((50000000/2)/2)/329.63))
#define F4NOTE ((uint16_t)(((50000000/2)/2)/349.23))
#define G4NOTE ((uint16_t)(((50000000/2)/2)/392.00))
#define A4NOTE ((uint16_t)(((50000000/2)/2)/440.00))
#define B4NOTE ((uint16_t)(((50000000/2)/2)/493.88))
#define C5NOTE ((uint16_t)(((50000000/2)/2)/523.25))
#define D5NOTE ((uint16_t)(((50000000/2)/2)/587.33))
#define E5NOTE ((uint16_t)(((50000000/2)/2)/659.25))
#define F5NOTE ((uint16_t)(((50000000/2)/2)/698.46))
#define G5NOTE ((uint16_t)(((50000000/2)/2)/783.99))
#define A5NOTE ((uint16_t)(((50000000/2)/2)/880.00))
#define B5NOTE ((uint16_t)(((50000000/2)/2)/987.77))
#define F4SHARPNOTE ((uint16_t)(((50000000/2)/2)/369.99))
#define G4SHARPNOTE ((uint16_t)(((50000000/2)/2)/415.3))
#define A4FLATNOTE ((uint16_t)(((50000000/2)/2)/415.3))
#define C5SHARPNOTE ((uint16_t)(((50000000/2)/2)/554.37))
#define A5FLATNOTE ((uint16_t)(((50000000/2)/2)/830.61))
#define OFFNOTE 0
#define SONG_LENGTH 15
uint16_t songarray[SONG_LENGTH] = {
                                   OFFNOTE,
                                   E4NOTE,
                                   E4NOTE,
                                   E4NOTE,
                                   OFFNOTE,
                                   D4NOTE,
                                   D4NOTE,
                                   D4NOTE,
                                   OFFNOTE,
                                   C4NOTE,
                                   C4NOTE,
                                   C4NOTE,
                                   C4NOTE,
                                   C4NOTE,
                                   C4NOTE
};

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

//!!!!!!!!!!!!!!!!!!!!!!  start_note of Block
__interrupt void ADCB2_interrupt(void);
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

//lab7
__interrupt void ADCA_ISR (void); //adca1 pie interrupt

// function predefinition
void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void setEPWM8A_RCServo(float angle);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t ADCB2_count = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t PWM1 = 0; // PWM1 we want to send to DAN chip Ex3
uint16_t PWM2 = 0; // PWM2 we want to send to DAN chip Ex3
uint16_t updown = 1;
uint16_t increment = 10;
// variables for spi data
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t spivalue4 = 0;
int16_t spivalue5 = 0;
int16_t spivalue6 = 0;
int16_t spivalue7 = 0;
int16_t spivalue8 = 0;
//int16_t timer = 0;

// Variables for balancing sensors
float gyrox;
float gyroy;
float gyroz;
float accelx;
float accely;
float accelz;
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;

float ADC1; // Ex3 value of the ADC1 set with joy stick
float ADC2;// Ex3 value of the ADC2 set with joy stick
int16_t adca0result = 0.0;
int16_t adca1result = 0.0;
int32_t ADCA1_count = 0;
int32_t SPIB_count = 0;
float adcouty = 0.0;
float adcoutx = 0.0;

//Variables to calibrate sensors
float accelzBalancePoint = -.65;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};

// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//State variables
float LeftWheel = 0; // Position of the wheel
float RightWheel = 0;
float tilt_value = 0; // 1st state of the system
float gyro_value = 0; // 2nd state of teh system
float vel_Left = 0.0; // 3rd state of the system
float vel_Right= 0.0;
float gyrorate_dot = 0.0; // 4th state of the system

float ubal = 0.0; // Control effort
float u_left = 5.0; // Control effort for one wheel
float u_right = 5.0;

//Turn and Frwd - Bkwd implementation
float WhlDiff = 0.0; // Angle diff in the wheels (rads)
float vel_WhlDiff = 0.0; // Velocity diff in the wheels (rads/s)
float errorDiff = 0.0; // Diff between the reference turn and the wheel diff
float errorDiff_Itg = 0.0; // Integral of the error (PID control for turning)
float turnref = 0.0; // Angle we want to turn
float turn = 0.0; // Control effort for turn command
float FwdBackOffset = 0.0; // Control effort to go frwd or bkwd
float turn_rate = 0.0; // Speed that we want to turn at (we set it to the value we want)
float vref = 0.0; // Speed we want to advance (we set it to the value we want)
float FBerror = 0.0; // Diff between ref speed and speed of the robot
float FBerror_Itg = 0.0; // Integral of the error (PI control for frwd-bkwd)

//past values
float vel_Left_1= 0.0;
float vel_Right_1= 0.0;
float LeftWheel_1 = 0;
float RightWheel_1 = 0;
float gyrorate_dot_1 = 0.0;
float gyro_value_1 = 0.0;
float WhlDiff_1 = 0.0;
float vel_WhlDiff_1 = 0.0;
float turnref_1 = 0.0;
float errorDiff_1 = 0.0;
float errorDiff_Itg_1 = 0.0;
float turn_1 = 0.0;
float turn_rate_1 = 0.0;
float FBerror_1 = 0.0;
float FBerror_Itg_1 = 0.0;

//Gains for balancing control
float K1 = -85;
float K2 = -4.7;
float K3 = -1.2;
float K4 = -0.12;

float Kp = 3.0; // Gains for turn
float Ki = 20.0;
float Kd = 0.08;
float Kp_l = 0.35; // Gains for fwrd-bkwd
float Ki_l = 1.5;

// Variables for switch-case
int myStateVar = 10; // To keep track of States, we start_note in Wait  state
//int myStateVar = 40; // test state
int timersc = 0; // To maintain steps for a certain time
float startangle = 0.0;
int ball = 0; // To know that we are looking for the ball
float errorball = 0.0; //Deviation angle from centered position of the ball
float errorgoal = 0.0; //Deviation angle from centered position of the goal
float centerX = 160.0; // Center position of the camera (to be determined) this variable could be an int
float posX = 0.0; // Current X position of the ball in the camera
float KpX = 0.008; // Proportional gain to center the ball
float thres = 5.0; // Threshold angle where we consider the ball centered
float posY = 0.0; // Current Y position of the ball in the camera
float aproxY = 50.0; // Y position we want the ball to get to (to be determined) this could be an int
float servopos = 0.0; // Current angle of the servo
float servo = 0.0; // Angle we want the servo to have
float turn1 = 18.84; // Value to make the robot do a 360 turn (to be determined)
//int shoot = 0; // We use shoot_var
int shot = 0; // To know if we have shot
int beat = 0; // To know if the beat is playing
int start_note = 0; //To know if the start_note command has been sent
int start = 0; // Start variable, set to 1 when start_note is 1
int goal = 0; //
int wall = 0; // To know when we are close to a wall
int ball_found = 0; // To know if camera detected the ball
int goal_found = 0; // To know if camera detected the goal
//int beat_finished = 0; // To control when the beat has finished (not used in final version)
int dance = 0; // To know when the robot is in Dancing state
int playTone = 0; // To make the buzzer play the "close to wall" tone
int beatcount = 0; // To count the number of beats recorded by the microphone
int notgrab = 0; // To know when teh grab command has faild
//int shoot_var =0;


//variables for sound/ beat detection
int16_t adcb0result=0;
float x[100]= {0};
float yc[10] = {0};
float yc_1[10] = {0};
float ye[10] = {0};
float ye_1[10] = {0};
float yg[10] = {0};
float yg_1[10] = {0};
float yc_max=0;
float ye_max=0;
float yg_max=0;
int shoot_var =0; // To know if the shoot command is played
int shoot_count =0;

float VOL=0;
float VOL_1=0;
//int beat =0; //same function as the previous one
float aa= 1.5;
int bb = 10;
int cc = 45;
int dd = 700; // Min volume to start_note detecting beat
float rat = 0;
float b[101]={  2.0089131384901197e-03, 6.4032873499578040e-04, -1.7662310132503288e-03,    -1.8966231855838251e-03,    7.9038298787438197e-04, 2.8250866960543826e-03, 9.7274726769560108e-04, -2.8535932093218977e-03,    -3.2069079180517828e-03,    1.3777460739364028e-03, 5.0108857805228734e-03, 1.7369488778204004e-03, -5.0869489066624630e-03,    -5.6717260737981379e-03,    2.4066077632725297e-03, 8.6179538038498871e-03, 2.9352017836365030e-03, -8.4357135384937401e-03,    -9.2235281203421979e-03,    3.8369713729420702e-03, 1.3470983718227284e-02, 4.4992711557421761e-03, -1.2684979985041140e-02,    -1.3611937750688167e-02,    5.5600514925787251e-03, 1.9176967391055018e-02, 6.2956283333650978e-03, -1.7455271677881148e-02,    -1.8429536833842467e-02,    7.4103785848253561e-03, 2.5171457314971404e-02, 8.1418571044648731e-03, -2.2250769713411937e-02,    -2.3165078063428872e-02,    9.1879041586407240e-03, 3.0795414085640505e-02, 9.8318928762857697e-03, -2.6528873794684965e-02,    -2.7276081156801475e-02,    1.0686709091186523e-02, 3.5390668308456406e-02, 1.1166118673320274e-02, -2.9780034614308684e-02,    -3.0269173855075916e-02,    1.1725680290077527e-02, 3.8398491060813049e-02, 1.1981403290429368e-02, -3.1604759414221834e-02,    -3.1774940699058361e-02,    1.2176082500102338e-02, 3.9444917234878515e-02, 1.2176082500102338e-02, -3.1774940699058361e-02,    -3.1604759414221834e-02,    1.1981403290429368e-02, 3.8398491060813049e-02, 1.1725680290077527e-02, -3.0269173855075916e-02,    -2.9780034614308684e-02,    1.1166118673320274e-02, 3.5390668308456406e-02, 1.0686709091186523e-02, -2.7276081156801475e-02,    -2.6528873794684965e-02,    9.8318928762857697e-03, 3.0795414085640505e-02, 9.1879041586407240e-03, -2.3165078063428872e-02,    -2.2250769713411937e-02,    8.1418571044648731e-03, 2.5171457314971404e-02, 7.4103785848253561e-03, -1.8429536833842467e-02,    -1.7455271677881148e-02,    6.2956283333650978e-03, 1.9176967391055018e-02, 5.5600514925787251e-03, -1.3611937750688167e-02,    -1.2684979985041140e-02,    4.4992711557421761e-03, 1.3470983718227284e-02, 3.8369713729420702e-03, -9.2235281203421979e-03,    -8.4357135384937401e-03,    2.9352017836365030e-03, 8.6179538038498871e-03, 2.4066077632725297e-03, -5.6717260737981379e-03,    -5.0869489066624630e-03,    1.7369488778204004e-03, 5.0108857805228734e-03, 1.3777460739364028e-03, -3.2069079180517828e-03,    -2.8535932093218977e-03,    9.7274726769560108e-04, 2.8250866960543826e-03, 7.9038298787438197e-04, -1.8966231855838251e-03,    -1.7662310132503288e-03,    6.4032873499578040e-04, 2.0089131384901197e-03};
float b_c[101] = {0.00257903680154237,  0.00126648976456572,    -0.00288709057275183,   -0.000935684962911477,  0.00340718678899778,    0.000548454901212869,   -0.00413890271800365,   3.59229447378188e-06,   0.00504489786459957,    -0.000819851374378008,  -0.00605318659887130,   0.00198020516515100, 0.00706242434618525,    -0.00353594180296237,   -0.00794981940638888,   0.00550283944354319,    0.00858107969374591,    -0.00785699334929706,    -0.00882164448233542,   0.0105337560701143, 0.00854834750125001,    -0.0134299286605813,    -0.00766061726761164,   0.0164090966894166,  0.00609034653140265,    -0.0193097672404520,    -0.00380965339862845,   0.0219557497600751, 0.000835905590996865,   -0.0241680503002279, 0.00276642441229356,    0.0257774283450389, -0.00688727623226206,   -0.0266367070261990,    0.0113758476382000, 0.0266319356224508,  -0.0160484829650471,    -0.0256915774218564,    0.0206990926502279, 0.0237930310716143, -0.0251114042932280,    -0.0209659797477565, 0.0290722072569925, 0.0172922863799481, -0.0323846746513494,    -0.0129023985202990,    0.0348808344171230, 0.00796847541281254, -0.0364323161922181,    -0.0026946842804783,    0.0369586188130503, -0.00269468428047483,   -0.0364323161922181,    0.00796847541281254, 0.0348808344171230, -0.0129023985202990,    -0.0323846746513494,    0.0172922863799481, 0.0290722072569925, -0.0209659797477565, -0.0251114042932280,    0.0237930310716143, 0.0206990926502279, -0.0256915774218564,    -0.0160484829650471,    0.0266319356224508,  0.0113758476382000, -0.0266367070261990,    -0.00688727623226206,   0.0257774283450389, 0.00276642441229356,    -0.0241680503002279, 0.000835905590996865,   0.0219557497600751, -0.00380965339862845,   -0.0193097672404520,    0.00609034653140265,    0.0164090966894166,  -0.00766061726761164,   -0.0134299286605813,    0.00854834750125001,    0.0105337560701143, -0.00882164448233542,   -0.00785699334929706,    0.00858107969374591,    0.00550283944354319,    -0.00794981940638888,   -0.00353594180296237,   0.00706242434618525,    0.00198020516515100, -0.00605318659887130,   -0.000819851374378008,  0.00504489786459957,    3.59229447378188e-06,   -0.00413890271800365,   0.000548454901212869,    0.00340718678899778,    -0.000935684962911477,  -0.00288709057275183,   0.00126648976456572,    0.00257903680154237};
float b_e[101]  = {-0.00191361746056702,    0.00262710611053178,    -0.000534565923857995,  -0.00235633154688708,   0.00298091912476042,    -0.000277666527414101,  -0.00328581166870757,   0.00373963989077809,    0.000150102851843648,   -0.00476926761366541,   0.00477318722188382,    0.000922105566535264,   -0.00681366015052575,   0.00590963336808249,    0.00218506261674224,    -0.00936189695916625,   0.00695156952801502,    0.00404185821000390,    -0.0122953008629194,    0.00769546495250769,    0.00653821440166489,    -0.0154422477308313,    0.00795216836539800,    0.00965464000176925,    -0.0185922576604047,    0.00756655529764194,    0.0133045603769965, -0.0215142716308369,    0.00643437708167332,    0.0173389385346117, -0.0239774131110765,    0.00451461391630467,    0.0215570627788933, -0.0257722650673923,    0.00183605302745532,    0.0257225656823979, -0.0267306162457707,    -0.00150263671612561,   0.0295832146434350, -0.0267417560688791,    -0.00533943426282828,   0.0328926292529666, -0.0257637141747737,    -0.00946155680458819,   0.0354318747203304, -0.0238283183078759,    -0.0136229731434770,    0.0370288755288159, -0.0210395362709968,    -0.0175652154958816,    0.0375737909743657, -0.0175652154958816,    -0.0210395362709968,    0.0370288755288159, -0.0136229731434770,    -0.0238283183078759,    0.0354318747203304, -0.00946155680458819,   -0.0257637141747737,    0.0328926292529666, -0.00533943426282828,   -0.0267417560688791,    0.0295832146434350, -0.00150263671612561,   -0.0267306162457707,    0.0257225656823979, 0.00183605302745532,    -0.0257722650673923,    0.0215570627788933, 0.00451461391630467,    -0.0239774131110765,    0.0173389385346117, 0.00643437708167332,    -0.0215142716308369,    0.0133045603769965, 0.00756655529764194,    -0.0185922576604047,    0.00965464000176925,    0.00795216836539800,    -0.0154422477308313,    0.00653821440166489,    0.00769546495250769,    -0.0122953008629194,    0.00404185821000390,    0.00695156952801502,    -0.00936189695916625,   0.00218506261674224,    0.00590963336808249,    -0.00681366015052575,   0.000922105566535264,   0.00477318722188382,    -0.00476926761366541,   0.000150102851843648,   0.00373963989077809,    -0.00328581166870757,   -0.000277666527414101,  0.00298091912476042,    -0.00235633154688708,   -0.000534565923857995,  0.00262710611053178,    -0.00191361746056702};
float b_g[101]  = {-0.00191198889107609,  0.00273780901538804,    -0.00230601896034723,   0.000704235167311191,   0.00147333198009441,    -0.00326682459072658,   0.00369800998345206,    -0.00224755093884431,   -0.000746918459525739,  0.00403494644264700,    -0.00589496273946939,   0.00497212007886024,    -0.00113190129625273,   -0.00417119716373384,   0.00834915121554320,    -0.00888746547380188,   0.00476819008581432,    0.00266709236447939,    -0.0100176339755103,    0.0133306841935944, -0.0101748410826972,    0.00121070304443314,    0.00969414376204929,    -0.0170847034727610,    0.0166255092566400, -0.00756011583867844,   -0.00644443322630395,   0.0187172116881641, -0.0227596886621234,    0.0156673783707236, 8.88492978540208e-17,   -0.0170390697669405,    0.0269232481312074, -0.0240900682306537,    0.00902825354942126,    0.0115350212382207, -0.0276469410320881,    0.0309909842098276, -0.0192012151050058,    -0.00262146882050501,   0.0241144879480448, -0.0346304117503228,    0.0285685052910608, -0.00836340363598541,   -0.0164686636161933,    0.0338683908088186, -0.0351652477276787,    0.0194485192197776, 0.00585056895086330,    -0.0285199342522575,    0.0375418140871875, -0.0285199342522575,    0.00585056895086330,    0.0194485192197776, -0.0351652477276787,    0.0338683908088186, -0.0164686636161933,    -0.00836340363598541,   0.0285685052910608, -0.0346304117503228,    0.0241144879480448, -0.00262146882050501,   -0.0192012151050058,    0.0309909842098276, -0.0276469410320881,    0.0115350212382207, 0.00902825354942126,    -0.0240900682306537,    0.0269232481312074, -0.0170390697669405,    8.88492978540208e-17,   0.0156673783707236, -0.0227596886621234,    0.0187172116881641, -0.00644443322630395,   -0.00756011583867844,   0.0166255092566400, -0.0170847034727610,    0.00969414376204929,    0.00121070304443314,    -0.0101748410826972,    0.0133306841935944, -0.0100176339755103,    0.00266709236447939,    0.00476819008581432,    -0.00888746547380188,   0.00834915121554320,    -0.00417119716373384,   -0.00113190129625273,   0.00497212007886024,    -0.00589496273946939,   0.00403494644264700,    -0.000746918459525739,  -0.00224755093884431,   0.00369800998345206,    -0.00326682459072658,   0.00147333198009441,    0.000704235167311191,   -0.00230601896034723,   0.00273780901538804,    -0.00191198889107609};
float b_es[101] = {-0.00191746796823039,    4.32602248355084e-05,   0.00208380377409912,    0.00302198858293263,    0.00215230056088431,    -0.000278225237351552,  -0.00300458582569970,   -0.00428781974242604,   -0.00293479738878889,   0.000749807538565419,   0.00478279164615112,    0.00645986254128361,    0.00407867979347046,    -0.00162352549406134,   -0.00743344115545765,   -0.00938073460918133,   -0.00536170192823721,   0.00302336507949274,    0.0108819415826635, 0.0128173988264681, 0.00655137034593126,    -0.00501269627880183,   -0.0149657711640772,    -0.0164830088111665,    -0.00743171568114561,   0.00758178043002030,    0.0194457042260450, 0.0200645167460603, 0.00782803862058433,    -0.0106435694882098,    -0.0240256595421603,    -0.0232531249820921,    -0.00762676980309381,   0.0140385556407937, 0.0283792589501032, 0.0257743236425558, 0.00678813885164660,    -0.0175483788035227,    -0.0321803888793574,    -0.0274142857946070,    -0.00535017806767473,   0.0209168640735953, 0.0351345594412933, 0.0280397888604238, 0.00342359463234721,    -0.0238762647361147,    -0.0370077158722801,    -0.0276095515601103,    -0.00117811272111587,   0.0261758371532283, 0.0376493955155396, 0.0261758371532283, -0.00117811272111587,   -0.0276095515601103,    -0.0370077158722801,    -0.0238762647361147,    0.00342359463234721,    0.0280397888604238, 0.0351345594412933, 0.0209168640735953, -0.00535017806767473,   -0.0274142857946070,    -0.0321803888793574,    -0.0175483788035227,    0.00678813885164660,    0.0257743236425558, 0.0283792589501032, 0.0140385556407937, -0.00762676980309381,   -0.0232531249820921,    -0.0240256595421603,    -0.0106435694882098,    0.00782803862058433,    0.0200645167460603, 0.0194457042260450, 0.00758178043002030,    -0.00743171568114561,   -0.0164830088111665,    -0.0149657711640772,    -0.00501269627880183,   0.00655137034593126,    0.0128173988264681, 0.0108819415826635, 0.00302336507949274,    -0.00536170192823721,   -0.00938073460918133,   -0.00743344115545765,   -0.00162352549406134,   0.00407867979347046,    0.00645986254128361,    0.00478279164615112,    0.000749807538565419,   -0.00293479738878889,   -0.00428781974242604,   -0.00300458582569970,   -0.000278225237351552,  0.00215230056088431,    0.00302198858293263,    0.00208380377409912,    4.32602248355084e-05,   -0.00191746796823039};

void main(void){
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // EPWM 2A
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(2, GPIO_INPUT, GPIO_PULLUP);

    // EPW 2B
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(3, GPIO_INPUT, GPIO_PULLUP);


    // EPW 9
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(16, GPIO_INPUT, GPIO_PULLUP);

    //GPIO 52
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr; //ex2
    PieVectTable.ADCA1_INT = &ADCA_ISR; // lab7
    PieVectTable.DMA_CH1_INT = &DMA_ISR;
    PieVectTable.ADCB2_INT = &ADCB2_interrupt;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // 10ms period for Ex1. 20ms period for Ex3, 1ms for Ex4
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 250000); //set to 250ms for buzzer
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 100000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    //!!!!!!!!!!!!!!!!!!!!!! DMAFFT Copy this block of code after your init_serial functions

    // used for FFT
    EALLOW;
    EPwm7Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm7Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm7Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm7Regs.TBPRD = 5000; // Set Period to 0.1ms sample. Input clock is 50MHz.
    EPwm7Regs.ETSEL.bit.SOCAEN = 1; // Disable SOC on A group
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal EPwm7Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm7Regs.TBCTL.bit.CTRMODE = 0x00; //unfreeze, and enter up count mode
    EDIS;


    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; // Disable SOC on A group
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal EPwm7Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0x00; //unfreeze, and enter up count mode
    EDIS;

    // used for FIR filters
    EALLOW;
    EPwm3Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm3Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm3Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm3Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event
    EPwm3Regs.TBCTR = 0x0; // Clear counter
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm3Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm3Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm3Regs.TBPRD = 25000; // Set Period to 0.5ms sample. Input clock is 50MHz.
    EPwm3Regs.ETSEL.bit.SOCAEN = 1; // Disable SOC on A group
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal EPwm7Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm3Regs.TBCTL.bit.CTRMODE = 0x00; //unfreeze, and enter up count mode
    EDIS;

    // EPWM 8A
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(14, GPIO_INPUT, GPIO_PULLUP);


    // SETTINGS FOR EPWM8
    EALLOW;
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; // counter mode set to up-count mode
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 3; // free soft emulation mode set to free run
    EPwm8Regs.TBCTL.bit.PHSEN = 0; // phase loading disabled
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; // clock divided by 1
    EPwm8Regs.TBCTR= 0; // start_note the timer at 0
    EPwm8Regs.TBPRD = 62500; //frequency of PWM set to 20kHz
    EPwm8Regs.CMPA.bit.CMPA = 2500; //duty cycle set to 0%
    EPwm8Regs.CMPB.bit.CMPB = 5000; //duty cycle set to 0%
    EPwm8Regs.AQCTLA.bit.CAU = 1; //clears the pin when TBCTR = CMPA
    EPwm8Regs.AQCTLA.bit.ZRO = 2; // sets the pin when TBCTR = 0
    EPwm8Regs.AQCTLB.bit.CBU = 1; //clears the pin when TBCTR = CMPB
    EPwm8Regs.AQCTLB.bit.ZRO = 2; // sets the pin when TBCTR = 0
    EPwm8Regs.TBPHS.bit.TBPHS = 0; //set the phase to zero
    EDIS;

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1; //power up the ADCs
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; //delay for 1ms to allow ADC time to power up
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA balancing
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3; //SOC0 will convert Channel to be A3
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2; //SOC1 will convert Channel to be A2
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB  Microphone signal
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x11; // EPWM7 ADCSOCA or another trigger you choose will trigger SOC0
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0x9; // EPWM3 ADCSOCA or another trigger you choose will trigger SOC0
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag, originally = 1 (need to set to 1 for non-DMA)
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // Interrupt pulses regardless of flag state

    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1; //enable INT1 flag, originally = 1 (need to set to 1 for non-DMA)
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT1 flag is cleared

    EDIS;


    setupSpib(); //Ex4 initializations for spib
    InitDma();
    init_eQEPs(); //

    // SETTINGS FOR EPWM2
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // counter mode set to up-count mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 3; // free soft emulation mode set to free run
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // phase loading disabled
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // clock divided by 1
    EPwm2Regs.TBCTR= 0; // start_note the timer at 0
    EPwm2Regs.TBPRD = 2500; //frequency of PWM set to 20kHz
    EPwm2Regs.CMPA.bit.CMPA = 0; //duty cycle set to 0%
    EPwm2Regs.CMPB.bit.CMPB = 0; //duty cycle set to 0%
    EPwm2Regs.AQCTLA.bit.CAU = 1; //clears the pin when TBCTR = CMPA
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // sets the pin when TBCTR = 0
    EPwm2Regs.AQCTLB.bit.CBU = 1; //clears the pin when TBCTR = CMPB
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // sets the pin when TBCTR = 0
    EPwm2Regs.TBPHS.bit.TBPHS = 0; //set the phase to zero

    // SETTINGS FOR EPWM9
    EPwm9Regs.TBCTL.bit.CTRMODE = 0; // counter mode set to up-count mode
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 3; // free soft emulation mode set to free run
    EPwm9Regs.TBCTL.bit.PHSEN = 0; // phase loading disabled
    EPwm9Regs.TBCTL.bit.CLKDIV = 1; // clock divided by 2
    EPwm9Regs.TBCTR= 0; // start_note the timer at 0
    EPwm9Regs.TBPRD = 2500; //frequency of PWM set to 20kHz
    //    EPwm9Regs.CMPA.bit.CMPA = 0; //duty cycle set to 0%
    EPwm9Regs.AQCTLA.bit.CAU = 0; //do nothing
    EPwm9Regs.AQCTLA.bit.ZRO = 3; // toggles the pin when TBCTR = 0
    EPwm9Regs.TBPHS.bit.TBPHS = 0; //set the phase to zero

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    EDIS;

    // Dan moved here to start_note ADCA trigger last thing.
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode//
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;//ex2
    //    IER |= M_INT10; //lab7
    IER |= M_INT10;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //ex2
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //lab7
    //    PieCtrlRegs.PIEIER10.bit.INTx2 = 1;
    //    PieCtrlRegs.PIEIER10.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADCA1
    PieCtrlRegs.PIEIER10.bit.INTx6 = 1;

    int16_t i = 0;
    float samplePeriod = 0.0001;
    // Clear input buffers:
    for(i=0; i < RFFT_SIZE; i++){
        fft_input[i] = 0.0f;
    }
    for (i=0;i<RFFT_SIZE;i++) {
        fft_input[i] = sin(125*2*PI*i*samplePeriod)+2*sin(2400*2*PI*i*samplePeriod);
    }
    hnd_rfft->FFTSize   = RFFT_SIZE;
    hnd_rfft->FFTStages = RFFT_STAGES;
    hnd_rfft->InBuf     = &fft_input[0];  //Input buffer
    hnd_rfft->OutBuf    = &test_output[0];  //Output buffer
    hnd_rfft->MagBuf    = &pwrSpec[0];  //Magnitude buffer

    hnd_rfft->CosSinBuf = &RFFTF32Coef[0];  //Twiddle factor buffer
    RFFT_f32_sincostable(hnd_rfft);         //Calculate twiddle factor

    for (i=0; i < RFFT_SIZE; i++){
        test_output[i] = 0;               //Clean up output buffer
    }

    for (i=0; i <= RFFT_SIZE/2; i++){
        pwrSpec[i] = 0;                //Clean up magnitude buffer
    }


    int16_t tries = 0;
    while(tries < 10*0) {  // Get ride of the 0 in 10*0 if you want to run this while loop and test out the FFT function with these sin waves
        RFFT_f32(hnd_rfft);                     //Calculate real FFT

#ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project
        // properties
        RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
#else
        RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
#endif
        maxpwr = 0;
        maxpwrindex = 0;

        for (i=0;i<(RFFT_SIZE/2);i++) {
            //for (i=0;i<(45);i++) {
            if (pwrSpec[i]>maxpwr) {
                maxpwr = pwrSpec[i];
                maxpwrindex = i;
            }
        }

        tries++;
        for (i=0;i<RFFT_SIZE;i++) {
            fft_input[i] = sin((125 + tries*125)*2*PI*i*samplePeriod)+2*sin((2400-tries*200)*2*PI*i*samplePeriod);
        }
    }
    //!!!!!!!!!!!!!!!!!!!!!!  End of Block

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {

            //            serial_printf(&SerialA," tilt_value %.3f, gyro_value %.3f, Left Wheel %.3f, Right Wheel %.3f\r\n",
            //                          tilt_value,gyro_value,LeftWheel,RightWheel);//printing gyro and acc values
            //            serial_printf(&SerialA," Cx = %.3f Cy = %.3f xx = %.3f\r\n",cx,cy);//printing gyro and acc value
            //            serial_printf(&SerialA, "Power: %.3f Frequency: %f   %d  %f \r\n", maxpwr, VOL , beat, rat);
            serial_printf(&SerialA, "state %d start_note: %d ball: %d  \r\n", myStateVar,start_note,ball_found);


            UARTPrint = 0;
        }

        //        //!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code after your UARTPrint == 1 while loop as above
        if ( (pingFFT == 1) || (pongFFT == 1) ) {
            if (pingFFT == 1) {
                pingFFT = 0;
                // Raw ADC data
                for(i=0; i<RFFT_SIZE; i++) {
                    //--- Read the ADC results:
                    fft_input[i] = AdcPingBufRaw[i]*3.0/4095.0;  // ping data
                }
            } else if (pongFFT == 1) {
                pongFFT = 0;
                // Raw ADC data
                for(i=0; i<RFFT_SIZE; i++) {
                    //--- Read the ADC result:
                    fft_input[i] = AdcPongBufRaw[i]*3.0/4095.0;  // pong data
                }
                //                hnd_rfft->InBuf     = &fft_input[0];  //Input buffer
            }

            RFFT_f32(hnd_rfft);

#ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project
            // properties
            RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
#else
            RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
#endif
            maxpwr = 0;
            maxpwrindex = 0;
            for (i=5;i<(RFFT_SIZE/2);i++) {
                //for (i=5;i<(45);i++) {
                if (pwrSpec[i]>maxpwr) {
                    maxpwr = pwrSpec[i];
                    maxpwrindex = i;
                    freq = maxpwrindex*10000.0/1024.0;
                }
            }

            ///// BEAT detection
            VOL = 0; // reset volume
            for (i=bb;i<(cc);i++) {     // summing low frequencies 100 to 450 Hz
                VOL = pwrSpec[i] + VOL;
            }

            if ((VOL>(VOL_1*aa)) & (VOL> dd)){         // is VOL -> sum of low frequencies if new VOL is 1.5 times larger then previous one and larger 700 -  beat detected
                beat = 1;                              // beat detected
                GpioDataRegs.GPASET.bit.GPIO22 = 1;          // turns LEDs
                GpioDataRegs.GPESET.bit.GPIO130 = 1;
                GpioDataRegs.GPBSET.bit.GPIO60 = 1;
                GpioDataRegs.GPCSET.bit.GPIO94 = 1;
                GpioDataRegs.GPESET.bit.GPIO131 = 1;
                GpioDataRegs.GPBSET.bit.GPIO61 = 1;
                GpioDataRegs.GPCSET.bit.GPIO95 = 1;
                GpioDataRegs.GPASET.bit.GPIO25 = 1;
                GpioDataRegs.GPESET.bit.GPIO157 = 1;
                GpioDataRegs.GPDSET.bit.GPIO97 = 1;
                GpioDataRegs.GPASET.bit.GPIO26 = 1;
                GpioDataRegs.GPESET.bit.GPIO158 = 1;
                GpioDataRegs.GPDSET.bit.GPIO111 = 1;
                GpioDataRegs.GPASET.bit.GPIO27 = 1;
                GpioDataRegs.GPESET.bit.GPIO159 = 1;
            }
            else {
                beat = 0;
                GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;        // else turn it off
                GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
                GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
                GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
                GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
                GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
                GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
                GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
                GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
                GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
                GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
                GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
                GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;
            }
            rat = (VOL/VOL_1);
            VOL_1 = VOL;    // save previous values

            //            UARTPrint = 1;
        }
        //!!!!!!!!!!!!!!!!!!!!!!  End of Block
    }
}



// SWI_isr,  Using this interrupt as a Software start_noteed interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // Operate with averaged values from the sensors and calculate control

    // Calculate 3rd state
    vel_Left = 0.6*vel_Left_1 + (100*LeftWheel - 100*LeftWheel_1);
    vel_Right= 0.6*vel_Right_1 + (100*RightWheel - 100*RightWheel_1);

    // Calculate control efforts for turn and Fwrd Bkwrd
    WhlDiff = LeftWheel - RightWheel;
    vel_WhlDiff = 0.333*vel_WhlDiff_1 + 166.667*WhlDiff - 166.667*WhlDiff_1;

    errorDiff = turnref-WhlDiff;
    errorDiff_Itg = errorDiff_Itg_1 +(errorDiff + errorDiff_1)*0.004/2;

    FBerror = vref-(vel_Left + vel_Right)/2;
    FBerror_Itg = FBerror_Itg_1 + (FBerror + FBerror_1)*0.004/2;

    turn = Kp*errorDiff + Ki*errorDiff_Itg - Kd*vel_WhlDiff;
    FwdBackOffset = Kp_l*FBerror + Ki_l*FBerror_Itg;

    if(fabs(turn)>3){
        errorDiff_Itg = errorDiff_Itg_1;
    }
    if(fabs(FwdBackOffset)>3){
        FBerror_Itg = FBerror_Itg_1;
    }

    if(turn>=4){
        turn = 4;
    }else if(turn<=-4){
        turn = -4;
    }

    if(FwdBackOffset>=4.0){
        FwdBackOffset = 4.0;
    }else if(FwdBackOffset<=-4.0){
        FwdBackOffset = -4.0;
    }

    //Calculate 4th state
    gyrorate_dot = 0.6*gyrorate_dot_1 + (100*gyro_value - 100*gyro_value_1);

    turnref = turnref_1 + (turn_rate + turn_rate_1)*0.004/2;

    ubal = -K1*tilt_value - K2*gyro_value - K3*(vel_Left + vel_Right)/2.0 - K4*gyrorate_dot;

    u_right = ubal/2.0 - turn - FwdBackOffset;
    u_left = ubal/2.0 + turn - FwdBackOffset;

    //assign output
    setEPWM2B(-u_left);
    setEPWM2A(u_right);

    // save past states
    vel_Left_1= vel_Left;
    vel_Right_1= vel_Right;
    LeftWheel_1 = LeftWheel;
    RightWheel_1 = RightWheel;
    gyrorate_dot_1 = gyrorate_dot;
    gyro_value_1 = gyro_value;
    vel_WhlDiff_1 = vel_WhlDiff;
    WhlDiff_1 = WhlDiff;
    errorDiff_1 = errorDiff;
    errorDiff_Itg = errorDiff_Itg_1;
    turnref_1 = turnref;
    turn_rate_1 = turn_rate;
    FBerror_1 = FBerror;
    FBerror_Itg_1 = FBerror_Itg;


    numSWIcalls++;


    //    myStateVar = 140;
    switch (myStateVar){

    case 10: // Wait state
        // Keep balance waiting for command
        timersc ++; // Reset timer after any condition occurs
        if(start == 1 && shot == 0){ // If start_note command is given
            timersc = 0;
            start = 0;
            myStateVar = 20; // Go Search ball
            startangle = WhlDiff; // We record the angle of the wheels for Search ball
        } else if (shot == 1 && timersc == 2500){ // If we have just shot and 10 secs have pass we assume we didn't score
            timersc = 0;
            shot = 0;
            myStateVar = 20; // Go to Search ball
        } else if (shot == 1 && beat == 1){ // If we have just shot and we hear the beat we dance
            timersc = 0;
//            beat_finished = 10000; (not used in final version)
            shot = 0;
            myStateVar = 100; // Go to Dance state
        }
        break;

    case 20: // Search ball
        // Turn around its edge looking for the ball
        turn_rate = 1;
        ball = 1; //Variable to know that we are looking for the ball
        if ((turnref-startangle) >= turn1){ // turn1 will be a variable that will make the robot do 1 turn
            turn_rate = 0;
            myStateVar = 30; // Go to Search trajectory
        } else if(ball_found == 1){ // implement variable that turns 1 when ball detected
            turn_rate = 0;
            myStateVar = 40; // Go to Approximation trajectory
        }
        break;

    case 30: // Search trajectory
        // Advance a certain distance to locate closer to the ball
        vref = 3.5;
        timersc++; // Reset timer after any condition occurs
        if (timersc == 2000 && ball == 1){ // If the ball is not found while advancing, we Search ball again
            timersc = 0;
            vref = 0;
            startangle = WhlDiff;
            myStateVar = 120; // Go to turn 90 deg
        } else if(ball_found == 1 && ball == 1){ // If we find the ball we do Approximation trajectory
            timersc = 0;
            vref = 0;
            myStateVar = 40; // Go to Approximation trajectory
        } else if(timersc == 1500 && ball == 0){ // If the goal is not found while advancing, we Search goal again
            timersc = 0;
            vref = 0;
            startangle = WhlDiff;
            myStateVar = 120; // Go to turn 90 deg
        } else if(goal_found == 1 && ball == 0){ //If we find the goal (and we are not looking for ball) we go to Positioning trajectory
            timersc = 0;
            vref = 0;
            myStateVar = 80; // Go to Positioning trajectory
        } else if(wall == 1){
            timersc = 0;
            myStateVar = 110; // Go to WALL state
        }
        break;

    case 40: // Approximation trajectory
        // Get close to the golf ball
        posX = 320-cx; //Invert the direction to correct the sign of error
        posY = 240-cy; //Invert the direction to correct the sign of error
        errorball = centerX - posX; // Difference between center position X in the cam and current X position in the cam
        turn_rate = KpX*errorball; // Turn rate to make the robot keep the ball centered (might need negative sign)
        vref = 2;
        if (posY <= aproxY){ // When the Y position of the ball has a certain value stop advancing
            vref = 0;
            if(fabs(errorball) <= thres){ // When the ball is centered we proceed to grab it (adjust thres value)
                turn_rate = 0;
                myStateVar = 50; // Go to Grab command (change to 50 when testing grab)
            }
        } else if(wall == 1){
            myStateVar = 110; // Go to WALL state
        } else if(ball_found == 0){
            vref = 0;
            myStateVar = 20; // Go to Search ball
        }
        break;

    case 50: // Grab command
        // Grab the golf ball
        timersc++;
        vref = 2;
        if (timersc == 750){ // After 3 sec advancing, the speed is set to 1
            vref = 1;
            timersc = 0;
            myStateVar = 140; // Close servo
        } else if(wall == 1){
            timersc = 0;
            vref = 0;
            myStateVar = 110; // Go to WALL state
        }
        break;

    case 60: // Check grab
        //Check if the grab was done
        timersc++;
        vref = -2;
        if(timersc == 600){ // Go backwards for 2.4 sec
            vref = 0;
            timersc = 0;
            if(ball_found==1){ // If ball is found we need to repeat the grab
                myStateVar = 150; //Go to Open servo
                notgrab = 1; // Acknowledge that the grab has not happened
            } else { // If ball is not found that means it is grabbed
                startangle = WhlDiff;
                myStateVar = 70; //Go to Search goal
                ball = 0; // We are not longer looking for the ball
            }
        }
        break;

    case 70: // Search goal
        // Turn around its edge looking for the goal
        turn_rate = 1;
        if ((turnref-startangle) >= turn1){ // 1turn will be a variable that will make the robot do 1 turn
            turn_rate = 0;
            myStateVar = 30; // Go to Search trajectory
        } else if(goal_found ==1){ // implement variable that turns 1 when ball detected
            turn_rate = 0;
            myStateVar = 80; // Go to Positioning trajectory
        }
        break;

    case 80: // Positioning trajectory
        // Center the goal and prepare to shoot
        posX = 320- gx; //Invert the direction to correct the sign of error
        errorgoal = centerX - posX; // Difference between center position X in the cam and current X position in the cam
        turn_rate = KpX*errorgoal; // Turn rate to make the robot keep the goal centered (might need negative sign)
        if (distance2marker < 1.3){ // Back off until distance to the goal is 1.3
            vref =-2;
        }else{
            vref = 0;
            if (shoot_var == 1){
                if(fabs(errorgoal) <= thres){ // When the goal is centered we shoot
                    turn_rate = 0;
                    myStateVar = 90; // Go to Shoot trajectory
                }
            }
        }
        break;

    case 90: // Shoot trajectory
        // Advance towards teh goal to shoot
        posX = 320- gx; //Invert the direction to correct the sign of error
        errorgoal = centerX - posX; // Difference between center position X in the cam and current X position in the cam
        turn_rate = KpX*errorgoal;
        vref = 0.7*(timersc/200) + 3; // accelerate the robot to shoot
        timersc++;
        if(distance2marker < 0.7){ // Open the servo when the distance to the goal is 0.7
            timersc = 0;
            myStateVar = 150; //Go to Open servo
        }
        break;

    case 100: // Dance command
        // Dance when the robot scores
        timersc++;
        if(beat == 1){ // Count up the beats
            beatcount++;
//            beat_finished--;
        }

        if((beatcount%4) == 0){ // Make the robot turn
            turnref = turnref + turn1*0.18;
        } else if((beatcount+2)%4){
            turnref = turnref - turn1*0.18;
        }

        if((beatcount%8) == 0){ // Make the servo go up and down
            setEPWM8A_RCServo(20);
        } else if((beatcount+4)%8){
            setEPWM8A_RCServo(-60);
        }
        dance = 1;

//        if (beat_finished==0){
//            dance = 0;
//            beatcount = 0;
//            myStateVar = 10; // Go to wait command
//        } else if (wall == 1){
//            myStateVar = 110;
//        }
        if (timersc==10000){// After 40sec (duration of the song) go to Wait command
            dance = 0;
            beatcount = 0;
            myStateVar = 10; // Go to wait command
        } else if (wall == 1){
            myStateVar = 110; // Go to Wall
        }

        break;

    case 110: // WALL
        // Include a trajectory to get away from the wall
        vref = -3;
        timersc++;
        if(timersc == 500){ // Go backwards for 2 second
            vref = 0;
            timersc = 0;
            startangle = WhlDiff;
            myStateVar = 130; // Go to Turn 180 deg
        }
        break;

    case 120: // Turn 90 deg
        turn_rate = 3; // Turn 90 deg to look in another direction
        if((turnref - startangle) >= (0.25*turn1) && ball == 1){
            turn_rate = 0;
            startangle = WhlDiff;
            myStateVar = 20; // Go to Search ball
        } else if((turnref - startangle) >= (0.25*turn1) && ball == 0){
            turn_rate = 0;
            startangle = WhlDiff;
            myStateVar = 70; // Go to Search goal
        }
        break;

    case 130:  // Turn 180 deg after WALL
        turn_rate = 3; // Turn 180 deg to look in opposite direction
        if ((turnref - startangle) >= (0.5*turn1)){ // When the segbot is close to 180 turn we change state
            if (ball == 1 && dance == 0){
                turn_rate = 0;
                playTone = 1; // start_note playing the tone
                myStateVar = 160; // Go to Play tone
            } else if(ball == 0 && dance == 0) {
                turn_rate = 0;
                startangle = WhlDiff;
                myStateVar = 70; // Go to Search goal
            } else if (dance == 1){
                turn_rate = 0;
                myStateVar = 100; // Go to Dance command
            }
        }
        break;

    case 140: //Close servo
        // change position of servo
        if(servopos <= 45){
            servopos = servopos + 5;
            setEPWM8A_RCServo(servopos);
        }else if(servopos >= 45){
            vref = 0;
            myStateVar = 60; // Go to Check grab
        } else if(wall == 1){
            //Play the buzzer with some tones to ask for the ball relocation (probably we will have to do it in wall function)
            vref = 0;
            myStateVar = 110; // Go to WALL state
        }
        break;

    case 150: //Open servo
        // change position of servo
        if(servopos >= -89){
            servopos = servopos - 5;
            setEPWM8A_RCServo(servopos);
        }else if(servopos <= -90 && notgrab == 1){
            vref = 0;
            notgrab = 0;
            myStateVar=40; // Go to the Approximation trajectory
        } else if(servopos <= -90 && notgrab == 0){//after shooting, back off for 2sec and go to wait state
            shot = 1; // Variable to know that we have just shot
            vref = -3; // Go backwards
            timersc++;
            if (timersc == 2000){ // After 8 seconds go to wait state
                timersc=0;
                vref=0;
                myStateVar = 10; // Wait state
            }
        }else if(wall == 1){
            vref = 0;
            myStateVar = 110; // Go to WALL state
        }
        break;

    case 160: // Play tone when the ball is too close to a wall
        if(playTone == 0){ // When the tone finish
            ball = 0;
            myStateVar = 10; // Go to Wait state
        }
        break;
    }
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;

    //    if ((numTimer0calls%250) == 0) {
    //        displayLEDletter(LEDdisplaynum);
    //        LEDdisplaynum++;
    //        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
    //            LEDdisplaynum = 0;
    //        }
    //    }

    // Blink LaunchPad Red LED
    //    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    //    if ((CpuTimer0.InterruptCount % 200) == 0) {
    //                UARTPrint = 1;
    //            }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    if(CpuTimer1.InterruptCount < SONG_LENGTH){
        EPwm9Regs.TBPRD = songarray[CpuTimer1.InterruptCount]; //Set period to the value stored in array
    }
    else if(CpuTimer1.InterruptCount == SONG_LENGTH){
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
        CpuTimer1.InterruptCount = 0;
        playTone = 0; // Acknowledge that the tone has finished playing
    }
    if(playTone == 1){
        CpuTimer1.InterruptCount++;
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    } else {
        CpuTimer1.InterruptCount = 0;
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    CpuTimer2.InterruptCount++;
    //  if ((CpuTimer2.InterruptCount % 50) == 0) {
    //      UARTPrint = 1;
    //  }
}

__interrupt void SPIB_isr(void){
    //Ex4 read 8 values off RX FIFO, dicarded first, accx,accy,accz, temp, gyrox,gyroy,gyroz
    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. In Ex2 and 3 zero
    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. In Ex3 PWM1 value
    spivalue3 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. In Ex3 PWM1 vale
    spivalue4 = SpibRegs.SPIRXBUF;
    spivalue5 = SpibRegs.SPIRXBUF;
    spivalue6 = SpibRegs.SPIRXBUF;
    spivalue7 = SpibRegs.SPIRXBUF;
    spivalue8 = SpibRegs.SPIRXBUF;
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027

    //Ex4 value conversion (rad/s)
    gyrox = spivalue6*250.0/32767.0;
    gyroy = spivalue7*250.0/32767.0;
    gyroz = spivalue8*250.0/32767.0;

    //(g)
    accelx = spivalue2*4.0/32767.0;
    accely = spivalue3*4.0/32767.0;
    accelz = spivalue4*4.0/32767.0;

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start_note---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (/*tilt_array[0] + */tilt_array[1] + tilt_array[2] + tilt_array[3])/3.0; //Divide by 4
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }

    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        }else{
            GpioDataRegs.GPBSET.bit.GPIO34 = 1; // Turn on Red LED
        }


        UARTPrint = 1; // Tell While loop to print
    }





    //    SPIB_count++;
    //    if ((SPIB_count % 200) == 0) {
    //        UARTPrint = 1;
    //    }


    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

}


void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    //between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    //66 which are also a part of the SPIB setup.
    DINT;

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

    //    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    //    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //-----------------------------------------------------------------------------------------------------------------
    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    //sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    SpibRegs.SPITXBUF  = 0x1300;
    SpibRegs.SPITXBUF  = 0x0000;
    SpibRegs.SPITXBUF  = 0x0000;
    SpibRegs.SPITXBUF  = 0x0013;
    SpibRegs.SPITXBUF  = 0x0200;
    SpibRegs.SPITXBUF  = 0x0806;
    SpibRegs.SPITXBUF  = 0x0000;
    // To address 00x13 write 0x00
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7); //
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    //read the additional 7 receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF  = 0x2300;
    SpibRegs.SPITXBUF  = 0x408C;
    SpibRegs.SPITXBUF  = 0x0288;
    SpibRegs.SPITXBUF  = 0x0C0A;
    // To address 00x23 write 0x00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    //read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;temp = SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF  = 0x2A81;
    // Write to address 0x2A the value 0x81
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //Ex4 setting offset for acc x(0x77,0x78), y(0x7A,0X7B), z (0x7D,0x7E)
    SpibRegs.SPITXBUF = (0x7700 | 0x00E7); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0057); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0014); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00DD); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001B); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0022); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

//!!!!!!!!!!!!!!!!!!!!!!  Copy these two function to the end of your C file
interrupt void DMA_ISR(void)                    // PIE7.1 @ 0x000DA0  DMA channel 1 interrupt
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;         // Must acknowledge the PIE group

    //GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;

    //--- Process the ADC data
    if(iPingPong == 0)  // Ping buffer filling, process Pong bugger
    {
        // Manage the DMA registers
        EALLOW;                                                     // Enable EALLOW protected register access
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPongBufRaw;      // Adjust DST start_note address for Pong buffer
        EDIS;                                                       // Disable EALLOW protected register access

        // Don't run ping FFT first time around
        if (DMAcount > 0) {
            pingFFT = 1;
        }
        iPingPong = 1;
    }
    else    // Pong buffer filling, process Ping buffer
    {
        // Manage the DMA registers
        EALLOW;                                                     // Enable EALLOW protected register access
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPingBufRaw;                    // Adjust DST start_note address for Ping buffer
        EDIS;                                                       // Disable EALLOW protected register access

        pongFFT = 1;
        iPingPong = 0;
    }

    DMAcount += 1;

}

void InitDma(void)
{
    EALLOW;

    //---------------------------------------------------------------------
    //--- Overall DMA setup
    //---------------------------------------------------------------------
    DmaRegs.DMACTRL.bit.HARDRESET = 1;          // Reset entire DMA module
    asm(" NOP");                                // 1 cycle delay for HARDRESET to take effect

    DmaRegs.DEBUGCTRL.bit.FREE = 1;             // 1 = DMA unaffected by emulation halt
    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 0;  // Not using CH1 Priority mode

    //---------------------------------------------------------------------
    //--- Configure DMA channel 1 to read the ADC results
    //---------------------------------------------------------------------
    DmaRegs.CH1.MODE.all = 0x8901; //good
    // bit 15        1:      CHINTE, 0=interrupt disabled, 1=interrupt enabled
    // bit 14        0:      DATASIZE, 0=16-bit, 1=32-bit
    // bit 13-12     00:     reserved
    // bit 11        1:      CONTINUOUS, 0=stop, 1=re-init after transfer complete
    // bit 10        0:      ONESHOT, 0=one burst on trigger, 1=all bursts on trigger
    // bit 9         0:      CHINTMODE, 0=start_note of transfer, 1=end of transfer
    // bit 8         1:      PERINTE, peripheral interrupt trigger enable, 0=disabled, 1=enabled
    // bit 7         0:      OVRINTE, overflow interrupt enable, 0=disabled, 1=enabled
    // bit 6-5       00:     reserved
    // bit 4-0       00001:  Set to channel number

    //--- Select DMA interrupt source                 /******** TRIGGER SOURCE FOR EACH DMA CHANNEL (unlisted numbers are reserved) ********/
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = 6;    // 0=none       6=ADCBINT1  12=ADCCINT2  18=ADCDINT3  32=XINT4      40=EPWM3SOCA  46=EPWM6SOCA  52=EPWM9SOCA   58=EPWM12SOCA  72=MREVTA    98=SD1FLT4    110=SPIRXDMAA     132=USBA_EPx_TX1
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = 0;    // 1=ADCAINT1   7=ADCBINT2  13=ADCCINT3  19=ADCDINT4  33=XINT5      41=EPWM3SOCB  47=EPWM6SOCB  53=EPWM9SOCB   59=EPWM12SOCB  73=MXEVTB    99=SD2FLT1    111=SPITXDMAB     133=USBA_EPx_RX2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = 0;    // 2=ADCAINT2   8=ADCBINT3  14=ADCCINT4  20=ADCDEVT   36=EPWM1SOCA  42=EPWM4SOCA  48=EPWM7SOCA  54=EPWM10SOCA  68=TINT0       74=MREVTB   100=SD2FLT2    112=SPIRXDMAB     134=USBA_EPx_TX2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = 0;    // 3=ADCAINT3   9=ADCBINT4  15=ADCCEVT   29=XINT1     37=EPWM1SOCB  43=EPWM4SOCB  49=EPWM7SOCB  55=EPWM10SOCB  69=TINT1       95=SD1FLT1  101=SD2FLT3    113=SPITXDMAC     135=USBA_EPx_RX3
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = 0;    // 4=ADCAINT4  10=ADCBEVT   16=ADCDINT1  30=XINT2     38=EPWM2SOCA  44=EPWM5SOCA  50=EPWM8SOCA  56=EPWM11SOCA  70=TINT2       96=SD1FLT2  102=SD2FLT4    114=SPIRXDMAC     136=USBA_EPx_TX3
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH6 = 0;    // 5=ADCAEVT   11=ADCCINT1  17=ADCDINT2  31=XINT3     39=EPWM2SOCB  45=EPWM5SOCB  51=EPWM8SOCB  57=EPWM11SOCB  71=MXEVTA      97=SD1FLT3  109=SPITXDMAA  131=USBA_EPx_RX1

    //--- DMA trigger source lock
    DmaClaSrcSelRegs.DMACHSRCSELLOCK.bit.DMACHSRCSEL1 = 0;              // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.DMACHSRCSELLOCK.bit.DMACHSRCSEL2 = 0;              // Write a 1 to lock (cannot be cleared once set)


    DmaRegs.CH1.BURST_SIZE.bit.BURSTSIZE = 0;                           // 0 means 1 word per burst
    DmaRegs.CH1.TRANSFER_SIZE = RFFT_SIZE-1;                          // RFFT_SIZE bursts per transfer

    DmaRegs.CH1.SRC_TRANSFER_STEP = 0;                                  // 0 means add 0 to pointer each burst in a transfer
    DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32)&AdcbResultRegs.ADCRESULT0;   // SRC start_note address

    DmaRegs.CH1.DST_TRANSFER_STEP = 1;                                  // 1 = add 1 to pointer each burst in a transfer
    DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPingBufRaw;                    // DST start_note address Ping buffer


    DmaRegs.CH1.CONTROL.all = 0x0091; //good
    // bit 15        0:      reserved
    // bit 14        0:      OVRFLG, overflow flag, read-only
    // bit 13        0:      RUNSTS, run status, read-only
    // bit 12        0;      BURSTSTS, burst status, read-only
    // bit 11        0:      TRANSFERSTS, transfer status, read-only
    // bit 10-9      00:     reserved
    // bit 8         0:      PERINTFLG, read-only
    // bit 7         1:      ERRCLR, error clear, 0=no action, 1=clear SYNCERR bit
    // bit 6-5       00:     reserved
    // bit 4         1:      PERINTCLR, periph event clear, 0=no action, 1=clear periph event
    // bit 3         0:      PERINTFRC, periph event force, 0=no action, 1=force periph event
    // bit 2         0:      SOFTRESET, 0=no action, 1=soft reset the channel
    // bit 1         0:      HALT, 0=no action, 1=halt the channel
    // bit 0         1:      RUN, 0=no action, 1=enable the channel

    //--- Finish up
    EDIS;

    //--- Enable the DMA interrupt
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;  // Enable DINTCH1 in PIE group 7
    IER |= M_INT7;                      // Enable INT7 in IER to enable PIE group


} // end InitDma()
//!!!!!!!!!!!!!!!!!!!!!!  End of Block


float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw*(2*3.14/(400*30)));
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2*3.14/(400*30)));
}

void setEPWM2A(float controleffort){//convert controleffort (-10,10) to CMPA (0,TBPRD)
    float value = 0;
    // saturation
    if(controleffort > 10){
        controleffort = 10;
    }
    if(controleffort < -10){
        controleffort = -10;
    }
    // conversion
    value = controleffort *(EPwm2Regs.TBPRD/20.0)+(EPwm2Regs.TBPRD/2.0);
    // asign value to CMPA
    EPwm2Regs.CMPA.bit.CMPA = value;
}

void setEPWM2B(float controleffort){//convert controleffort (-10,10) to CMPB (0,TBPRD)
    float value = 0;
    // saturation
    if(controleffort > 10){
        controleffort = 10;
    }
    if(controleffort < -10){
        controleffort = -10;
    }
    // conversion
    value = controleffort *(EPwm2Regs.TBPRD/20.0)+(EPwm2Regs.TBPRD/2.0);
    // asign value to CMPA
    EPwm2Regs.CMPB.bit.CMPB = value;
}

__interrupt void ADCA_ISR (void){
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;

    //     Here covert ADCIND0 to volts
    adcouty = adca0result *3.0/4096;
    adcoutx = adca1result *3.0/4096;

    //lab7
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00)); // set the first bit to 1 to start_note reading, start_note reading from register 3A
    SpibRegs.SPITXBUF = 0; // write 7 more word to recieve 7 values we want
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;
    SpibRegs.SPITXBUF = 0;

    //    if ((ADCA1_count % 100) == 0) {
    //        UARTPrint = 1;
    //    }

    ADCA1_count++;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void setDACA(float dacouta0) {
    if (dacouta0 > 3.0) dacouta0 = 3.0;  //max Saturation
    if (dacouta0 < 0.0) dacouta0 = 0.0;  // min saturation
    DacaRegs.DACVALS.bit.DACVALS =(dacouta0* 4095)/3.0; // ????perform scaling of 0-3 to 0-4095
}
void setDACB(float dacouta1) {
    if (dacouta1 > 3.0) dacouta1 = 3.0;
    if (dacouta1 < 0.0) dacouta1 = 0.0;
    DacbRegs.DACVALS.bit.DACVALS = (dacouta1* 4095)/3.0; // ????perform scaling of 0-3 to 0-4095??
}

int16_t ADCvalue = 0;
// FIR filter recofnition
__interrupt void ADCB2_interrupt(void){
    ADCB2_count++;
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    ADCvalue = AdcbResultRegs.ADCRESULT1;
    x[0] =(ADCvalue/4095.0)*3.0;            // ADCIND0 voltage reading;

    yc[0]=0; // reset values
    ye[0]=0;
    yg[0]=0;
    int i=0;
    for (i=100; i>=0; i--){
        yc[0]=b_c[i]*x[i]+yc[0];            // filtering signal C5
        ye[0]=b_e[i]*x[i]+ye[0];            // filtering signal E5
        yg[0]=b_g[i]*x[i]+yg[0];            // filtering signal G5
    }
    yc_max =  yc[0];        // Set max to first value of array
    ye_max =  ye[0];
    yg_max =  yg[0];
    for (i=9; i>=0; i--){
        if (yc[i]> yc_max)  yc_max =yc[i]; // find max value of filtered signal
        if (ye[i]> ye_max)  ye_max =ye[i];
        if (yg[i]> yg_max)  yg_max =yg[i];
    }

    //Save past states before exiting from the function so that next sample they are the older state
    for (i = 100; i>=0; i--) {
        x[i] = x[i-1];

    }
    // push back filtered signals
    for (i = 9; i>=1; i--) {
        yc[i] = yc[i-1];
        ye[i] = ye[i-1];
        yg[i] = yg[i-1];

    }

    if ((yc_max > 0.3) & (shoot_var != 2)){          // if maximum is larger 0.3 shoot variable is 3
        shoot_var =3;
        GpioDataRegs.GPASET.bit.GPIO22 = 1;          // turns LEDs on for row 1
        GpioDataRegs.GPESET.bit.GPIO130 = 1;
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
        shoot_count = 6000;                          // start count to wait for another note
    } else {
        if  (shoot_var ==3){                        // if next note is not detected count down
            shoot_count--;
            if ( shoot_count <= 0){                 // reset shoot_count value to 0
                shoot_var = 0;
                GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;       // else turn LED off
                GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
            }
        }
    }
    if ((ye_max > 0.3) & (shoot_var ==3)){          // if second note played and ye_ma
        shoot_var =2;                               // value 2
        GpioDataRegs.GPCSET.bit.GPIO94 = 1;         // turns LEDs on for row 2
        GpioDataRegs.GPESET.bit.GPIO131 = 1;
        GpioDataRegs.GPBSET.bit.GPIO61 = 1;
        shoot_count = 6000;                         // start count to wait for another note
    } else {
        if  (shoot_var == 2){
            shoot_count--;
            if ( shoot_count <= 0){
                shoot_var = 0;                             //after 3 sec reset value
                GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;      // else turn LED off
                GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
            }
        }
    }
    if ((yg_max > 0.3) & (shoot_var ==2)){          // if second note played
        shoot_var =1;                               // value 2
        GpioDataRegs.GPCSET.bit.GPIO95 = 1;         // turns LEDs on for row 2
        GpioDataRegs.GPASET.bit.GPIO25 = 1;
        GpioDataRegs.GPESET.bit.GPIO157 = 1;
        shoot_count = 6000;                         // start count to reset
    } else {
        if  (shoot_var == 1){
            shoot_count--;
            if ( shoot_count <= 0){
                shoot_var = 0;                          //after 3 sec reset value
                GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;   // else turn LED off
                GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
                GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
            }
        }
    }



    if ((yg_max > 0.3)& (start_note !=2)){           // if maximum is larger 0.3 start_note variable is 3
        start_note =3;                               // value 3
        GpioDataRegs.GPASET.bit.GPIO22 = 1;          // turns LEDs on for row 1
        GpioDataRegs.GPESET.bit.GPIO130 = 1;
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
        shoot_count = 6000;                           // start count to wait for another note
    } else {
        if  (start_note ==3){
            shoot_count--;
            if ( shoot_count <= 0){
                start_note = 0;//1214 temp change            //after 3 sec reset value
                GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;        // else turn LED off
                GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
            }
        }
    }
    if ((ye_max > 0.3) & (start_note ==3)){     // if second note played
        start_note =2;                          // value 2
        GpioDataRegs.GPCSET.bit.GPIO94 = 1;     // turns LEDs on for row 1
        GpioDataRegs.GPESET.bit.GPIO131 = 1;
        GpioDataRegs.GPBSET.bit.GPIO61 = 1;
        shoot_count = 6000;                     // start count to wait for another note
    } else {
        if  (start_note == 2){
            shoot_count--;
            if ( shoot_count <= 0){
                start_note = 0;////1214 temp change     //after 3 sec reset value
                GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;   // else turn LED off
                GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
                GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
            }
        }
    }
    if ((yc_max > 0.3) & (start_note ==2)){     // if third note played
        start_note =1;                          // value 2
        GpioDataRegs.GPCSET.bit.GPIO95 = 1;     // turns LEDs on for row 1
        GpioDataRegs.GPASET.bit.GPIO25 = 1;
        GpioDataRegs.GPESET.bit.GPIO157 = 1;
        shoot_count = 6000;                     // start count to wait for another note
    } else {
        if  (start_note == 1){
            start = 1;
            shoot_count--;
            if ( shoot_count <= 0){
                start_note = 0;//1214 temp change      //after 3 sec reset value
                GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;  // else turn LED off
                GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
                GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
            }
        }
    }

    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
}

// SERVO MOTOR
void setEPWM8A_RCServo(float angle){// set angle of servo on EPWM8A
    float value = 0;
    // saturation
    if(angle > 90){
        angle = 90;
    }
    if(angle < -90){
        angle = -90;
    }
    // conversion
    value = angle *(5000/180.0)+5000.0;
    // asign value to CMPA
    EPwm8Regs.CMPA.bit.CMPA = value;
}
