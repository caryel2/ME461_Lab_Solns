//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

/*
What we learned from this lab:
In lab6, we learned how to the eQEP peripheral of the TMS320F28379D processor, and we learned how to use the DC motor’s optical encoder to sense the angle of the DC motor.
We also learned how to calculate velocity and angular rate, as well as how to track the car. The lab also help us understand PI control.
We implemented several functions for the robot car like going straight forward/backward, stop and turning around.
*/

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
// Exercise 2.3
__interrupt void SPIB_isr(void); // Add a predefinition.
// Exercise 4
void setupSpib(void); // Add a predefinition.

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//-----------Exercise 2.3-----------
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;

//-----------Exercise 3-----------
int16_t pwmvalue1 = 0;
int16_t pwmvalue2 = 0;
int16_t updown = 1;
int16_t uselessvalue = 0;
int16_t adcvalue1 = 0;
int16_t adcvalue2 = 0;
int16_t receivecount = 0;
float adcvolt1 = 0;
float adcvolt2 = 0;

//-----------Exercise 4-----------
int16_t useless= 0;
int16_t accelXraw = 0;
int16_t accelYraw = 0;
int16_t accelZraw = 0;
int16_t gyroXraw = 0;
int16_t gyroYraw = 0;
int16_t gyroZraw = 0;
int16_t Temperature = 0;
float accelXreading = 0;
float accelYreading = 0;
float accelZreading = 0;
float gyroXreading = 0;
float gyroYreading = 0;
float gyroZreading = 0;

//----------------Lab 6.1---------
void init_eQEPs(void); // Predefine eQEPs initialization function
float readEncLeft(void); // Predefine readEncLeft() to read left encoder
float readEncRight(void); // Predefine readEncRight() to read right encoder

//float leftwheel = 0;
//float rightwheel = 0;
void setEPWM2A(float controleffort); // Set controleffort for the right wheel
void setEPWM2B(float controleffort); // Set controleffort for the left wheel
float leftdistance = 0;
float rightdistance = 0;
float currentL = 0;
float currentR = 0;
float previousL = 0;
float previousR = 0;
float uleft = 5; // Control effort variables for left wheel
float uright = 5; // Control effort variables for right wheel
float currentLvRaw = 0;
float currentRvRaw = 0;
float Kp = 3;
float Ki = 25;
float Vref = 0;
float e_K_L = 0;
float e_K_1_L = 0;
float I_K_L = 0;
float I_K_1_L = 0;
float u_K_L = 0;
float e_K_R = 0;
float e_K_1_R = 0;
float I_K_R = 0;
float I_K_1_R = 0;
float u_K_R = 0;

// Lab 6 Exercise 4
float turn = 0;
float e_turn = 0;
float e_L = 0;
float e_R = 0;
float K_turn = 3;
//-----------------End of Lab 6.1------------
// -----------------------Exercise 6.5
float Wr = 0.56759; // feet
float Rwh = 0.19460;//feet
float thetaR = 0;
float thetaL = 0;
float thetaPos = 0;
float robotX = 0;
float robotY = 0;
float thetaAve = 0;
float thetaVelAve = 0;
float robotXvel = 0;
float robotYvel = 0;
float previousXvel = 0;
float previousYvel = 0;
float currentXvel = 0;
float currentYvel = 0;
float feet = 0;
float previousThetaL = 0;
float previousThetaR = 0;
float thetaVelL = 0;
float thetaVelR = 0;
//----------------------end of exercise 6.5

//--------------------------------Exercise 2.3-----------------------
//__interrupt void SPIB_isr(void) {
//    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16 bit value off RX FIFO. Probably is zero since no chip
//    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16 bit value off RX FIFO. Again probably zero
//    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027
//    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
//}
//--------------------------------end of exercise 2.3----------------------
////-------------------------------start of exercise 3-----------------
//__interrupt void SPIB_isr(void){
//    uselessvalue = SpibRegs.SPIRXBUF; // Read the first but useless value off RX FIFO. Probably is zero since no chip.
//    adcvalue1 = SpibRegs.SPIRXBUF; // Read the first useful value as the 16 bit adc1 value off RX FIFO. Again probably zero
//    adcvalue2 = SpibRegs.SPIRXBUF; // Read the second useful value as the 16 bit adc2 value off RX FIFO. Again probably zero
//    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027
//    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
//    adcvolt1 = adcvalue1*3.3/4095.0; // Convert adc1 volt value to be within a range of 0-3.3V.
//    adcvolt2 = adcvalue2*3.3/4095.0; // Convert adc2 volt value to be within a range of 0-3.3V.
//}
//// -----------------------end of exercise 3------------------------
//-------------------------------start of exercise 4-----------------
__interrupt void SPIB_isr(void) {
    useless = SpibRegs.SPIRXBUF; // Read the first but useless 16 bit value off RX FIFO. Probably is zero since no chip
    accelXraw = SpibRegs.SPIRXBUF; // Read the first useful value as the 16 bit accelerometer X axis value off RX FIFO. Again probably zero.
    accelYraw = SpibRegs.SPIRXBUF; // Read the second useful value as the 16 bit accelerometer Y axis value off RX FIFO. Again probably zero.
    accelZraw = SpibRegs.SPIRXBUF; // Read the third useful value as the 16 bit accelerometer Z axis value off RX FIFO. Again probably zero.
    Temperature = SpibRegs.SPIRXBUF;  // Read the forth useful value as the 16 bit gyro Y axis value off RX FIFO. Again probably zero.
    gyroXraw = SpibRegs.SPIRXBUF; // Read the fifth useful value as the 16 bit temperature value off RX FIFO. Again probably zero.
    gyroYraw = SpibRegs.SPIRXBUF; // Read the sixth useful value as the 16 bit gyro Y axis value off RX FIFO. Again probably zero.
    gyroZraw = SpibRegs.SPIRXBUF; // Read the seventh useful value as the 16 bit gyro Z axis value off RX FIFO. Again probably zero.

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027
    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.


    accelXreading = accelXraw*4.0/32767.0; // Convert accelerometer X axis value to be within a range of -4g~4g.
    accelYreading = accelYraw*4.0/32767.0; // Convert accelerometer Y axis value to be within a range of -4g~4g.
    accelZreading = accelZraw*4.0/32767.0; // Convert accelerometer Z axis value to be within a range of -4g~4g.
    gyroXreading = gyroXraw*250.0/32767.0; // Convert gyro X axis value to be within a range of -250~250.
    gyroYreading = gyroYraw*250.0/32767.0; // Convert gyro Y axis value to be within a range of -250~250.
    gyroZreading = gyroZraw*250.0/32767.0; // Convert gyro Z axis value to be within a range of -250~250.


    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}
// -----------------------end of exercise 4------------------------


void main(void)
{
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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    // Exercise 2.3
    PieVectTable.SPIB_RX_INT = &SPIB_isr; // Add SPIB_isr to the PieVectTable
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    // Exercise 2.2
    //ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000); // cpu_timer0_isr() is called every 10 ms.
    // Exercise 3
    //ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000); // cpu_timer0_isr() is called every 20 ms.
    // Exercise 4
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // cpu_timer0_isr() is called every 1 ms.
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000); // Set the CPU timer interrupts to timeout every 4 milliseconds.
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_serialSCID(&SerialD,115200);

    //-----------------lab 6.2 The following is for EPWM2A ----------------------------------------
    // The following code initialize GPIO2 to EPWM2A -------
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); // Set Mux Index to 1 so that EPWM2A is used instead of GPIO2.
    // The following code initialize GPIO2 to EPWM2B -------
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); // Set Mux Index to 1 so that EPWM2B is used instead of GPIO3.

    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up Mode: set CTRMODE (bit 1-0) to 00, which is up-count mode.
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // Free Soft emulation mode to Free Run: set FREE_SOFT (bit 15-14) to 1x (10 or 11 are both ok), which is free-run.
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // Disable the phase loading: set PHSEN (bit 2) to 0, which does not load the TBCTR from the TBPHS.
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Clock divide by 1: set CLKDIV (bit 12-10) to 000, so the clock has a period of 1/50000000 seconds.
    EPwm2Regs.TBCTR = 0; // Start the timer at zero.
    EPwm2Regs.TBPRD = 2500; // Keep the same TBPRD as EPWM12A. Set TBPRD to 2500 so that the period of the PWM signal is 20KHz.
    // Start the duty cycle at 0 %
    EPwm2Regs.CMPA.bit.CMPA = 0; // 0/2500 = 0%, initially start the duty cycle at 0%.
    EPwm2Regs.CMPB.bit.CMPB = 0; // 0/2500 = 0%, initially start the duty cycle at 0%.
    EPwm2Regs.AQCTLA.bit.CAU = 1; // Have the signal pin be cleared when the TBCTR register reaches the value in CMPA: set CAU (bit 5-4) to 01, which force EPWMxA output low (clear) when TBCTR = CMPA on Up Count.
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // Have the pin be set when the TBCTR register is zero: set ZRO (bit 1-0) to 10, which force EPWMxA output high (set) when TBCTR = 0.
    EPwm2Regs.AQCTLB.bit.CBU = 1; // Have the signal pin be cleared when the TBCTR register reaches the value in CMPB: set CAU (bit 5-4) to 01, which force EPWMxB output low (clear) when TBCTR = CMPB on Up Count.
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // Have the pin be set when the TBCTR register is zero: set ZRO (bit 1-0) to 10, which force EPWMxB output high (set) when TBCTR = 0.
    EPwm2Regs.TBPHS.bit.TBPHS = 0; // Set the phase to zero.
    // Lab 6: Exerise 1
    init_eQEPs(); // Call init_eQEPs() inside main() after the init_serialSCIA(),
//----------------------lab 6 -----------------------------
    setupSpib();
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);
    // -------------------------------------Start of Exercise 2.1 ------------------------------------
    //    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    //    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    //    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    //    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    //    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); // Set GPIO63 pin to SPISIMOB, GPIO63 Index 15 is SPISIMOB.
    //    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); // Set GPIO64 pin to SPISOMIB, GPIO63 Index 15 is SPISOMIB.
    //    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); // Set GPIO65 pin to SPICLKB, GPIO63 Index 15 is SPICLKB.
    //    EALLOW;
    //    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    //    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    //    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    //    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //    EDIS;
    //    // ------------------------------------
    //    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset, 0h (R/W) = Initializes the SPI operating flags to the reset condition.
    //    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    //    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    //    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master, 1h (R/W) = SPI is configured as a master.
    //    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF, the number of bits to be shifted is 16 and Fh (R/W) = 16-bit word.
    //    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission, 1h (R/W) = Enables transmission For the 4-pin option.
    //    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    //    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt, 0h (R/W) = Disables the interrupt.
    //    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x31; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    //    // 50MHZ. And this setting divides that base clock to create SCLK's period
    //    // For SPIBRR = 3 to 127: SPI Baud Rate = LSPCLK / (SPIBRR + 1), 50 MHz/1 MHz = 50, 50 - 1 = 49. 49 in Dec is 31 in Hex.
    //    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    //    SpibRegs.SPIFFTX.bit.SPIRST = 1; // Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive. 1h (R/W) = SPI FIFO can resume transmit or receive.
    //    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    //    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    //    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    //    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set, 1h (R/W) = Write 1 to clear SPIFFRX[RXFFINT] flag.
    //    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL. 1h (R/W) = RX FIFO interrupt based on RXFFIL match (greater than or equal to) will be enabled.
    //    SpibRegs.SPIFFCT.bit.TXDLY = 0x10; // Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip. 10h (R/W) = The next word in the TX FIFO buffer is transferred to
    // SPITXBUF 16 serial clock cycles after completion of transmission of the previous word.
    //    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset. 1h (R/W) = SPI is ready to transmit or receive the next character.
    //    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset. 1h (R/W) = Release transmit FIFO from reset.
    //    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    //    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don't think this is needed. Need to Test
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; // Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below
    // 10h (R/W) = A RX FIFO interrupt request is generated when there are 16 words in the RX buffer.
    // ----------------------------------------------End of Exercise 2.1 ------------------------

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    //   ---------------------Exersice 2.3 Initialize SPIB_ISR based on Pie Vector Table----------
    IER |= M_INT6; // SPIB_RX: INT6.3

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SPIB_ISR in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1; // SPIB_RX: INT6.3

    //   ---------------------------End of Initializing SPIB_ISR based on Pie Vector Table----------
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            // Print out the position measurements
            serial_printf(&SerialA,"position X = %.3f position Y =  %.3f Robot Angle = %.3f thetaL %.3f thetaR %.3f \r\n", robotX,robotY, thetaPos, thetaL, thetaR);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
    // Insert SWI ISR Code here.......

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;
    // ---------------------------- Start of Exercise 2.2----------------------------------------
    // Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //    GpioDataRegs.GPACLEAR.bit.GPIO09 = 1; // Clear GPIO9 Low.
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    // ---------------------- End of Exercise 2.2-------------------------------------------------

    // ---------------------- Start of Exercise 3-------------------------------------------------
    //
    //    if (updown == 1) { // If updown == 1, it should count up.
    //        pwmvalue1 = pwmvalue1+10; // pwmvalue1 should increase by 10.
    //        pwmvalue2 = pwmvalue2+10; // pwmvalue2 should increase by 10.
    //        if (pwmvalue1 >= 3000){ // When pwmvalue reached the max value, start decreasing pwmvalue1 and pwmvalue2 by 10.
    //            updown = 0; // Start to decrease pwmvalue1 and pwmvalue2 when the interrupt function is called the next time.
    //        }
    //    }
    //    if (updown == 0) { // If updown == 0, it should count down.
    //        pwmvalue1 = pwmvalue1-10; // pwmvalue1 should decrease by 10.
    //        pwmvalue2 = pwmvalue2-10; // pwmvalue2 should decrease by 10.
    //        if (pwmvalue1 <= 0) { // When pwmvalue reached the min value, start increasing pwmvalue1 and pwmvalue2 by 10.
    //            updown = 1; // Start to increase pwmvalue1 and pwmvalue2 when the interrupt function is called the next time.
    //        }
    //    }
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when three values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x00DA; // 0x00DA must be sent as the first 16 bit value.
    //    SpibRegs.SPITXBUF = pwmvalue1; // Sends the 16 bit PWM1 command value between 0 and 3000.
    //    SpibRegs.SPITXBUF = pwmvalue2; // Sends the 16 bit PWM2 command value between 0 and 3000.
    //
    //    receivecount++;
    //    if ((receivecount %5) ==0) { // 5 * 20 ms = 100 ms.
    //        UARTPrint = 1; // Set the print flag to 1 and print the value in units of volts to Tera Term every 100 milliseconds.
    //    }
    // -----------------------------------End of exercise 3----------------------------------------

    //----------------------------start of Exercise 4 --------------------
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when three values are in the RX FIFO
    SpibRegs.SPITXBUF= 0xBA00;
    SpibRegs.SPITXBUF= 0x0000;
    SpibRegs.SPITXBUF= 0x0000;
    SpibRegs.SPITXBUF= 0x0000;
    SpibRegs.SPITXBUF= 0x0000;
    SpibRegs.SPITXBUF= 0x0000;
    SpibRegs.SPITXBUF= 0x0000;
    SpibRegs.SPITXBUF= 0x0000;
    //--------------------------------------End of exercise 4

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }
    if ((CpuTimer0.InterruptCount %200) ==0){
        UARTPrint = 1;
    }
    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
    // ------------------------------ Lab 6: Exerise 2 ------------------------------
    // Pass the uLeft and uRight global float variables
    // setEPWM2A(uright);
    // setEPWM2B(-uleft); // nagetive to spin in the positive direction
    // Call the read functions
    // thetaL = readEncLeft(); // units: radius
    // thetaR = readEncRight(); // units: radius
    // leftdistance = thetaL*5; // units: feet, 5 radius/feet
    // rightdistance = thetaR*5; // units: feet, 5 radius/feet
    // currentL = leftdistance; // units: feet
    // currentR = rightdistance; // units: feet
    // currentLvRaw = (currentL-previousL)/0.004; // Calculate the current velocity of left wheel, units: feet/s.
    // currentRvRaw = (currentR-previousR)/0.004; // Calculate the current velocity of right wheel, units: feet/s.
    // ------------------------------ End of Lab 6: Exerise 2 ------------------------------
    // ------------------------------ Lab 6: Exerise 3/5 ------------------------------
    // Call the read functions
    thetaL = readEncLeft(); // units: radius
    thetaR = readEncRight(); // units: radius
    leftdistance = thetaL*Rwh; // units: feet
    rightdistance = thetaR*Rwh; // units: feet
    currentL = leftdistance; // units: feet
    currentR = rightdistance; // units: feet
    currentLvRaw = (currentL-previousL)/0.004; // Calculate the current velocity of left wheel, units: feet/s.
    currentRvRaw = (currentR-previousR)/0.004; // Calculate the current velocity of right wheel, units: feet/s.

    // ----------------------- 6.5-----------------------------------
    thetaVelL = (thetaL-previousThetaL)/0.004; // Calculate the current angular rate of left wheel, units: radius/s.
    thetaVelR =  (thetaR-previousThetaR)/0.004; // Calculate the current angular rate of right wheel, units: radius/s.
    thetaPos = Rwh/Wr*(thetaR-thetaL); // Calculate the robot car angle

    thetaAve = 0.5*(thetaR+thetaL); // Calculate the average rotation angle
    thetaVelAve = 0.5*(thetaVelL + thetaVelR); // Calculate the average rotation angle rate

    currentXvel = Rwh*thetaVelAve*cos(thetaPos); // Calculate the current velocity in X coordinate
    currentYvel = Rwh*thetaVelAve*sin(thetaPos); // Calculate the current velocity in Y coordinate
    // integrate with previous value and current value
    robotX = robotX + 0.004*0.5*(currentXvel + previousXvel);  // Calculate the current position in X coordinate
    robotY = robotY + 0.004*0.5*(currentYvel + previousYvel);  // Calculate the current position in Y coordinate

    previousXvel = currentXvel; // Store current velocity in X coordinate as the previous velocity in X coordinate in next 4 ms
    previousYvel = currentYvel; // Store current velocity in Y coordinate as the previous velocity in Y coordinate in next 4 ms
    previousL = leftdistance; // Store current distance of left wheel as the previous distance of left wheel in next 4 ms
    previousR = rightdistance; // Store current distance of right wheel as the previous distance of right wheel in next 4 ms
    previousThetaL = thetaL; // Store current rotation angle of left wheel as the previous rotation angle of left wheel in next 4 ms
    previousThetaR = thetaR; // Store current rotation angle of right wheel as the previous rotation angle of right wheel in next 4 ms
    // -------------------------- end of 6.5------------------------
    // Exercise 6.4
    e_turn = turn + (currentLvRaw - currentRvRaw);
    // ------------------------------ Lab 6: Exerise 3 ------------------------------
    // For left wheel
    e_K_1_L = e_K_L; // The previous value is the "current" value from last 4ms
    I_K_1_L = I_K_L; // The previous value is the "current" value from last 4ms
    //e_K_L = Vref - currentLvRaw; // Exersice 6.3
    // Exercise 6.4
    e_K_L = Vref - currentLvRaw - K_turn * e_turn;
    if ((u_K_L >= 10) || (u_K_L <= -10)) { // Saturated at 10 or -10
        I_K_L = I_K_1_L; // Set Ik equal to the previous Ik 4 ms ago
    }
    else {
        I_K_L = I_K_1_L + 0.004 * (e_K_L + e_K_1_L) / 2;
    }
    u_K_L = Kp * e_K_L + Ki * I_K_L;
    // For right wheel
    e_K_1_R = e_K_R; // The previous value is the "current" value from last 4ms
    I_K_1_R = I_K_R; // The previous value is the "current" value from last 4ms
    //e_K_R = Vref - currentRvRaw; // Exersice 6.3
    // Exercise 6.4
    e_K_R = Vref - currentRvRaw + K_turn * e_turn;
    if ((u_K_R >= 10) || (u_K_R <= -10)) { // Saturated at 10 or -10
        I_K_R = I_K_1_R; // Set Ik equal to the previous Ik 4 ms ago
    }
    else {
        I_K_R = I_K_1_R + 0.004 * (e_K_R + e_K_1_R) / 2;
    }
    u_K_R = Kp * e_K_R + Ki * I_K_R;
    setEPWM2A(u_K_R); // Set the control effort of the right wheel
    setEPWM2B(-u_K_L); // Set the control effort of the left wheel
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    CpuTimer2.InterruptCount++;
    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}

// ------------------------------Start of Exercise 4 ------------------------------------------------------------
void setupSpib(void) // Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //-------------------------------------------------Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    // between each transfer to 0. Also don't forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    // 66 which are also a part of the SPIB setup.
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF, the number of bits to be shifted is 16 and Fh (R/W) = 16-bit word.
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00; // The TXdelay between each transfer is 0.
    // The GPIO settings for GPIO9, 63, 64, 65 and 66.
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

    // ------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    //    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x31; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK's period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    //SpibRegs.SPIFFCT.bit.TXDLY = 0x0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don't think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more
    //-----------------------------------------------------------------------------------------------------------------
    //----------------------------------------------------Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    // sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.

    SpibRegs.SPITXBUF = (0x1300 | 0x0000);// To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);// To address 00x14 write 0x00; To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);// To address 00x16 write 0x00; To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);// To address 00x18 write 0x00; To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);// To address 00x1A write 0x02; To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);// To address 00x1B write 0x08; To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);// To address 00x1E write 0x00; To address 00x1F write 0x00

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7); // Wait until 7 16 bit values are written.
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // Read 7 garbage receive values off the RX FIFO.
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // temp read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF= 0x2300;// To address 00x23 write 0x00
    SpibRegs.SPITXBUF= 0x408C;// To address 00x24 write 0x40; To address 00x25 write 0x8C
    SpibRegs.SPITXBUF= 0x0288;// To address 00x26 write 0x02; To address 00x27 write 0x88
    SpibRegs.SPITXBUF= 0x0C0A;// To address 00x28 write 0x0C; To address 00x29 write 0x0A
    // wait for the correct number of 16 bit values to be received into the RX FIFO

    while(SpibRegs.SPIFFRX.bit.RXFFST !=4); // Wait until 4 16 bit values are written.
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // Read 4 garbage receive values off the RX FIFO.
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // temp read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //---------------------------------Step 4.---------------------------------
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF= 0x2A81; // Write to address 0x2A the value 0x81

    while(SpibRegs.SPIFFRX.bit.RXFFST !=1); // wait for one byte to be received
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
    SpibRegs.SPITXBUF = (0x7700 | 0x0019); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x004C); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0017); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00BA); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001E); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x000E); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
// ------------------------------End of Exercise 4 function ------------------------------------------------------------
// ------------------------------ Lab 6: Exerise 1 ------------------------------
// in left and right read function, the rad/count = 2*pi/400/30, since gear ratio is 30:1, and quodrature(400 count/revolution)
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
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(-0.000523599)); // 2*pi/(30*400)=0.000523599, use negtivie to inverse rotation
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.

    return (raw*(0.000523599)); // 2*pi/(30*400)=0.000523599
}
// ------------------------------ End of Lab6 exercise 1------------------
// ------------------------------ Lab6 exercise 2------------------
// Copied from Lab3
void setEPWM2A(float controleffort){ // Set control effort for right wheel
    // Satuate to 10 if controleffort is greater than 10.
    if (controleffort >= 10) {
        controleffort = 10;
    }
    // Satuate to -10 if controleffort is less than -10.
    if (controleffort <= -10) {
        controleffort = -10;
    }
    // Linear scaling: -10a + b = 0, b = 50, 10a + b = 100
    // Thus, a = 5, b = 50
    float dutycycle = controleffort*5.0+50.0;
    float countnumber = EPwm2Regs.TBPRD*dutycycle/100.0; // Divide 100 here to avoid truncation. count number = period * duty cycle
    EPwm2Regs.CMPA.bit.CMPA = countnumber; // Assign countnumber to EPWM2's CMPA
}

// Copied from Lab3
void setEPWM2B(float controleffort){ // Set control effort for left wheel
    // Satuate to 10 if controleffort is greater than 10.
    if (controleffort >= 10) {
        controleffort = 10;
    }
    // Satuate to -10 if controleffort is less than -10.
    if (controleffort <= -10) {
        controleffort = -10;
    }
    // Linear scaling: -10a + b = 0, b = 50, 10a + b = 100
    // Thus, a = 5, b = 50
    float dutycycle = controleffort*5.0+50.0;
    float countnumber = EPwm2Regs.TBPRD*dutycycle/100.0; // Divide 100 here to avoid truncation. count number = period * duty cycle
    EPwm2Regs.CMPB.bit.CMPB = countnumber; // Assign countnumber to EPWM2's CMPB
}

//------------------------------End of Lab6 exercise 2------------------
