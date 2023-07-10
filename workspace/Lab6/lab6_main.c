//#############################################################################
 // FILE:   lab6_main.c
//
// TITLE:  Lab 6: DC Motor Speed Control and Steering a Three Wheel Robot Car
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

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

float accelgain = 1.0;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// LAB 5: My variables
void GPIO_setup(void);
void SPI_setup(void);
__interrupt void SPIB_ISR(void);
float numberToAcceleration(int16_t value); // Converts given sensor reading from (-32768, 32767) to (-4, 4)
float numberToRotVel(int16_t value); // Converts given sensor reading from (-32768, 32767) to (-250, 250)
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t DAN_PWM1 = 0;
int16_t DAN_PWM2 = 0;
int16_t temp = 0;
int16_t DAN_ADC1 = 0;
int16_t DAN_ADC2 = 0;
float ADC1_volts = 0;
float ADC2_volts = 0;
int16_t DAN_PWM_flip = 1;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

// LAB 6: My variables
void eQEPs_setup(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
float readEncLeft(void);
float readEncRight(void);
float leftWheelAngle = 0.0;
float rightWheelAngle = 0.0;
void setDACA(float volts);
void setDACB(float volts);

float uLeft = 5.0; // Initialize to 5 control effort (zero speed)
float uRight = 5.0;

//Velocity calculations
float posLeftk;
float posLeftk_1;
float posRightk;
float posRightk_1;
float vLeftk;
float vRightk;
float Vref = 0;
float Kp = 3.0;
float Ki = 25.0;
float errLeftk;
float errLeftk_1;
float errRightk;
float errRightk_1;
float ILeftk;
float ILeftk_1;
float IRightk;
float IRightk_1;
float turn = 0;
float Kp_turn = 3;
float errTurn;
// L6E5:
float rWheel = 0.1955;
float width = 0.576;
float pose;
float thetaAvg;
float omegaAvg;
float x_vel;
float y_vel;
float x_coor;
float y_coor;
float x_vel_1;
float y_vel_1;
float x_coor_1;
float y_coor_1;
float leftWheelAngle_1;
float rightWheelAngle_1;
float omegaLeft;
float omegaRight;
uint16_t identifier = 0;
uint16_t counter = 0;
uint16_t busy = 0;
float angleCounter = 0;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// SETUP FUNCTIONS ONLY
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void GPIO_setup(void) {
    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

//    // LED1 and PWM Pin
//    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
//    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

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
}

void EPWM_setup(void) {
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

    // Setting the MUX values for lab 3 EPWM pins
    // LED1
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);
    // DC MOTOR 1
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    // DC MOTOR 2
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    // SERVO MOTOR 1
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);
    // SERVO MOTOR 2
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);
    // BOARD BUZZER
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);

    // EPWM settings for desired pins
    EPwm12Regs.TBCTL.bit.CTRMODE = 0;  //count up mode
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2;  //free run mode so PWM continues after a break point
    EPwm12Regs.TBCTL.bit.CLKDIV = 0;  //sets signal divide to 1
    EPwm12Regs.TBCTL.bit.PHSEN = 0;  //disables phase loading
    EPwm12Regs.TBCTR = 0;  //starts timer at zero
    EPwm12Regs.TBPRD = 2500;  //sets carrier frequency to 20 kHz
    // Carrier frequency= signal frequency/TBPRD
    EPwm12Regs.CMPA.bit.CMPA = 0;  //sets duty cycle value (started at 0% duty)
    EPwm12Regs.AQCTLA.bit.CAU = 1;  //sets EPWM signal low when TBCTR == CMPA
    EPwm12Regs.AQCTLA.bit.ZRO = 2;  //sets EPMW signal high when TBCTR == 0
    EPwm12Regs.TBPHS.bit.TBPHS = 0;  //sets phase to zero

    // Same settings as EPWM12 signal unless otherwise commented
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250;  //sets motor speed to 0 (50% duty cycle)
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    // Sets up EPWM2B specific settings
    EPwm2Regs.CMPB.bit.CMPB = 1250;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;

    //Same settings as EPWM2 signal unless otherwise commented
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; // Divide input frequency by 16
                                    // This is done in order to represent TBPRD with 16 bits due to a LOW carrier frequency
                                    // TBPRD can only represent up to 65,535, with CLKDIV=0, TBPRD far exceeded this maximum value
    EPwm8Regs.TBCTL.bit.PHSEN = 0;
    EPwm8Regs.TBCTR = 0;
    EPwm8Regs.TBPRD = 62500; // Period needed for 50 Hz carrier frequency with CLKDIV=4
                             // Carrier frequency = (CLKDIV * TBPRD) / INPUT FREQUENCY (50 MHz)
    EPwm8Regs.CMPA.bit.CMPA = 5000; // Initially set to 8% duty cycle (0 degree angle)
    EPwm8Regs.AQCTLA.bit.CAU = 1;
    EPwm8Regs.AQCTLA.bit.ZRO = 2;
    EPwm8Regs.TBPHS.bit.TBPHS = 0;
    // Sets up EPWM8B specific settings
    EPwm8Regs.CMPB.bit.CMPB = 5000; // Initially set to 8% duty cycle (0 degree angle)
    EPwm8Regs.AQCTLB.bit.CBU = 1.;
    EPwm8Regs.AQCTLB.bit.ZRO = 2;

    // Same settings as EPWM12 signal unless otherwise commented
    EPwm9Regs.TBCTL.bit.CTRMODE = 0;
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm9Regs.TBCTL.bit.CLKDIV = 1; // Given
    EPwm9Regs.TBCTL.bit.PHSEN = 0;
    EPwm9Regs.TBCTR = 0;
    EPwm9Regs.TBPRD = 0;
    EPwm9Regs.AQCTLA.bit.CAU = 0; // Do nothing when TBCTR = CMPA
                                  // This is done in order to achieve constant 50% duty cycle to the buzzer signal (read ZRO comment)
    //EPwm9Regs.CMPA.bit.CMPA = 0; Commented out because buzzer signal always has 50% duty cycle, we do not care what CMPA is
    EPwm9Regs.AQCTLA.bit.ZRO = 3; // When TBCTR = 0 (the step after TBCTR=TBPRD), toggle output square wave
                                  // If signal is HIGH, change to LOW. If signal is LOW, change to HIGH
    EPwm9Regs.TBPHS.bit.TBPHS = 0;
}

void SPI_setup(void) {
    // Step 1: cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    // between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    // 66 which are also a part of the SPIB setup.

    // GPIO Setup
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

    // SPIB Setup
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
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

//    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPIFFCT.bit.TXDLY = 0; // L5E4: Set to 0
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes

    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;


    //Step 2: perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
    // sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // L5E4: Remember have to specify FIRST address. So first 8 bits specifies the starting address,
    // then second 8 bits are the data going into that first address. Every 8 bits after are the
    // date to be stored in the subsequent addresses
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
    SpibRegs.SPITXBUF = 0x1300;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0013;
    SpibRegs.SPITXBUF = 0x0200; // Config 8-bit: 00000010, Gyro Config: 00000000
    // Sets Fchoice_b = 00, which means Fchoice = 11, enabling DLPF_CFG in config, which allows for a gyro cutoff frequency of 92 Hz
    // Gyro config also gives rotational velocity scale of +-250 deg/s (highest resolution)
    SpibRegs.SPITXBUF = 0x0806; // Accel Config 1: 00001000, Accel Config 2: 00000110
    // Accel config 1 gives accel scale of +- 4g's
    // Accel config 2 gives accel_fchoice_b = 0, so accel_fchoice = 1. With a_dlpf_cfg = 6, there is a bandwidth of 5 Hz according to table
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.


    // Step 3: perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x2300;
    SpibRegs.SPITXBUF = 0x408C;
    SpibRegs.SPITXBUF = 0x0288;
    SpibRegs.SPITXBUF = 0x0C0A;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.


    // Step 4: perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);


    // Step 5: the remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);


    // Acceleration Offsets

    // X-accel
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0016); // Default: 0x77EB
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00A8); // Default: 0x7812
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // Y-accel
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E8); // Default: 0x7A10
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0090); // Default: 0x7BFA
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // Z-accel
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // Default: 0x7D21
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // Default: 0x7E50
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

void DAC_setup(void) {
    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
    EDIS;
}

void eQEPs_setup(void) {
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


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MAIN
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    GPIO_setup();

    // LAB 6
    EPWM_setup();

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
    PieVectTable.SPIB_RX_INT = &SPIB_ISR; // LAB 5

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
//    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000); // L5E2
//    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 20000); // L5E3
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // L5E4
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    // LAB 5
    SPI_setup();
    DAC_setup();

    // LAB 6
    eQEPs_setup();

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;  // LAB 5: SPIB
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // LAB5: Enable SPIB_RX: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    
    // IDLE loop. Just sit and loop forever (optional):
    while (1)
    {
        if (UARTPrint == 1 ) {
//            serial_printf(&SerialA,"Num Timer 0:%ld, JS1: %.3f, JS2: %.3f \r\n",numTimer0calls,ADC1_volts, ADC2_volts);
//            serial_printf(&SerialA,"Acceleration: X: %.3f, Y: %.3f, Z: %.3f \r\nRotation: X: %.3f, Y: %.3f, Z: %.3f \r\n",accelX,accelY,accelZ,gyroX,gyroY,gyroZ);
            serial_printf(&SerialA,"Left Angle: %.3f, Right Angle: %.3f \r\n",leftWheelAngle,rightWheelAngle);
            serial_printf(&SerialA,"Left Velocity: %.3f, Right Velocity: %.3f, Left Angle: %.3f, Right Angle: %.3f \r\n",vLeftk, vRightk, leftWheelAngle,rightWheelAngle);
            UARTPrint = 0;
        }
    }
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// INTERRUPT FUNCTIONS ONLY
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
__interrupt void SPIB_ISR(void){
    // MPU
    temp = SpibRegs.SPIRXBUF;
    int16_t accelX_raw = SpibRegs.SPIRXBUF;
    int16_t accelY_raw = SpibRegs.SPIRXBUF;
    int16_t accelZ_raw = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    int16_t gyroX_raw = SpibRegs.SPIRXBUF;
    int16_t gyroY_raw  = SpibRegs.SPIRXBUF;
    int16_t gyroZ_raw  = SpibRegs.SPIRXBUF;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO66 high to end Slave Select

    accelX = numberToAcceleration(accelX_raw);
    accelY = numberToAcceleration(accelY_raw);
    accelZ = numberToAcceleration(accelZ_raw);
    gyroX = numberToAcceleration(gyroX_raw);
    gyroY = numberToAcceleration(gyroY_raw);
    gyroZ = numberToAcceleration(gyroZ_raw);


    setDACA(accelgain*accelZ + 1.5);

    // Acknowledge code
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
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
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // DEFAULT CPU0_ISR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

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
    // DEFAULT CPU0_ISR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // TELLING MPU TO SEND US DATA
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = 0xBA00;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    // Initial angle reading test
    leftWheelAngle = readEncLeft();
    rightWheelAngle = readEncRight();

    // Calculate distant position from angle
    posLeftk = leftWheelAngle * rWheel;
    posRightk = rightWheelAngle * rWheel;

    vLeftk = (posLeftk - posLeftk_1) / 0.004;
    vRightk = (posRightk - posRightk_1) / 0.004;
	
    // Set EPWMs (wheel speeds) using uLeft and uRight
    setEPWM2A(uRight);
    setEPWM2B(-uLeft);

    // Exit commands
    posLeftk_1 = posLeftk;
    posRightk_1 = posRightk;
    CpuTimer1.InterruptCount++;
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


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HELPER FUNCTIONS ONLY
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setDACA(float volts) {
    int16_t DACOutInt = 0;
    DACOutInt = volts * (4095 / 3.0); // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}

void setDACB(float volts) {
    int16_t DACOutInt = 0;
    DACOutInt = volts * (4095 / 3.0); // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}

// LAB3PART2 Sets a corresponding duty cycle according to the given control effort
// Output duty cycle ranges from 0% to 100% mapped from -10 to 10 control effort
void setEPWM2A(float controleffort) {
    if (controleffort > 10) {  //saturates at max effort, 10 (duty cycle 100%)
        controleffort = 10;
    } else if (controleffort < -10) {  //saturates at min effort, -10 (duty cycle 0%)
        controleffort = -10;
    }
    EPwm2Regs.CMPA.bit.CMPA = ((float)EPwm2Regs.TBPRD) * ((controleffort + 10.0)/20.0); //setting the CMPA value directly according to the calculated duty cycle
}

// LAB3PART2 Same as previous function, but for second motor
void setEPWM2B(float controleffort) {
    if (controleffort > 10) {
        controleffort = 10;
    } else if (controleffort < -10) {
        controleffort = -10;
    }
    EPwm2Regs.CMPB.bit.CMPB = ((float)EPwm2Regs.TBPRD) * ((controleffort + 10.0)/20.0);
}

float readEncLeft(void) {
 int32_t raw = 0;
 uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
 raw = EQep1Regs.QPOSCNT;
 if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
// 100 slits in the encoder disk so 100 square waves per one revolution of the
// DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
// of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
 return (-raw*(2.0*PI/12000.0)); // Added negative sign to fix left wheel output
}

float readEncRight(void) {
 int32_t raw = 0;
 uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
 raw = EQep2Regs.QPOSCNT;
 if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
// 100 slits in the encoder disk so 100 square waves per one revolution of the
// DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
// of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
 return (raw*(2.0*PI/12000.0)); // Added negative sign to fix left wheel output
}

float numberToRotVel(int16_t value) {
    if (value < 0) {
        return value * (250.0 / 32768.0);
    }
    return value * (250.0 / 32767.0);
}

float numberToAcceleration(int16_t value) {
    if (value < 0) {
        return value * (4.0 / 32768.0);
    }
    return value * (4.0 / 32767.0);
}
