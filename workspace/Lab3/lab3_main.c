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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// My variables
void GPIO_setup(void);
void EPWM_setup(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);

//Lab3 Part1
int16_t DIR_LED = 1;

//Lab3 Part2
int16_t DIR_MOTOR = 1;
float MOTOR_CE = 0;

//Lab3 Part3
int16_t DIR_RC = 1;
float ANGLE_CE = 0;

// Lab3 Part4
uint16_t songIndex = 0; //initializes the variable tracking motor duty cycle

void setEPWM2A(float controleffort) {
    if (controleffort < -10) {
        controleffort = -10;
    } else if (controleffort > 10) {
        controleffort = 10;
    }
    EPwm2Regs.CMPA.bit.CMPA = ((float)EPwm2Regs.TBPRD) * ((controleffort + 10.0) / 20.0);
}

void setEPWM2B(float controleffort) {
    if (controleffort < -10) {
        controleffort = -10;
    } else if (controleffort > 10) {
        controleffort = 10;
    }
    EPwm2Regs.CMPB.bit.CMPB = ((float)EPwm2Regs.TBPRD) * ((controleffort + 10.0) / 20.0);
}

// LAB3PART3 Sets a corresponding duty cycle according to the given angle
// Output duty cycle ranges from 4% to 12% mapped from -90 to 90 degrees
void setEPWM8A_RCServo(float angle) {
    if (angle > 90) { // If greater than max angle, saturate to max angle
        angle = 90;
    } else if (angle < -90) { // If less than min angle, saturate to min angle
        angle = -90;
    }
    EPwm8Regs.CMPA.bit.CMPA = ((float)EPwm8Regs.TBPRD) * ((angle + 180.0) / 2250.0); // Map angle in degrees to RC servo duty cycle
                                                                                     // Maps (-90 degrees, 90 degrees) to (4%, 12%)
}

// LAB3PART3 Same as previous function, but for second motor
void setEPWM8B_RCServo(float angle) {
    if (angle > 90) {
        angle = 90;
    } else if (angle < -90) {
        angle = -90;
    }
    EPwm8Regs.CMPB.bit.CMPB = ((float)EPwm8Regs.TBPRD) * ((angle + 180.0) / 2250.0);
}

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

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    GPIO_setup();

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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000); // Lab3 Part4: Changed period to 1/8 s (125 ms)
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
				serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
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

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

//    if ((numTimer0calls%250) == 0) {
//        displayLEDletter(LEDdisplaynum);
//        LEDdisplaynum++;
//        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
//            LEDdisplaynum = 0;
//        }
//    }

	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;


}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    // LAB3PART4
    EPwm9Regs.TBPRD = song2array[songIndex]; // Change TBPRD, which changes the buzzer signal frequency, to current note in songarray
    if (songIndex < SONG2_LENGTH) { // If we are not at the end of the song, advance to next note
        songIndex++;
    } else if (songIndex >= SONG_LENGTH) { // Else if song has ended, change pin back to a GPIO pin and set to low (to avoid noise)
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
    }
	
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    //Lab3 Part1
//    if (DIR_LED == 1) { // DIR_LED tracks whether the LED should brighten or darken (increase/decrease CMPA)
//        EPwm12Regs.CMPA.bit.CMPA ++;
//        if (EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD) { // If at max brightness, signal to begin darkening LED (flip DIR_LED value)
//            DIR_LED = 0;
//        }
//    } else if (DIR_LED == 0) { // Now decrease brightness since DIR_LED == 0
//        EPwm12Regs.CMPA.bit.CMPA --;
//        if (EPwm12Regs.CMPA.bit.CMPA == 0){ // Flip DIR_LED when at minimum brightness
//            DIR_LED = 1;
//        }
//    }

    //Lab3 Part2
//    if (DIR_MOTOR == 1) { // DIR_LED tracks whether the LED should brighten or darken (increase/decrease CMPA)
//        MOTOR_CE += 0.001;
//        setEPWM2A(MOTOR_CE);
//        setEPWM2B(MOTOR_CE);
//        if (MOTOR_CE >= 10) { // If at max brightness, signal to begin darkening LED (flip DIR_LED value)
//            DIR_MOTOR = 0;
//        }
//    } else if (DIR_MOTOR == 0) { // Now decrease brightness since DIR_LED == 0
//        MOTOR_CE -= 0.001;
//        setEPWM2A(MOTOR_CE);
//        setEPWM2B(MOTOR_CE);
//        if (MOTOR_CE <= -10) { // If at max brightness, signal to begin darkening LED (flip DIR_LED value)
//            DIR_MOTOR = 1;
//        }
//    }

    // LAB3PART3: RC MOTOR DIR CHANGE
//    if (DIR_RC == 1) { // DIR_RC tracks when to increase/decrease RC motor speed
//        ANGLE_CE += 0.1; // RC_ANGLE is motor angle that is passed to setEPWM8A_RCServo function, sets RC servo to specified angle
//        if (ANGLE_CE >= 90.0) { // If above max angle (use > because floating point math is not perfect), begin decreasing (flip DIR_RC value)
//            DIR_RC = 0;
//        }
//    } else if (DIR_RC == 0) {
//        ANGLE_CE -= 0.1;
//        if (ANGLE_CE <= -90.0) { // If below min angle, begin increasing
//            DIR_RC = 1;
//        }
//    }
//    // Call functions that set RC servo duty cycle in order to move it to specified angle
//    setEPWM8A_RCServo(ANGLE_CE);
//    setEPWM8B_RCServo(ANGLE_CE);
//
//    CpuTimer2.InterruptCount++;
//	if ((CpuTimer2.InterruptCount % 50) == 0) {
//		UARTPrint = 1;
//	}
}
