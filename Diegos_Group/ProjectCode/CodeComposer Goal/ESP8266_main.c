//#############################################################################
// FILE:   labstarter_main.c
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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCD_ISR(void) ;


void serialRXA(serial_t *s, char data);
void serialRXC(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint32_t numRXC = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t ESP8266whichcommand = 0;
int16_t ESP8266insidecommands = 0;
float goal=0;
uint16_t timer=0;
float myfloat1=0;
//float myfloat2=0;
int16_t collect = 0;
char gusarray[50];
int16_t g = 0;
char debug_array[100];
int16_t debugi = 0;

float turn = 0;

int16_t adcd0result=0; // 2  result variables to store ADCIND0 and ADCIND1’s raw reading
int16_t adcd1result=0;
float voltage1=0;
float voltage2=0;
float v1k0=0; //adcd readings v1k0 and v2k0 is the current reading, v1k1 and v2k1 is the previos value. others also.
float v1k1=0;
float v1k2=0;
float v1k3=0;
float v1k4=0;
float v2k0=0;
float v2k1=0;
float v2k2=0;
float v2k3=0;
float v2k4=0;
void setDACA(float dacouta0) {
 if (dacouta0 > 3.0) dacouta0 = 3.0;
 if (dacouta0 < 0.0) dacouta0 = 0.0;
 DacaRegs.DACVALS.bit.DACVALS =(dacouta0* 4095)/3.0; // ????perform scaling of 0-3 to 0-4095??
}
void setDACB(float dacouta1) {
 if (dacouta1 > 3.0) dacouta1 = 3.0;
 if (dacouta1 < 0.0) dacouta1 = 0.0;
 DacbRegs.DACVALS.bit.DACVALS = (dacouta1* 4095)/3.0; // ????perform scaling of 0-3 to 0-4095??
}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file

    InitSysCtrl();

    InitGpio();

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    DELAY_US(100000);
    // RESET ESP8266
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    DELAY_US(500000);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    DELAY_US(3000000);

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
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.ADCD1_INT = &ADCD_ISR;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 10000);
    ConfigCpuTimer(&CpuTimer1, 200, 50000);
    ConfigCpuTimer(&CpuTimer2, 200, 5000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    init_serial(&SerialC,115200,serialRXC);
    //    init_serial(&SerialD,115200,serialRXD);

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; //?????? set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; //???? EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //????set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // ????EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //????set to SOC1, the last converted, and it will set INT1 flag ADCD1????
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    // Enable DACA and DACB outputs
        EALLOW;
        DacaRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacA output-->uses ADCINA0
        DacaRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
        DacaRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
        DacbRegs.DACOUTEN.bit.DACOUTEN = 1;//enable dacB output-->uses ADCINA1
        DacbRegs.DACCTL.bit.LOADMODE = 0;//load on next sysclk
        DacbRegs.DACCTL.bit.DACREFSEL = 1;//use ADC VREF as reference voltage
        EDIS;



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
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1; // for ADCD1
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    serial_send(&SerialC,"AT\r\n",strlen("AT\r\n"));

    displayLEDletter(0);
    ESP8266whichcommand = 1;
    ESP8266insidecommands = 1;
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"numRXA=%ld,numRXC=%ld,goal=%.1f v1:%.3f v2:%.3f\r\n",numRXA,numRXC,goal,voltage1,voltage2);
            UARTPrint = 0;
        }
    }
}

__interrupt void ADCD_ISR (void)
{
 adcd0result = AdcdResultRegs.ADCRESULT0;
 adcd1result = AdcdResultRegs.ADCRESULT1;
 // Here covert ADCIND0 to volts
 v1k0= ( adcd0result/4096.0)*3.0; //scaling the reading to real voltage
 v2k0= ( adcd1result/4096.0)*3.0;
 voltage1=(v1k0+v1k1+v1k2+v1k3+v1k4)*0.2;//simple filter that make an average of the five values
 voltage2=(v2k0+v2k1+v2k2+v2k3+v2k4)*0.2;

 v1k4=v1k3;
 v1k3=v1k2;
 v1k2=v1k1;
 v1k1=v1k0;//pass the value 

 v2k4=v2k3;
 v2k3=v2k2;
 v2k2=v2k1;
 v2k1=v2k0;


 // Here write voltages value to DACA
 setDACA(voltage1); //to scope the voltage reading. Make it easy to test.
 setDACB(voltage2);
 if (voltage1 > 1.1){//if the voltage of the IR sensor is greater than 1.1V, it means the ball goes into the goal. That will change the goal equal to 1.
     goal=1;
 } else if (voltage2 > 1.1){
     goal=1;
 }
 // Print ADCIND0’s voltage value to TeraTerm every 100ms
 AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
 PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
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

    if ((numTimer0calls%250) == 0) {
//        displayLEDletter(LEDdisplaynum);
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
    if (goal>0){
            timer++;
            if ((timer % 60) == 0){
                goal=0;
            }
        }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 20) == 0) {
              UARTPrint = 1;
    }

}


char sendto8266[99];
char send8266Command[64];
int16_t send8266CommandSize = 0;
int16_t sendingto8266 = 0;
int16_t sendtoSize = 0;
// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    //    if (sendingto8266 == 0) {
    //        sendto8266[0] = data;
    //        sendingto8266 = 1;
    //        serial_send(&SerialC,"AT+CIPSEND=0,1\r\n",strlen("AT+CIPSEND=0,1\r\n"));
    //        //serial_send(&SerialC,sendto8266,1);
    //    }


}




char sendback[10];
char past4[4] = {'\0','\0','\0','\0'};
// This function is called each time a char is received over UARTA.
void serialRXC(serial_t *s, char data) {
    numRXC++;
    if (ESP8266insidecommands == 1) {
        if (ESP8266whichcommand == 1) {
            past4[0] = data;
            if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                ESP8266whichcommand = 2;
                displayLEDletter(1);
                serial_send(&SerialC,"AT+CWMODE=3\r\n",strlen("AT+CWMODE=3\r\n"));
                past4[0] = '\0';
                past4[1] = '\0';
                past4[2] = '\0';
                past4[3] = '\0';
            }
            past4[3] = past4[2];
            past4[2] = past4[1];
            past4[1] = past4[0];
        } else if (ESP8266whichcommand == 2) {
            past4[0] = data;
            if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                ESP8266whichcommand = 3;
                displayLEDletter(2);
                //serial_send(&SerialC,"AT+CWJAP=\"COECSLNIGHT\",\"f33dback5\"\r\n",strlen("AT+CWJAP=\"COECSLNIGHT\",\"f33dback5\"\r\n"));
                serial_send(&SerialC,"AT+CWJAP=\"MECHNIGHT\",\"f33dback5\"\r\n",strlen("AT+CWJAP=\"MECHNIGHT\",\"f33dback5\"\r\n"));
                past4[0] = '\0';
                past4[1] = '\0';
                past4[2] = '\0';
                past4[3] = '\0';
            }
            past4[3] = past4[2];
            past4[2] = past4[1];
            past4[1] = past4[0];
        } else if (ESP8266whichcommand == 3) {
            past4[0] = data;
            if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                displayLEDletter(3);
                ESP8266whichcommand = 4;
                serial_send(&SerialC, "AT+CIPSTA=\"192.168.1.57\"\r\n", strlen("AT+CIPSTA=\"192.168.1.57\"\r\n")); //IP address to type into labview
                past4[0] = '\0';
                past4[1] = '\0';
                past4[2] = '\0';
                past4[3] = '\0';
            }
            past4[3] = past4[2];
            past4[2] = past4[1];
            past4[1] = past4[0];
        } else if (ESP8266whichcommand == 4) {
            past4[0] = data;
            if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                displayLEDletter(4);
                ESP8266whichcommand = 5;
                serial_send(&SerialC,"AT+CIPMUX=1\r\n", strlen("AT+CIPMUX=1\r\n"));
                past4[0] = '\0';
                past4[1] = '\0';
                past4[2] = '\0';
                past4[3] = '\0';
            }
            past4[3] = past4[2];
            past4[2] = past4[1];
            past4[1] = past4[0];
        } else if (ESP8266whichcommand == 5) {
            past4[0] = data;
            if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                ESP8266whichcommand = 6;
                displayLEDletter(5);
                serial_send(&SerialC,"AT+CIPSERVER=1,1336\r\n", strlen("AT+CIPSERVER=1,1336\r\n")); //1336 is the port to type into Labview
                past4[0] = '\0';
                past4[1] = '\0';
                past4[2] = '\0';
                past4[3] = '\0';
            }
            past4[3] = past4[2];
            past4[2] = past4[1];
            past4[1] = past4[0];
        } else if (ESP8266whichcommand == 6) {
            past4[0] = data;
            if ((past4[0]=='\n') && (past4[1]=='\r') && (past4[2]=='K') && (past4[3]=='O')) {

                displayLEDletter(6);
                ESP8266whichcommand = 12;  // should never get in 12
                ESP8266insidecommands = 0;
                past4[0] = '\0';
                past4[1] = '\0';
                past4[2] = '\0';
                past4[3] = '\0';
            }
            past4[3] = past4[2];
            past4[2] = past4[1];
            past4[1] = past4[0];

        } else if (ESP8266whichcommand == 12) {
            sendback[0] = ' ';
            sendback[1] = 'D';
            sendback[2] = 'a';
            sendback[3] = 'n';
            displayLEDletter(0xF);
            serial_send(&SerialA,sendback,4);
        }

        sendback[0] = data;
        serial_send(&SerialA,sendback,1);
    } else {  // ESP8266insidecommands == 0
        // for now just echo char
        if (sendingto8266 == 1) {
            if (data == '>') {
                serial_send(&SerialC,sendto8266,sendtoSize);
                sendingto8266 = 0;
            }
        } else if (collect==0){
            if (data == '*'){ //Reading from teraterm, looking for * symbol to start collecting into gusarray, maybe have different symbols for different notes?
                collect = 1;
                g = 0;
            }
        } else if (collect==1){ //If collect == 1, this is what we want to mainly edit
            if (data =='a') {
                sendingto8266 = 1;
                sendtoSize = sprintf(sendto8266,"%.5f\r\n",goal); //goal is the value we want to send to the LabVIEW.
                if (sendtoSize <= 9) {
                    send8266CommandSize = strlen("AT+CIPSEND=0,0\r\n");  // second zero is place holder for a number 1 to 9
                } else {
                    if (sendtoSize > 99) {
                        sendtoSize = 99;
                        sendto8266[97] = '\r';
                        sendto8266[98] = '\n';
                    }
                    send8266CommandSize = strlen("AT+CIPSEND=0,00\r\n"); // second zero zero is place holder for a number 10 to 99
                }
                sprintf(send8266Command,"AT+CIPSEND=0,%d\r\n",sendtoSize);
                serial_send(&SerialC,send8266Command,send8266CommandSize); //AT+CIPSEND is send command on chip. 0,3: Not sure what 0 is for (id?) but the 3 corresponds to length being sent.

                collect = 0;
                g=0;
            } else if (data =='\n'){ //End of array, should collect 1 or 2 floats from the * statement and turn it into myfloat variables, depending on labview
                gusarray[g]='\0';
                //sscanf(gusarray,"%f %f",&myfloat1,&myfloat2);
                sscanf(gusarray,"%f",&myfloat1);

                turn = myfloat1;
                UARTPrint = 1;

                collect = 0;
                g=0;
            } else {
                gusarray[g]=data;
                g++;
                if (g>=50){
                    g = 0;
                }
            }
        }
        if (debugi < 100) {
            debug_array[debugi] = data;
            debugi++;
        }
        sendback[0] = data;
        serial_send(&SerialA,sendback,1);
    }
}

