#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "RotaryEncoder.h"


//slave select macros
#define SLAVE_SELECT_INIT() do { TRISDbits.TRISD3 = 0; } while (0)
#define SLAVE_SELECT_SET(state) do { LATDbits.LATD3 = state; } while (0)

#define SPI_NOP_COMMAND 0xC000
#define ENCODER_ANGLE_REGISTER 0xFFFF// 0x7FFE
#define MSB_BIT_MASK 0x3FFF

#define TIMER_PERIOD 0x4E20 // 20,000 ticks with 1:2 prescaler = 1ms (0x9C40 -> 2ms)

static char mode;
static unsigned short rawAngle = 0;

/**
 * @Function RotaryEncoder_Init(char interfaceMode)
 * @param interfaceMode, one of the two #defines determining the interface
 * @return SUCCESS or ERROR
 * @brief initializes hardware in appropriate mode along with the needed interrupts */
int RotaryEncoder_Init(char interfaceMode){
    mode = interfaceMode;
    //SPI configuration
    SPI2CON = 0;//clears all SPI2 control bits  (32 bits in total)

    SPI2CONbits.MSTEN = 1; //enables master mode

    SPI2CONbits.MODE32 = 0; //16 bit mode
    SPI2CONbits.MODE16 = 1;

    SPI2CONbits.CKE = 0; //falling edge - mode 1
    SPI2CONbits.CKP = 0; //idle low - mode 1
    SPI2CONbits.SMP = 1; //clock phase - mode 1

    unsigned int clc = SPI2BUF; //clearing current buffer
    SPI2BRG = 3; //sets the master clock frequency to 5 MHz

    SLAVE_SELECT_INIT(); //sets RD3 as output
    SLAVE_SELECT_SET(1); //sets value of slave select to high
    
    if(interfaceMode == ENCODER_INTERRUPT_MODE){
        IEC1bits.SPI2RXIE = 1; //SPI2 receive interrupt enable
        IPC7bits.SPI2IP = 2; //SPI2 interrupt priority
        IFS1bits.SPI2RXIF = 0; //lower receive interrupt flag
        
        //Timer 2 Configuration
        T2CON = 0x0;//clearing the control bits, also turns timer off

        T2CONbits.TCKPS = 1; //timer pre-scaler 1:2, 50ns between ticks
        TMR2 = 0; //reset the timer bits
        PR2 = TIMER_PERIOD; //sets the period match to 20,000 ticks which is in total 1ms
    
        IEC0bits.T2IE = 1; //enable timer2 interrupt
        IPC2bits.T2IP = 2; //interrupt priority for timer2
        IFS0bits.T2IF = 0; //clears interrupt flag
        
        T2CONbits.ON = 1; //turn on timer 2 
        SPI2CONbits.ON = 1; //enable SPI
        
        SLAVE_SELECT_SET(0); //start transmission with encoder
        SPI2BUF = ENCODER_ANGLE_REGISTER; //sending angle address
    }else{
       SPI2CONbits.ON = 1; //enable SPI 
    }
}

/**
 * @Function int RotaryEncoder_ReadRawAngle(void)
 * @param None
 * @return 14-bit number representing the raw encoder angle (0-16384) */
unsigned short RotaryEncoder_ReadRawAngle(void){
    if(mode == ENCODER_BLOCKING_MODE){
        asm(" nop "); //delay 500ns
        SLAVE_SELECT_SET(0);
        asm(" nop "); //delay 500ns
        SPI2BUF = (unsigned short)ENCODER_ANGLE_REGISTER; //data must be configured
        while(SPI2STATbits.SPIRBF == 0); //waiting for reception to finish
        unsigned short clc = SPI2BUF;
        SLAVE_SELECT_SET(1);
        asm(" nop "); //delay 500ns

        SLAVE_SELECT_SET(0);
        asm(" nop "); //delay 500ns
        SPI2BUF = (unsigned short)SPI_NOP_COMMAND; //data must be configured
        while(SPI2STATbits.SPIRBF == 0); //waiting for reception to finish
        rawAngle = SPI2BUF;
        SLAVE_SELECT_SET(1);
        asm(" nop "); //delay 500ns

        return (rawAngle & MSB_BIT_MASK); //remove 2 MSB
    }
    
    return rawAngle;  
}

//parity function used to inserting parity bits into a message
int parity(unsigned short in){
    int p = 0;
    int i = 1;
    while(i < 0x8000){
        if(in & i){
            p += 1;
        }
        i = (i << 1);
    }
    p = p%2;
    return p;
}

//adds parity bit to MSB of the data
int configureData(short int data){
    int par = parity(data);
    int config = data | (par << 15);
    return config;
}

void __ISR ( _TIMER_2_VECTOR ) Timer2IntHandler ( void ) {
    IFS0bits.T2IF = 0; //lower timer flag
    
    unsigned short clc = SPI2BUF; //making sure the buffer is clear
    SLAVE_SELECT_SET(0); //start transmission with encoder
    SPI2BUF = ENCODER_ANGLE_REGISTER; //sending angle address
}

static int count = 0;
void __ISR ( _SPI_2_VECTOR ) __SPI2Interrupt ( void ) {
    IFS1bits.SPI2RXIF = 0; //lower RX flag
    
    SLAVE_SELECT_SET(1);
    
    rawAngle = SPI2BUF;
    rawAngle = rawAngle & MSB_BIT_MASK;
}

//#define IF_TESTING_RE_BLOCKING

#ifdef IF_TESTING_RE_BLOCKING
#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"

int main(){
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    FreeRunningTimer_Init();
    RotaryEncoder_Init(ENCODER_BLOCKING_MODE);
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    sprintf(testMessage, "SPI BLOCKING MODE");
    Protocol_SendDebugMessage(testMessage);
    
    unsigned short packet = 0;
    unsigned int previousTime = FreeRunningTimer_GetMilliSeconds();
    while (1){
        unsigned int currentTime = FreeRunningTimer_GetMilliSeconds();
        
        if(currentTime-previousTime >= 10){ //10 millisecond delay -> 100Hz
            previousTime = currentTime;
            
            packet = RotaryEncoder_ReadRawAngle();
            packet = Protocol_ShortEndednessConversion(packet);
            Protocol_SendMessage(2,ID_ROTARY_ANGLE,&packet);
            
        }
        
    }
}

#endif

//#define IF_TESTING_RE_INTERRUPT

#ifdef IF_TESTING_RE_INTERRUPT
#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"

int main(){
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    FreeRunningTimer_Init();
    RotaryEncoder_Init(ENCODER_INTERRUPT_MODE);
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    sprintf(testMessage, "SPI INTERRUPT MODE");
    Protocol_SendDebugMessage(testMessage);
    
    unsigned short packet = 0;
    unsigned int previousTime = FreeRunningTimer_GetMilliSeconds();
    while (1){
        unsigned int currentTime = FreeRunningTimer_GetMilliSeconds();
        
        if(currentTime-previousTime >= 10){ //10 millisecond delay -> 100Hz
            previousTime = currentTime;
            
            packet = RotaryEncoder_ReadRawAngle();
            packet = Protocol_ShortEndednessConversion(packet);
            Protocol_SendMessage(2,ID_ROTARY_ANGLE,&packet);
        }
        
    }
}

#endif