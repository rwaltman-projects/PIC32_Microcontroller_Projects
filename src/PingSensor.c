#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "../include/BOARD.h"
#include "PingSensor.h"

//#define IF_TESTING_PS

#define TRIGGER_INIT() do {TRISDbits.TRISD9 = 0;} while (0) //setting pin 7 on the shield to an output
#define TRIGGER_SET(val) do { LATDbits.LATD9 = val; } while (0)

#define IC3PIN_GET() (PORTDbits.RD10) //returns the current value of the IC3Pin

#define MICRO_DELAY 7 //7 ticks of the TMR4 module is greater than 10 micro seconds
#define TIME_TO_DISTANCE_NUMERATOR 68
#define TIME_TO_DISTANCE_DENOMINATOR 4000


static unsigned int time = 0;
static unsigned int prevTime = 0;

static unsigned int upgoing = 0;
static unsigned int downgoing = 0;
static unsigned int dif = 0;
/**
 * @Function PingSensor_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes hardware for PingSensor with the needed interrupts */
int PingSensor_Init(void){
    //initializing Trigger Timer 4
    T4CON = 0x0;//clearing the control bits, also turns timer off
    
    T4CONbits.TCKPS = 6; //timer pre-scaler 1:64, 1600ns between ticks, 1 tick = 1.6 microsecond (6))
    TMR4 = 0; //reset the timer bits
    PR4 = 0x927C; //sets the period match to 625 ticks = 1ms, 37500 ticks = 60ms  (0x927C)
    
    IEC0bits.T4IE = 1; //enable timer5 interrupt
    IPC4bits.T4IP = 1; //interrupt priority for timer5
    IFS0bits.T4IF = 0; //clears interrupt flag
    
    TRIGGER_INIT();
    TRIGGER_SET(0);
    T4CONbits.ON = 1; //turns the timer on
    
    //Timer 2 Configuration
    T2CON = 0x0;//clearing the control bits, also turns timer off
    
    T2CONbits.TCKPS = 3; //timer pre-scaler 1:8, 200ns between ticks
    TMR2 = 0; //reset the timer bits
    PR2 = 0xEA60; //sets the period match to 60,000 ticks which is in total 12ms
   
    //setting up IC 
    IC3CONbits.ON = 0; //disable IC3
    
    IC3CONbits.ICM = 6; //IC mode: captures on every edge, starting with rising
    IC3CONbits.FEDGE = 1; //captures rising edge first
    IC3CONbits.ICTMR = 1; //sets the IC timer to timer 2
    IC3CONbits.ICI = 0; //interrupt on every capture event
    
    IPC3bits.IC3IP = 1; //priority level 1
    IEC0bits.IC3IE = 1; //enables interrupt
    
    IC3CONbits.ON = 1; //enable IC3
    T2CONbits.ON = 1; //turn on timer 2 
    
    
}

/**
 * @Function int PingSensor_GetDistance(void)
 * @param None
 * @return Unsigned Short corresponding to distance in millimeters */
unsigned short PingSensor_GetDistance(void){
    //unsigned short dist = (unsigned short)(TIME_TO_DISTANCE*dif);
    unsigned short dist = (dif*TIME_TO_DISTANCE_NUMERATOR)/TIME_TO_DISTANCE_DENOMINATOR;
    return dist;
}
//trigger timer

void __ISR ( _TIMER_4_VECTOR ) Timer4IntHandler ( void ){
    IFS0bits.T4IF = 0; //clear Timer interrupt
    time++;
    
    //sets the trigger high for approximately 11.2 micro seconds
    TRIGGER_SET(1);
    unsigned int prevMicro = TMR4;
    unsigned int currentMicro = TMR4;
    while(currentMicro - prevMicro < MICRO_DELAY){
        currentMicro = TMR4;
    }
    TRIGGER_SET(0);
}


//IC interrupt
//might want to change the code so that the buffer is always read
void __ISR ( _INPUT_CAPTURE_3_VECTOR ) __IC3Interrupt ( void ){
    static int up = 1;
    IFS0bits.IC3IF = 0; //clear IC interrupt
    if(up){
        if(IC3PIN_GET() == 1){
            upgoing = (unsigned short)(0xFFFF & IC3BUF);
        }else{
            up = 0; //polarity is messed up, attempt to take the rising edge again
        }
    }else{
        if(IC3PIN_GET() == 0){
            downgoing = (unsigned short)(0xFFFF & IC3BUF);
            dif = (unsigned short)(downgoing - upgoing);
        }else{
            up = 0; //polarity is messed up, attempt to take the rising edge again
        }
    }
    up ^= 1;
}

#ifdef IF_TESTING_PS
#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"

int main(){
    BOARD_Init();
    LEDS_INIT();
    Protocol_Init();
    FreeRunningTimer_Init();
    PingSensor_Init();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    
    unsigned int previousTime = FreeRunningTimer_GetMilliSeconds();
    while (1){
        unsigned int currentTime = FreeRunningTimer_GetMilliSeconds();
        
        if(currentTime-previousTime >= 100){ //100 milliseconds -> 10Hz
            previousTime = currentTime;
            unsigned int converted = Protocol_ShortEndednessConversion(PingSensor_GetDistance());
            Protocol_SendMessage(2,ID_PING_DISTANCE,&converted);
            
        }
        
    }
}

#endif


