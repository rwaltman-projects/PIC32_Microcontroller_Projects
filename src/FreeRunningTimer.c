#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "Protocol.h"
#include "FreeRunningTimer.h"

//#define IF_TESTING_FRT


static unsigned int milliCount = 0;
static unsigned int microCount = 0;
/**
 * @Function TIMERS_Init(void)
 * @param none
 * @return None.
 * @brief  Initializes the timer module */
void FreeRunningTimer_Init(void){
    T5CON = 0x0;//clearing the control bits, also turns timer off
    
    T5CONbits.TCKPS = 3; //timer pre-scaler 1:8, 200ns between ticks, 5 ticks = 1 microsecond
    TMR5 = 0; //reset the timer bits
    PR5 = 0x1388; //sets the period match to 5000 ticks = 1ms 
    
    IEC0bits.T5IE = 1; //enable timer5 interrupt
    IPC5bits.T5IP = 1; //interrupt priority for timer5
    IFS0bits.T5IF = 0; //clears interrupt flag
    
    T5CONbits.ON = 1; //turns the timer on
}

/**
 * Function: TIMERS_GetMilliSeconds
 * @param None
 * @return the current MilliSecond Count
   */
unsigned int FreeRunningTimer_GetMilliSeconds(void){
    return milliCount;
}

/**
 * Function: TIMERS_GetMicroSeconds
 * @param None
 * @return the current MicroSecond Count
   */
unsigned int FreeRunningTimer_GetMicroSeconds(void){
    unsigned int cur = microCount + (TMR5/5);
    return cur; //time is in milliseconds
}

void __ISR ( _TIMER_5_VECTOR , ipl3auto ) Timer5IntHandler ( void ) {
    IFS0bits.T5IF = 0; //lowers interrupt flag
    milliCount++;
    microCount+=1000; //1 milliseconds = 1000 microseconds
}

#ifdef IF_TESTING_FRT

int main(){
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    FreeRunningTimer_Init();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    
    unsigned char led_state = 0;
    unsigned int previousTime = FreeRunningTimer_GetMilliSeconds();
    while (1){
        unsigned int currentTime = FreeRunningTimer_GetMilliSeconds();
        unsigned int microTime = FreeRunningTimer_GetMicroSeconds();
        
        if(currentTime-previousTime >= 2000){ //2000 milliseconds = 2 seconds
            previousTime = currentTime;
            
            led_state ^= 0x01; //toggle right most led state
            LEDS_SET(led_state);
            
            sprintf(testMessage, "Millisecond Count: %d, Microsecond Count: %d", currentTime, microTime);
            Protocol_SendDebugMessage(testMessage);
        }
        
    }
}

#endif
