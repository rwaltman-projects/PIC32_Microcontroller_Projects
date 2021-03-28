#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "RCServo.h"

//#define IF_TESTING_RCS

// these two macros N/M is equal to pulse to tick conversion
#define MICRO_TICK_CONV_N 5
#define MICRO_TICK_CONV_D 4

static unsigned int highTime = RC_SERVO_CENTER_PULSE; //pulse length in micro seconds

/**
 * @Function RCServo_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes hardware required and set it to the CENTER PULSE */
int RCServo_Init(void){
    //configuring timer 3
    T3CON = 0x0;//clearing the control bits, also turns timer off
    
    T3CONbits.TCKPS = 5; //timer pre-scaler 1:32, 800ns between ticks
    TMR3 = 0; //reset the timer bits
    PR3 = 0xF424; //sets the period match to 62,500 ticks which is in total 50ms
    
    
    OC3CONbits.ON = 0; //disable OC3
    OC3R = 0; //sets pulse time to default center
    OC3RS = (RC_SERVO_CENTER_PULSE*MICRO_TICK_CONV_N)/MICRO_TICK_CONV_D; //sets pulse time to default center
    
    
    OC3CONbits.OCTSEL = 1; //sets the timer to timer 3
    OC3CONbits.OCM = 5; //sets to PWM mode, disabled fault
     
    IPC3bits.OC3IP = 1; //priority level 1
    IEC0bits.OC3IE = 1; //enables interrupt
    IFS0bits.OC3IF = 0; //raises interrupt flag to jump start
    
    OC3CONbits.ON = 1; //enables OC3
    T3CONbits.ON = 1; //enables Timer 3
}

/**
 * @Function int RCServo_SetPulse(unsigned int inPulse)
 * @param inPulse, integer representing number of microseconds
 * @return SUCCESS or ERROR
 * @brief takes in microsecond count, converts to ticks and updates the internal variables
 * @warning This will update the timing for the next pulse, not the current one */
int RCServo_SetPulse(unsigned int inPulse){
    if(inPulse > RC_SERVO_MAX_PULSE || inPulse < RC_SERVO_MIN_PULSE){
        return ERROR;
    }
    highTime = inPulse;
    return SUCCESS;
}

/**
 * @Function int RCServo_GetPulse(void)
 * @param None
 * @return Pulse in microseconds currently set */
unsigned int RCServo_GetPulse(void){
    return highTime;
}

/**
 * @Function int RCServo_GetRawTicks(void)
 * @param None
 * @return raw timer ticks required to generate current pulse. */
unsigned int RCServo_GetRawTicks(void){
    return ((highTime*MICRO_TICK_CONV_N)/MICRO_TICK_CONV_D);
}

static unsigned int amount = 0;
//OC3 interrupt, interrupts whenever timer overflows (every 50ms)
void __ISR ( _OUTPUT_COMPARE_3_VECTOR ) __OC3Interrupt ( void ) {
    IFS0bits.OC3IF = 0; //lowers interrupt flag
    OC3RS = ((highTime*MICRO_TICK_CONV_N)/MICRO_TICK_CONV_D);
    amount++;
}

#ifdef IF_TESTING_RCS
#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"

int main(){
    BOARD_Init();
    LEDS_INIT();
    Protocol_Init();
    RCServo_Init();
     
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    
    unsigned int servoVal = 0;
    unsigned int currentVal = 0;
    while (1) {
        if (Protocol_IsMessageAvailable()) {
            if (Protocol_ReadNextID() == ID_COMMAND_SERVO_PULSE) {
                
                Protocol_GetPayload(&servoVal); //get value received for servo
                servoVal = Protocol_IntEndednessConversion(servoVal); //convert the short to proper endedness
                RCServo_SetPulse(servoVal);//set the new highTime servo
                
                currentVal = RCServo_GetPulse(); //get new highTime value
                currentVal = Protocol_IntEndednessConversion(currentVal); //convert to OS endedness
                Protocol_SendMessage(4, ID_SERVO_RESPONSE, &currentVal); //send the servo response with an unsigned int containing highTime
            }
        }
    }
}

#endif