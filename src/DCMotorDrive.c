#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "DCMotorDrive.h"

#define FORWARD 1
#define BACKWARD 0
#define DRIVER_PINS_INIT() do { TRISDbits.TRISD9 = 0; TRISDbits.TRISD10 = 0; } while (0) //pin 7 -> IN1, pin 8 -> IN2
#define DRIVER_PINS_DRIVE(direction) do { LATDbits.LATD9 = direction; LATDbits.LATD10 = !(direction); } while (0)
#define DRIVER_PINS_BRAKE() do { LATDbits.LATD9 = 0; LATDbits.LATD10 = 0; } while (0)
#define DRIVER_PIN_GET_DIRECTION() (LATDbits.LATD9)

#define PWM_FREQUENCY 0x4E20 //ticks given 1:1 prescaller for 2KHz frequnecy
#define MAX_DUTY_CYCLE PWM_FREQUENCY
#define STOP_DUTY_CYCLE 0

static int motorSpeed = 0;
/**
 * @Function DCMotorDrive_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes timer3 to 2Khz and set up the pins
 * @warning you will need 3 pins to correctly drive the motor  */
int DCMotorDrive_Init(void){
    //configuring timer 3
    T3CON = 0x0;//clearing the control bits, also turns timer off
    
    T3CONbits.TCKPS = 0; //timer pre-scaler 1:1, 25ns between ticks
    TMR3 = 0; //reset the timer bits
    PR3 = PWM_FREQUENCY; //sets the period match to 20,000 ticks which is in total 0.5ms -> 2KHz
    
    OC3CONbits.ON = 0; //disable OC3
    OC3R = STOP_DUTY_CYCLE; //sets initial duty cycle
    OC3RS = STOP_DUTY_CYCLE; //sets initial duty cycle to be loaded on rollover events
    
    
    OC3CONbits.OCTSEL = 1; //sets the timer to timer 3
    OC3CONbits.OCM = 6; //sets to PWM mode, disabled fault
    
    DRIVER_PINS_INIT();
    DRIVER_PINS_DRIVE(FORWARD);
    OC3CONbits.ON = 1; //enables OC3
    T3CONbits.ON = 1; //enables Timer 3
}


/**
 * @Function DCMotorDrive_SetMotorSpeed(int newMotorSpeed)
 * @param newMotorSpeed, in units of Duty Cycle (+/- 1000)
 * @return SUCCESS or ERROR
 * @brief Sets the new duty cycle for the motor, 0%->0, 100%->1000 */
int DCMotorDrive_SetMotorSpeed(int newMotorSpeed){
    if(newMotorSpeed > MAXMOTORSPEED || newMotorSpeed < (-1*MAXMOTORSPEED)){ //speed out of range
        return ERROR;
    }
    
    motorSpeed = newMotorSpeed;
    
    if(newMotorSpeed < 0){ //direction is backwards
        DRIVER_PINS_DRIVE(BACKWARD);
        newMotorSpeed *= -1;
    }else{//direction is forwards
        DRIVER_PINS_DRIVE(FORWARD);
    }
    
    OC3RS = MAX_DUTY_CYCLE*newMotorSpeed/MAXMOTORSPEED; //convert motor speed to ticks

    return SUCCESS;
}


/**
 * @Function DCMotorControl_GetMotorSpeed(void)
 * @param None
 * @return duty cycle of motor 
 * @brief returns speed in units of Duty Cycle (+/- 1000) */
int DCMotorControl_GetMotorSpeed(void){
    return motorSpeed;
}


/**
 * @Function DCMotorDrive_SetBrake(void)
 * @param None
 * @return SUCCESS or FAILURE
 * @brief set the brake on the motor for faster stop */
int DCMotorDrive_SetBrake(void){
    DRIVER_PINS_BRAKE();
    return SUCCESS;
}


//#define IF_TESTING_MD

#ifdef IF_TESTING_MD
#include "Protocol.h"
#include "MessageIDs.h"

int main(){
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    FreeRunningTimer_Init();
    DCMotorDrive_Init();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    int receivedSpeed = 0;
    
    while (1){
        if (Protocol_IsMessageAvailable()) {
            if (Protocol_ReadNextID() == ID_COMMAND_OPEN_MOTOR_SPEED) {
                Protocol_GetPayload(&receivedSpeed);
                receivedSpeed = Protocol_IntEndednessConversion(receivedSpeed);
                
                DCMotorDrive_SetMotorSpeed(receivedSpeed);
                sprintf(testMessage, "received speed: %d, OC3RS: %X", receivedSpeed, OC3RS);
                Protocol_SendDebugMessage(testMessage);
                
            }
        }
        
    }
}

#endif

