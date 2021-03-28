#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "FeedbackControl.h"

#define MAX_CONTROL_OUTPUT (1<<FEEDBACK_MAXOUTPUT_POWER)

static int KP;
static int KI;
static int KD;

static int A; //integral accumulator
static int previousSensorValue;

/**
 * @Function FeedbackControl_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes the controller to the default values and (P,I,D)->(1, 0, 0)*/
int FeedbackControl_Init(void){
    KP = 0;
    KI = 0;
    KD = 0;
    A = 0;
    previousSensorValue = 0;
}

/**
 * @Function FeedbackControl_SetProportionalGain(int newGain);
 * @param newGain, integer proportional gain
 * @return SUCCESS or ERROR
 * @brief sets the new P gain for controller */
int FeedbackControl_SetProportionalGain(int newGain){
    KP = newGain;
    return SUCCESS;
}

/**
 * @Function FeedbackControl_SetIntegralGain(int newGain);
 * @param newGain, integer integral gain
 * @return SUCCESS or ERROR
 * @brief sets the new I gain for controller */
int FeedbackControl_SetIntegralGain(int newGain){
    KI = newGain;
    return SUCCESS;
}

/**
 * @Function FeedbackControl_SetDerivativeGain(int newGain);
 * @param newGain, integer derivative gain
 * @return SUCCESS or ERROR
 * @brief sets the new D gain for controller */
int FeedbackControl_SetDerivativeGain(int newGain){
    KD = newGain;
    return SUCCESS;
}

/**
 * @Function FeedbackControl_GetPorportionalGain(void)
 * @param None
 * @return Proportional Gain
 * @brief retrieves requested gain */
int FeedbackControl_GetProportionalGain(void){
    return KP;
}

/**
 * @Function FeedbackControl_GetIntegralGain(void)
 * @param None
 * @return Integral Gain
 * @brief retrieves requested gain */
int FeedbackControl_GetIntegralGain(void){
    return KI;
}

/**
 * @Function FeedbackControl_GetDerivativeGain(void)
 * @param None
 * @return Derivative Gain
 * @brief retrieves requested gain */
int FeedbackControl_GetDerivativeGain(void){
    return KD;
}

/**
 * @Function FeedbackControl_Update(int referenceValue, int sensorValue)
 * @param referenceValue, wanted reference
 * @param sensorValue, current sensor value
 * @brief performs feedback step according to algorithm in lab manual */
int FeedbackControl_Update(int referenceValue, int sensorValue){
    int error = referenceValue - sensorValue; //calculate error
    A += error; //integrate error
    int D = -1*(sensorValue - previousSensorValue); //numerical derivative
    previousSensorValue = sensorValue; //update previous sensor value
    
    int u = (KP*error)+(KI*A)+(KD*D); //compute control
    
    if(u > MAX_CONTROL_OUTPUT){
        u = MAX_CONTROL_OUTPUT; //clip control output
        A -= error; //anti-windup
        return u;
    }
    if(u < (-1*MAX_CONTROL_OUTPUT)){
        u = (-1*MAX_CONTROL_OUTPUT);//clip control output
        A -= error; //anti-windup
        return u;
    }
    
    return u;
}

/**
 * @Function FeedbackControl_ResetController(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief resets integrator and last sensor value to zero */
int FeedbackControl_ResetController(void){
    A = 0;
    previousSensorValue = 0;
}

//#define IF_TESTING_FBC

#ifdef IF_TESTING_FBC

typedef struct{
    int P;
    int I;
    int D;
}gainPacket;

typedef struct{
    int reference;
    int sensor;
}updatePacket;

#include "Protocol.h"
#include "MessageIDs.h"

int main(){
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    FreeRunningTimer_Init();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    gainPacket packetG;
    updatePacket packetU;
    int noReturn = 0;
    int feedbackOutput = 0;

    while (1){
        if (Protocol_IsMessageAvailable()) {
            if (Protocol_ReadNextID() == ID_FEEDBACK_SET_GAINS) {
                
                Protocol_GetPayload(&packetG);
                
                //setting gain values from packet
                packetG.P = Protocol_IntEndednessConversion(packetG.P);
                packetG.I = Protocol_IntEndednessConversion(packetG.I);
                packetG.D = Protocol_IntEndednessConversion(packetG.D);
                
                FeedbackControl_SetProportionalGain(packetG.P);
                FeedbackControl_SetIntegralGain(packetG.I);
                FeedbackControl_SetDerivativeGain(packetG.D);
                
                //setting packet from gain values
                packetG.P = FeedbackControl_GetProportionalGain();
                packetG.I = FeedbackControl_GetIntegralGain();
                packetG.D = FeedbackControl_GetDerivativeGain();
                
                packetG.P = Protocol_IntEndednessConversion(packetG.P);
                packetG.I = Protocol_IntEndednessConversion(packetG.I);
                packetG.D = Protocol_IntEndednessConversion(packetG.D);
                
                Protocol_SendMessage(12,ID_FEEDBACK_SET_GAINS_RESP,&packetG);
                
                
            }
            if (Protocol_ReadNextID() == ID_FEEDBACK_RESET_CONTROLLER) {
                Protocol_GetPayload(&noReturn);
                
                FeedbackControl_ResetController();
                Protocol_SendMessage(4,ID_FEEDBACK_RESET_CONTROLLER_RESP,&noReturn);
            }
            if (Protocol_ReadNextID() == ID_FEEDBACK_UPDATE) {
                
                Protocol_GetPayload(&packetU);
                
                packetU.reference = Protocol_IntEndednessConversion(packetU.reference);
                packetU.sensor = Protocol_IntEndednessConversion(packetU.sensor);
                
                feedbackOutput = FeedbackControl_Update(packetU.reference, packetU.sensor);
                feedbackOutput = Protocol_IntEndednessConversion(feedbackOutput);
                
                Protocol_SendMessage(4,ID_FEEDBACK_UPDATE_OUTPUT,&feedbackOutput);
            }
        }
    }
}
#endif


