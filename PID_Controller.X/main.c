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
#include "MessageIDs.h"
#include "FreeRunningTimer.h"
#include "DCMotorDrive.h"
#include "RotaryEncoder.h"
#include "FeedbackControl.h"
#include "ADCFilter.h"
#include "NonVolatileMemory.h"

#define MAX_RATE 6000 //6000 //max rate with a 1 millisecond update time
#define TICK_RATE 1
#define ENCODER_ROLLOVER 0x3FFF //(1<<14)

#define REPORT_RATE 5 //reports feedback at 5ms intervals (200Hz)

#define MAX_POT 1023
#define TOTAL_DEGREES 360
#define POT_DEGREE_RANGE 300

#define SHAFT_RATIO 84

#define SENSOR_MODE 1
#define COMMAND_MODE 0

#define STARTUP_DELAY 10 //10 ms startup delay

#define RANGE_LIMIT 150

#define GAIN_PAGE_ADDRESS 0x08 //NVM address to store the 6 integers
#define GAINS_LENGTH 24 //24 characters -> 6 integers

typedef struct{
    int P;
    int I;
    int D;
}gainPacket;

typedef struct{
    int error;
    int reference;
    int sensorVal;
    int commandVal;
}reportPacket;

static unsigned short previousAngle = 0;
static unsigned short currentAngle = 0;

static int encoderSpeed = 0;
static int accumulatedAngle = 0;

static int gains[] = {0,0,0,0,0,0}; //first 3 integers are mode 0, second 3 are mode 1

short lowPassFilter[] =  {-54, -64, -82, -97, -93, -47, 66, 266, 562, 
                                951, 1412, 1909, 2396, 2821, 3136, 3304, 
                                3304, 3136, 2821, 2396, 1909, 1412, 951, 
                                562, 266, 66, -47, -93, -97, -82, -64, -54};

//calculates motor speed
int AccumulatedAnglePosition(){
    previousAngle = currentAngle;
    currentAngle = RotaryEncoder_ReadRawAngle();
    
    encoderSpeed = currentAngle-previousAngle;
    if(encoderSpeed > MAX_RATE){
        encoderSpeed -= ENCODER_ROLLOVER;
        accumulatedAngle -= ENCODER_ROLLOVER;
    }
    else if(encoderSpeed < (-1*MAX_RATE)){
        encoderSpeed += ENCODER_ROLLOVER;
        accumulatedAngle += ENCODER_ROLLOVER;
    }
    return (accumulatedAngle+currentAngle);
}

//loads the current gains from NVM
void loadGainsNVM(){
    unsigned char* char_gains = (char*)gains; 
    NonVolatileMemory_ReadPage(GAIN_PAGE_ADDRESS, GAINS_LENGTH, char_gains);
}

//stores current gains array into NVM
void storeGainsNVM(){
    unsigned char* char_gains = (char*)gains; 
    NonVolatileMemory_WritePage(GAIN_PAGE_ADDRESS, GAINS_LENGTH, char_gains);
}

int main(){
    BOARD_Init();
    Protocol_Init();
    FreeRunningTimer_Init();
    DCMotorDrive_Init();
    RotaryEncoder_Init(ENCODER_INTERRUPT_MODE);
    FeedbackControl_Init();
    ADCFilter_Init();
    NonVolatileMemory_Init();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    int previousUpdateTime = FreeRunningTimer_GetMilliSeconds();
    int currentUpdateTime = FreeRunningTimer_GetMilliSeconds();
    
    int previousReportTime = FreeRunningTimer_GetMilliSeconds();
    int currentReportTime = FreeRunningTimer_GetMilliSeconds();
    
    ADCFilter_SetWeights(1, lowPassFilter); //setting the low pass filter weights to the pot pin (0)
    
    int rotaryEncoderPacket = 0;
    
    int reading = 0; //reading from potentiometer
    
    int curAngle = 0; //converted curPos to curAngle
    int curPos = 0; // current position received from AccumulatedAnglePosition
    int feedbackOutput = 0; //feedback output variable
    
    int sensorTargetAngle = 0; //sensor target angle -150 to 150
    int commandTargetAngle = 0; //command target angle -150 to 150
    
    int sensorTargetPos = 0; //sensor target angle converted into actual encoder measurements
    int commandTargetPos = 0; //sensor command angle converted into actual encoder measurements
    int curTargetPos = 0; //either sensor or target depending on mode
    
    gainPacket packetG; //contains PID gains
    reportPacket packetR;
    
    char mode = 1; //either sensor mode or command mode
    int noReturn = 0; //send when there is no message to return
    
    loadGainsNVM(); //loading gains from NVM
    
    //delay for encoder to self-calibrate
    int previousDelay = FreeRunningTimer_GetMilliSeconds();
    int currentDelay = FreeRunningTimer_GetMilliSeconds();
    while(currentDelay - previousDelay < STARTUP_DELAY){
        currentDelay = FreeRunningTimer_GetMilliSeconds();
    }
    
    FeedbackControl_SetProportionalGain(gains[3*mode]); //setting gains from current gains array loaded from NVM
    FeedbackControl_SetIntegralGain(gains[3*mode+1]);
    FeedbackControl_SetDerivativeGain(gains[3*mode+2]);
    
    while (1){
        currentUpdateTime = FreeRunningTimer_GetMilliSeconds();
        currentReportTime = FreeRunningTimer_GetMilliSeconds();
        
        if(currentUpdateTime - previousUpdateTime > TICK_RATE){ //every 1 milliseconds
            previousUpdateTime = currentUpdateTime;
            curPos = AccumulatedAnglePosition(); //updating motor speed
            curAngle = (curPos*TOTAL_DEGREES)/(SHAFT_RATIO*ENCODER_ROLLOVER);
        }
        
        if(currentReportTime - previousReportTime > REPORT_RATE){ //every 5 milliseconds
            previousReportTime = currentReportTime;
            
            if(mode == SENSOR_MODE){
                reading = ADCFilter_FilteredReading(1); //take low pass filtered reading
                sensorTargetAngle = (reading*POT_DEGREE_RANGE/MAX_POT) - POT_DEGREE_RANGE/2; //maps (0 to 1023) to (-150 to 150)
                sensorTargetPos = (sensorTargetAngle*SHAFT_RATIO*ENCODER_ROLLOVER)/TOTAL_DEGREES; //maps (0-300) to position of shaft
                curTargetPos = sensorTargetPos;
            }else{
                commandTargetPos = (commandTargetAngle*SHAFT_RATIO*ENCODER_ROLLOVER)/TOTAL_DEGREES; //maps (0-300) to position of shaft
                curTargetPos = commandTargetPos;
            }

            feedbackOutput = FeedbackControl_Update(curTargetPos, curPos);
            long long int transfer = feedbackOutput;
            transfer *= MAXMOTORSPEED;
            feedbackOutput = (transfer >> FEEDBACK_MAXOUTPUT_POWER);
            
            if((curAngle > RANGE_LIMIT) && (feedbackOutput > 0)){
                DCMotorDrive_SetBrake();
            }else if(curAngle < (-1*RANGE_LIMIT) && (feedbackOutput < 0)){
                DCMotorDrive_SetBrake();
            }else{
                DCMotorDrive_SetMotorSpeed(feedbackOutput);
            }
            
            //report back to main interface
            if(mode == SENSOR_MODE){
                packetR.error = curAngle - sensorTargetAngle;
                packetR.reference = sensorTargetAngle;
            }else{
                packetR.error = curAngle - commandTargetAngle;
                packetR.reference = commandTargetAngle;
            }
            packetR.commandVal = commandTargetAngle;
            packetR.sensorVal = sensorTargetAngle;
            
            packetR.error = Protocol_IntEndednessConversion(packetR.error);
            packetR.reference = Protocol_IntEndednessConversion(packetR.reference);
            packetR.commandVal = Protocol_IntEndednessConversion(packetR.commandVal);
            packetR.sensorVal = Protocol_IntEndednessConversion(packetR.sensorVal);
            
            Protocol_SendMessage(16,ID_LAB5_REPORT,&packetR);
        }
        
        if (Protocol_IsMessageAvailable()) {
            if (Protocol_ReadNextID() == ID_FEEDBACK_SET_GAINS) {
                
                Protocol_GetPayload(&packetG);
                
                //setting gain values from packet
                packetG.P = Protocol_IntEndednessConversion(packetG.P);
                packetG.I = Protocol_IntEndednessConversion(packetG.I);
                packetG.D = Protocol_IntEndednessConversion(packetG.D);
                
                //FeedbackControl_ResetController(); //reset the controller before setting new gains
                FeedbackControl_SetProportionalGain(packetG.P);
                FeedbackControl_SetIntegralGain(packetG.I);
                FeedbackControl_SetDerivativeGain(packetG.D);
                
                //copying new gains to their corresponding array index
                gains[3*mode] = packetG.P;
                gains[3*mode+1] = packetG.I;
                gains[3*mode+2] = packetG.D;
                
                //setting packet from gain values
                packetG.P = FeedbackControl_GetProportionalGain();
                packetG.I = FeedbackControl_GetIntegralGain();
                packetG.D = FeedbackControl_GetDerivativeGain();
                
                packetG.P = Protocol_IntEndednessConversion(packetG.P);
                packetG.I = Protocol_IntEndednessConversion(packetG.I);
                packetG.D = Protocol_IntEndednessConversion(packetG.D);
                
                Protocol_SendMessage(12,ID_FEEDBACK_SET_GAINS_RESP,&packetG);
                
                //storing new gains into NVM
                storeGainsNVM();
            }
            
            if (Protocol_ReadNextID() == ID_FEEDBACK_REQ_GAINS) {
                
                Protocol_GetPayload(&noReturn); //no payload
                
                //getting packet from gain values
                packetG.P = FeedbackControl_GetProportionalGain();
                packetG.I = FeedbackControl_GetIntegralGain();
                packetG.D = FeedbackControl_GetDerivativeGain();
                
                packetG.P = Protocol_IntEndednessConversion(packetG.P);
                packetG.I = Protocol_IntEndednessConversion(packetG.I);
                packetG.D = Protocol_IntEndednessConversion(packetG.D);
                
                Protocol_SendMessage(12,ID_FEEDBACK_CUR_GAINS,&packetG);
            }
            
            if (Protocol_ReadNextID() == ID_FEEDBACK_RESET_CONTROLLER) {
                Protocol_GetPayload(&noReturn);
                
                FeedbackControl_ResetController();
                Protocol_SendMessage(4,ID_FEEDBACK_RESET_CONTROLLER_RESP,&noReturn);
            }
            
            if (Protocol_ReadNextID() == ID_COMMANDED_POSITION) {
                
                Protocol_GetPayload(&commandTargetAngle);
                commandTargetAngle = Protocol_IntEndednessConversion(commandTargetAngle);
            }
            
            if (Protocol_ReadNextID() == ID_LAB5_SET_MODE) {
                Protocol_GetPayload(&mode);
                
                //set gains based on current mode
                FeedbackControl_SetProportionalGain(gains[3*mode]); 
                FeedbackControl_SetIntegralGain(gains[3*mode+1]);
                FeedbackControl_SetDerivativeGain(gains[3*mode+2]);
            }
            
            if (Protocol_ReadNextID() == ID_LAB5_REQ_MODE) {
                Protocol_GetPayload(&noReturn);
                Protocol_SendMessage(1,ID_LAB5_CUR_MODE,&mode);
            }
        }
    }
}