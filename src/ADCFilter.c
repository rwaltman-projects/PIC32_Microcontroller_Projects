#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "ADCFilter.h"

//#define IF_TESTING_ADC

#define AMOUNT_CHANNELS 4
#define A0_HEX 0x00000004
#define A1_HEX 0x00000010
#define A2_HEX 0x00000100
#define A3_HEX 0x00000400

typedef struct{
    short rawDatabuffer[AMOUNT_CHANNELS][FILTERLENGTH]; //stores filter data
    unsigned int rawDataInd[AMOUNT_CHANNELS]; //start index for each channel
    short filterBuffer[AMOUNT_CHANNELS][FILTERLENGTH]; //keeps track of filters for each channel
}ADCData;

static ADCData dataChannels;

/**
 * @Function ADCFilter_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes ADC system along with naive filters */
int ADCFilter_Init(void){
    AD1CON1bits.ON = 0; //disable ADC
    
    AD1PCFG = A0_HEX | A1_HEX | A2_HEX | A3_HEX; //enables A0-A3
    AD1CSSL = A0_HEX | A1_HEX | A2_HEX | A3_HEX; //adds scanner to analog pins
    
    AD1CON1bits.ASAM = 1; //auto sample
    AD1CON1bits.SSRC = 7; //auto convert
    AD1CON1bits.FORM = 0; //integer 16 bit mode
    
    AD1CON2bits.VCFG = 0; //internal reference voltage
    AD1CON2bits.CSCNA = 1;  //scan mode
    AD1CON2bits.SMPI = 3; //interrupt after sampling/conversion of 4 analog pins
    
    AD1CON2bits.BUFM = 0; //one 16-word buffer
    AD1CON3bits.ADRC = 0; //sets TAD to use PB clock
    
    AD1CON3bits.ADCS = 173; //ADC TAD to 348
    AD1CON3bits.SAMC = 16; //sample time TAD = 16
    
    IFS1bits.AD1IF = 0; //lowers interrupt flag
    IPC6bits.AD1IP = 2; //sets priority to 1
    IEC1bits.AD1IE = 1; //enable AD1 interrupt
    
    AD1CON1bits.ON = 1; //enable ADC
}

/**
 * @Function ADCFilter_RawReading(short pin)
 * @param pin, which channel to return
 * @return un-filtered AD Value
 * @brief returns current reading for desired channel */
short ADCFilter_RawReading(short pin){
    return dataChannels.rawDatabuffer[pin][dataChannels.rawDataInd[pin]];
}

/**
 * @Function ADCFilter_FilteredReading(short pin)
 * @param pin, which channel to return
 * @return Filtered AD Value
 * @brief returns filtered signal using weights loaded for that channel */
short ADCFilter_FilteredReading(short pin){
    short* values = dataChannels.rawDatabuffer[pin];
    short* filter = dataChannels.filterBuffer[pin];
    short startIndex = dataChannels.rawDataInd[pin];
    return ADCFilter_ApplyFilter(filter, values, startIndex);
}

/**
 * @Function short ADCFilter_ApplyFilter(short filter[], short values[], short startIndex)
 * @param filter, pointer to filter weights
 * @param values, pointer to circular buffer of values
 * @param startIndex, location of first sample so filter can be applied correctly
 * @return Filtered and Scaled Value
 * @brief returns final signal given the input arguments
 * @warning returns a short but internally calculated value should be an int */
short ADCFilter_ApplyFilter(short filter[], short values[], short startIndex){
    long long int val = 0;
    int filterInd;
    int start = startIndex;
    for(filterInd = 0; filterInd < FILTERLENGTH; filterInd++){
        val += (int)filter[filterInd]*(int)values[start];
        start--;
        if(start < 0){
            start = FILTERLENGTH-1;
        }
    }
    return ((val >> 15)& 0x0000FFFF);
}

/**
 * @Function ADCFilter_SetWeights(short pin, short weights[])
 * @param pin, which channel to return
 * @param pin, array of shorts to load into the filter for the channel
 * @return SUCCESS or ERROR
 * @brief loads new filter weights for selected channel */
int ADCFilter_SetWeights(short pin, short weights[]){
    if(pin < 0 || pin > 3){
        return ERROR; //pin is invalid
    }
    int i;
    for(i = 0; i < FILTERLENGTH; i++){
        dataChannels.filterBuffer[pin][i] = weights[i]; //copies data into filter buffer for channel (pin)
    }
    return SUCCESS;
}

void __ISR ( _ADC_VECTOR ) ADCIntHandler ( void ) {
    IFS1bits.AD1IF = 0; //lowers interrupt flag
    
    //adding data to the filter buffer
    dataChannels.rawDataInd[0] = ((dataChannels.rawDataInd[0] + 1) % FILTERLENGTH);
    dataChannels.rawDatabuffer[0][dataChannels.rawDataInd[0]] = ADC1BUF0;
    
    dataChannels.rawDataInd[1] = ((dataChannels.rawDataInd[1] + 1) % FILTERLENGTH);
    dataChannels.rawDatabuffer[1][dataChannels.rawDataInd[1]] = ADC1BUF1;
    
    dataChannels.rawDataInd[2] = ((dataChannels.rawDataInd[2] + 1) % FILTERLENGTH);
    dataChannels.rawDatabuffer[2][dataChannels.rawDataInd[2]] = ADC1BUF2;
    
    dataChannels.rawDataInd[3] = ((dataChannels.rawDataInd[3] + 1) % FILTERLENGTH);
    dataChannels.rawDatabuffer[3][dataChannels.rawDataInd[3]] = ADC1BUF3;
    
}

#ifdef IF_TESTING_ADC
#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"
#include "FrequencyGenerator.h"

typedef struct{
    short raw;
    short filtered;
}Packet;

int main(){
    BOARD_Init();
    LEDS_INIT();
    Protocol_Init();
    FreeRunningTimer_Init();
    FrequencyGenerator_Init();
    ADCFilter_Init();
    
    FrequencyGenerator_SetFrequency(DEFAULT_TONE);
    FrequencyGenerator_On();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    unsigned short channel = 0;
    
    unsigned short CurFrequency;
    unsigned char freqState;
    
    short receivedFilter[FILTERLENGTH];
    
    Packet packet;
    
    unsigned int previousTime = FreeRunningTimer_GetMilliSeconds();
    unsigned int currentTime = FreeRunningTimer_GetMilliSeconds();
    
    while (1){
        if (Protocol_IsMessageAvailable()){
            int messageID = Protocol_ReadNextID();
            if(messageID == ID_ADC_SELECT_CHANNEL){ //channel select
                Protocol_GetPayload(&channel);
                Protocol_SendMessage(1,ID_ADC_SELECT_CHANNEL_RESP, &channel);
            }
            if (Protocol_ReadNextID() == ID_LAB3_SET_FREQUENCY){ //frequency set
                Protocol_GetPayload(&CurFrequency);
                CurFrequency = Protocol_ShortEndednessConversion(CurFrequency);
                FrequencyGenerator_SetFrequency(CurFrequency);
            }
            if (Protocol_ReadNextID() == ID_LAB3_FREQUENCY_ONOFF){ //frequency enable
                Protocol_GetPayload(&freqState);
                if (freqState) {
                    FrequencyGenerator_On();
                } else {
                    FrequencyGenerator_Off();
                }
            }
            if (Protocol_ReadNextID() == ID_ADC_FILTER_VALUES){
                Protocol_GetPayload(&receivedFilter);
                int i;
                for(i = 0; i < FILTERLENGTH; i++){
                    receivedFilter[i] = Protocol_ShortEndednessConversion(receivedFilter[i]);
                }
                ADCFilter_SetWeights(channel, receivedFilter);
                Protocol_SendMessage(1,ID_ADC_FILTER_VALUES_RESP, &channel);
            }
        }
    
        currentTime = FreeRunningTimer_GetMilliSeconds();
        if(currentTime - previousTime > 10){ //10 millisecond delay
            previousTime = currentTime;
            packet.filtered = ADCFilter_FilteredReading(channel);
            packet.raw = ADCFilter_RawReading(channel);
            
            packet.filtered = Protocol_ShortEndednessConversion(packet.filtered);
            packet.raw = Protocol_ShortEndednessConversion(packet.raw);
            
            Protocol_SendMessage(4,ID_ADC_READING, &packet);
        }
    
    }
}

#endif

//#define SOFTWARE_TEST

#ifdef SOFTWARE_TEST

#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"

typedef struct{
    short raw;
    short filtered;
}Packet;


int main(){
    BOARD_Init();
    LEDS_INIT();
    Protocol_Init();
    FreeRunningTimer_Init();
    ADCFilter_Init();
    
    //low pass filter
    short lowFilter[] = {-54, -64, -82, -97, -93, -47, 66, 266, 562, 951, 1412, 1909, 2396, 2821, 3136, 3304, 3304, 3136, 2821, 2396, 1909, 1412, 951, 562, 266, 66, -47, -93, -97, -82, -64, -54};
    short highFilter[] = {0, 0, 39, -91, 139, -129, 0, 271, -609, 832, -696, 0, 1297, -3011, 4755, -6059, 6542, -6059, 4755, -3011, 1297, 0, -696, 832, -609, 271, 0, -129, 139, -91, 39, 0};
    short rawData[32];
    
    int i;
    for(i = 0; i < 32; i++){
        rawData[i] = 666;
    }
    
    char testMessage[MAXPAYLOADLENGTH];
    
    unsigned short rawDataInd = 0;
    
    Packet packet;
    
    unsigned int previousTime = FreeRunningTimer_GetMilliSeconds();
    unsigned int currentTime = FreeRunningTimer_GetMilliSeconds();
    while(1){
        
        currentTime = FreeRunningTimer_GetMilliSeconds();
        if(currentTime - previousTime > 10){ //10 millisecond delay
            packet.filtered = ADCFilter_ApplyFilter(lowFilter, rawData, rawDataInd);
            packet.raw = rawData[rawDataInd];
            rawDataInd = (rawDataInd+1)%32;
            
            previousTime = currentTime;
     
            sprintf(testMessage, "raw: %d, filtered: %d", packet.raw, packet.filtered);
            Protocol_SendDebugMessage(testMessage);
        }
    }
}
#endif