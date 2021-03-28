/* 
 * File:   Protocol.c
 * Author: Riley Altman
 *
 * Created on January 14, 2021
 */
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


//#define IF_TESTING
#define MAX_BUFFER_LENGTH 512
#define PACKET_BUFFER_LENGTH 16

/*******************************************************************************
 * RECEIVED PACKET STRUCT, ENUM, AND FUNCTION DECLARATIONS                     *
 ******************************************************************************/

typedef enum {
    IDLE, RECEIVE_LEN, RECEIVE_PAYLOAD, RECEIVE_TAIL, RECEIVE_CHECKSUM, RECEIVE_R, RECEIVE_N
} RXState;

typedef struct{
    unsigned char id;
    unsigned char len; //stores the length of the payload
    unsigned char checksum;
    unsigned char payload[MAXPAYLOADLENGTH];
    unsigned int curInd;
}packet;

typedef struct{
    packet buffer[PACKET_BUFFER_LENGTH];
    unsigned int head;
    unsigned int tail;
    unsigned int modFlag; //determines when the buffer is currently being modified.
}packetBuffer;

void resetPacket(packet* P);

int addPacket(packetBuffer* P, packet D);

int pullPacket(packetBuffer* P, packet* D);

int packetBufferIsEmpty(packetBuffer* P);

int packetBufferIsFull(packetBuffer* P);

unsigned int packetBufferGetLength(packetBuffer* P);

/*******************************************************************************
 * CIRCULAR BUFFER STRUCTS AND FUNCTION DECLARATIONS                           *
 ******************************************************************************/

typedef struct{
    unsigned int head;
    unsigned int tail;
    unsigned char data [MAX_BUFFER_LENGTH];
    unsigned int modFlag; //determines whether buffer is currently being modified
}circBuffer;

int circAddData(circBuffer* C, unsigned char inData);

char circPullData(circBuffer* C);

int circIsEmpty(circBuffer* C);

int circIsFull(circBuffer* C);

unsigned int circGetLength(circBuffer* C);

unsigned int circGetHead(circBuffer* C);

unsigned int circGetTail(circBuffer* C);

unsigned int circGetModFlag(circBuffer* C);

/*******************************************************************************
 * STATIC DATA                                                                 *
 ******************************************************************************/
static circBuffer TXBuffer;

static packetBuffer PBuffer;
static packet currentPacket;

static packet newPacket;
static unsigned char ledStates; //stores LED state when received

int IBFlag = 0; //Interrupted Buffer Flag: if the ISR interrupts during putChar this flag will go high

int TXFlag = 0; //goes high when there is currently something being transmitted through sendMessage functions
int ITXFlag = 0; //Interrupted Transmission Flag: goes high when sendMessage functions are interrupted
int errorFlag = 0; //if there is an error in receiving packets, goes high

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/
/**
 * @Function Protocol_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief 
 * @author mdunne */
int Protocol_Init(void){
    // we first clear the Configs Register to ensure a blank state and peripheral is off.
    U1MODE = 0;
    U1STA = 0; //clear control register

    //calculate brg
    int sourceClock = BOARD_GetPBClock() >> 3;
    int brg = sourceClock / 115200;
    brg++;
    brg >>= 1;
    brg--;
    U1BRG = brg;
    
    U1MODEbits.PDSEL = 0; //8 bit no parity 
    
    //setting up interrupt
    IPC6bits.U1IP = 1; //sets the ranking of the interrupt
    
    U1STAbits.UTXISEL = 2; //triggers when the transmit buffer becomes empty
    IEC0bits.U1TXIE = 1; //enables the transmit interrupt
    
    U1STAbits.URXISEL = 0; //triggers when the receives character.
    IEC0bits.U1RXIE = 1; //enables the receive interrupt
    
    U1MODEbits.UARTEN = 1; //enable UART1
    U1STAbits.UTXEN = 1; //enable transmission
    U1STAbits.URXEN = 1; //enable reception
}

/**
 * @Function int Protocol_SendMessage(unsigned char len, void *Payload)
 * @param len, length of full <b>Payload</b> variable
 * @param Payload, pointer to data, will be copied in during the function
 * @return SUCCESS or ERROR
 * @brief 
 * @author mdunne */
int Protocol_SendMessage(unsigned char len, unsigned char ID, void *Payload){
    TXFlag = 1;
    unsigned char* payload = (char *) Payload; //might need to convert to unsigned char*
    //making the payload
    PutChar(HEAD);
    PutChar(len+0x01); //adds the ID to the payload
    
    unsigned char checksum = 0x00;
    checksum = Protocol_CalcIterativeChecksum(ID, checksum);
    PutChar(ID);
    
    //enqueue the data
    int i;
    for(i = 0; i < len; i++){
        unsigned char next = payload[i];
        checksum = Protocol_CalcIterativeChecksum(next, checksum);
        PutChar(next);
    }
    
    PutChar(TAIL);
    PutChar(checksum);
    PutChar('\r');
    PutChar('\n');
    TXFlag = 0;
    if(ITXFlag){ //there was an LED_GET message that didn't send
        ITXFlag = 0;
        Protocol_SendMessage(1, ID_LEDS_STATE, &ledStates);
    }
    return SUCCESS;
}

/**
 * @Function int Protocol_SendDebugMessage(char *Message)
 * @param Message, Proper C string to send out
 * @return SUCCESS or ERROR
 * @brief Takes in a proper C-formatted string and sends it out using ID_DEBUG
 * @warning this takes an array, do <b>NOT</b> call sprintf as an argument.
 * @author mdunne */
int Protocol_SendDebugMessage(char *Message){
    TXFlag = 1;
    //making the payload
    
    if(strlen(Message) > MAXPAYLOADLENGTH){ //if the size of the message is larger than max payload
        return ERROR;
    }
    
    PutChar(HEAD);
    PutChar(strlen(Message)+0x01); //adds the ID to the payload
    
    unsigned char checksum = 0x00;
    checksum = Protocol_CalcIterativeChecksum(ID_DEBUG, checksum);
    PutChar(ID_DEBUG);
    
    //enqueue the data
    int i;
    for(i = 0; i < strlen(Message); i++){
        unsigned char next = Message[i];
        checksum = Protocol_CalcIterativeChecksum(next, checksum);
        PutChar(next);
    }
    
    PutChar(TAIL);
    PutChar(checksum);
    PutChar('\r');
    PutChar('\n');
    TXFlag = 0;
    if(ITXFlag){ //there was an LED_GET message that didn't send
        ITXFlag = 0;
        Protocol_SendMessage(1, ID_LEDS_STATE, &ledStates);
    }
    return SUCCESS;
}

/**
 * @Function unsigned char Protocol_ReadNextID(void)
 * @param None
 * @return Reads ID of next Packet
 * @brief Returns ID_INVALID if no packets are available
 * @author mdunne */
unsigned char Protocol_ReadNextID(void){
    if(packetBufferIsEmpty(&PBuffer)){ //if there are no packets in packet buffer
        errorFlag = 1;
        return ID_INVALID;
    }
    pullPacket(&PBuffer, &currentPacket);
    return currentPacket.id;
}

/**
 * @Function int Protocol_GetPayload(void* payload)
 * @param payload, Memory location to put payload
 * @return SUCCESS or ERROR
 * @brief this function is to be called after ReadNextID and pulls that packets data off it
 * @author mdunne */
int Protocol_GetPayload(void* payload){
    int i;
    
    int size = sizeof((char*)payload); //size in bytes (max should be 4 bytes - 128 bits)
    
    if(currentPacket.id == ID_INVALID){ //when ReadNextID is called it pulls the next packet off buffer
        errorFlag = 1;                  //ReadNextId should be called before this function
        return ERROR;                   //if that pulled packet has ID_INVALID message returns error
    }
    
    if(size < currentPacket.len - 1){ //length - 1 because id is included in payload
        errorFlag = 1;
        return ERROR;
    }
    
    for(i = 1; i < currentPacket.len; i++){
        ((char*)payload)[i-1] = currentPacket.payload[i];
    }
    return SUCCESS;
}

/**
 * @Function char Protocol_IsMessageAvailable(void)
 * @param None
 * @return TRUE if Queue is not Empty
 * @brief 
 * @author mdunne */
char Protocol_IsMessageAvailable(void){
    if(!packetBufferIsEmpty(&PBuffer)){
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function char Protocol_IsQueueFull(void)
 * @param None
 * @return TRUE is QUEUE is Full
 * @brief 
 * @author mdunne */
char Protocol_IsQueueFull(void){
    if(packetBufferIsFull(&PBuffer)){
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function char Protocol_IsError(void)
 * @param None
 * @return TRUE if error
 * @brief Returns if error has occurred in processing, clears on read
 * if there error flag is high, it returns true and error flag is reset
 * @author mdunne */
char Protocol_IsError(void){
    if(errorFlag){
        errorFlag = 0;
        return TRUE;
    }
    return FALSE;
}

/**
 * @Function char Protocol_ShortEndednessConversion(unsigned short inVariable)
 * @param inVariable, short to convert endedness
 * @return converted short
 * @brief Converts endedness of a short. This is a bi-directional operation so only one function is needed
 * @author mdunne */
unsigned short Protocol_ShortEndednessConversion(unsigned short inVariable){
    return (inVariable >> 8) | (inVariable << 8);
}

/**
 * @Function char Protocol_IntEndednessConversion(unsigned int inVariable)
 * @param inVariable, int to convert endedness
 * @return converted short
 * @brief Converts endedness of a int. This is a bi-directional operation so only one function is needed
 * @author mdunne */
unsigned int Protocol_IntEndednessConversion(unsigned int inVariable){
    unsigned short top = inVariable >> 16;
    unsigned short bot = inVariable;
    
    top = Protocol_ShortEndednessConversion(top);
    bot = Protocol_ShortEndednessConversion(bot);
    
    unsigned int newB = bot;
    unsigned int newT = top;
    
    return (newB << 16) | newT;
}

/**
 * @Function char Protocol_CalcIterativeChecksum(unsigned char charIn, unsigned char curChecksum)
 * @param charIn, new char to add to the checksum
 * @param curChecksum, current checksum, most likely the last return of this function, can use 0 to reset
 * @return the new checksum value
 * @brief Returns the BSD checksum of the char stream given the curChecksum and the new char
 * @author mdunne */
unsigned char Protocol_CalcIterativeChecksum(unsigned char charIn, unsigned char curChecksum){
    curChecksum = (curChecksum >> 1) + (curChecksum << 7);
    curChecksum += charIn;
    return curChecksum;
}

/**
 * @Function void Protocol_runReceiveStateMachine(unsigned char charIn)
 * @param charIn, next character to process
 * @return None
 * @brief Runs the protocol state machine for receiving characters, it should be called from 
 * within the interrupt and process the current character
 * @author mdunne */
void Protocol_RunReceiveStateMachine(unsigned char charIn){
    static RXState currentState = IDLE; //begin the state machine in idle state
    switch (currentState) {
        case IDLE:
            if(charIn == HEAD){
                currentState = RECEIVE_LEN;
            }else{
                errorFlag = 1; //something other than head was received
            }
            break;
        case RECEIVE_LEN:
            newPacket.len = charIn;
            currentState = RECEIVE_PAYLOAD;
            break;
        case RECEIVE_PAYLOAD:
            newPacket.payload[newPacket.curInd] = charIn; //stores next char into packets payload
            if(newPacket.curInd == 0){
                newPacket.id = charIn;
            }
            newPacket.curInd++;
            newPacket.checksum = Protocol_CalcIterativeChecksum(charIn, newPacket.checksum); //update checksum
            if(newPacket.curInd == newPacket.len){ //when we received amount of characters specified in len got to next state
                currentState = RECEIVE_TAIL;
            }
            break;
        case RECEIVE_TAIL: 
            if(charIn == TAIL){ //next character is the TAIL
                currentState = RECEIVE_CHECKSUM;
            }else{  //character was not a tail, reset the packet and go back to idle
                currentState = IDLE;
                errorFlag = 1;
                resetPacket(&newPacket);
            }
            break;
        case RECEIVE_CHECKSUM:
            if(charIn == newPacket.checksum){
                currentState = RECEIVE_R;
            }else{
                currentState = IDLE;
                errorFlag = 1;
                resetPacket(&newPacket);
            }
            break;
        case RECEIVE_R:
            if(charIn == '\r'){
                currentState = RECEIVE_N;
            }else{
                currentState = IDLE;
                errorFlag = 1;
                resetPacket(&newPacket);
            }
            break;
        case RECEIVE_N:
            if(charIn == '\n'){
                //the packet is valid
                if(newPacket.id == ID_LEDS_SET){
                    LEDS_SET(newPacket.payload[1]);
                }else if(newPacket.id == ID_LEDS_GET){
                    ledStates = LEDS_GET();
                    if(TXFlag){ //currently transmitting message, wait to send LED states
                        ITXFlag = 1; //set interrupted transmission flag high
                    }else{//not currently transmitting message, so send the LED states
                        Protocol_SendMessage(1, ID_LEDS_STATE, &ledStates);
                    }
                }else{
                    addPacket(&PBuffer, newPacket); //adds the valid packet to the packet buffer
                }
            }else{
                errorFlag = 1; //didn't receive proper end character
            }
            currentState = IDLE; //finished, go back to IDLE
            resetPacket(&newPacket);
            break;
    }
}

/**
 * @Function char PutChar(char ch)
 * @param ch, new char to add to the circular buffer
 * @return SUCCESS or ERROR
 * @brief adds to circular buffer if space exists, if not returns ERROR
 * @author mdunne */
int PutChar(char ch){
    int error = circAddData(&TXBuffer, ch);
    if(error == ERROR){ //if the transmit buffer is full
        errorFlag = 1;
        return ERROR;
    }
    if(U1STAbits.TRMT){ //UART is idle
        IFS0bits.U1TXIF = 1; //force interrupt
    }
    return SUCCESS;
}

/*******************************************************************************
 *  RX and TX ISR
 ******************************************************************************/
void __ISR(_UART1_VECTOR) IntUart1Handler(void){
    if(IFS0bits.U1TXIF == 1){ //transmit interrupt
        IFS0bits.U1TXIF = 0; //clearing the interrupt flag
        if(TXBuffer.modFlag == 1){//buffer is currently being modified by another function
            IBFlag = 1;
            return;
        }
        if(!circIsEmpty(&TXBuffer)){ //if the buffer isn't empty
            U1TXREG = circPullData(&TXBuffer); //puts the pulled data from TXBuffer into transmit buffer
        }
    }
    if(IFS0bits.U1RXIF == 1){ //receive interrupt
        IFS0bits.U1RXIF = 0;
        Protocol_RunReceiveStateMachine(U1RXREG & 0x00FF);
    }
}
/*******************************************************************************
 *  PRIVATE FUNCTIONS FOR PACKET
 ******************************************************************************/
/**
 * @Function void resetPacket(packet* P)
 * @param P, packet pointer to be reset
 * @return SUCCESS or ERROR
 * @brief resets the packet P with 0. Used when a packet is pulled from the buffer
 * @author Riley Altman */
void resetPacket(packet* P){
    P->curInd = 0;
    P->id = ID_INVALID;
    P->len = 0;
    P->checksum = 0;
    int i;
    for(i = 0; i < MAXPAYLOADLENGTH; i++){
        P->payload[i] = 0x00;
    }
}
/**
 * @Function int addPacket(packetBuffer* P, packet D)
 * @param P, packet buffer, D, packet to add to buffer
 * @return SUCCESS or ERROR
 * @brief adds packet D into the packet buffer P
 * @author Riley Altman */
int addPacket(packetBuffer* P, packet D){
    P->modFlag = 1;
    if(!packetBufferIsFull(P)){
        //copying over packet data
        P->buffer[P->tail].checksum = D.checksum ;
        P->buffer[P->tail].curInd = D.curInd;
        P->buffer[P->tail].id = D.id;
        P->buffer[P->tail].len = D.len;
        int i;
        for(i = 0; i < D.len; i++){
            P->buffer[P->tail].payload[i] = D.payload[i]; 
        }
               
        //updating head and tail
        P->tail = (P->tail + 1) % PACKET_BUFFER_LENGTH ;
        P->modFlag = 0;
//        if(IBFlag){ //interrupt was triggered during the previous operations
//            IBFlag = 0;
//            IFS0bits.U1TXIF = 1; //force interrupt
//        }
        return SUCCESS; //success
    }
    P->modFlag = 0;
//    if(IBFlag){ //interrupt was triggered during the previous operations
//        IBFlag = 0;
//        IFS0bits.U1TXIF = 1; //force interrupt
//    }
    return ERROR; //failed to add new data
}

/**
 * @Function int pullPacket(packetBuffer* P, packet* D)
 * @param P, packet buffer, D, packet to add to buffer
 * @return SUCCESS or ERROR
 * @brief puts the next packet from the packet buffer into the static D packet
 * @author Riley Altman */
int pullPacket(packetBuffer* P, packet* D){
    P->modFlag = 1;
    if(!packetBufferIsEmpty(P)){
        //copying over data
        D->checksum = P->buffer[P->head].checksum;
        D->curInd = P->buffer[P->head].curInd;
        D->id = P->buffer[P->head].id;
        D->len = P->buffer[P->head].len;
        int i;
        for(i = 0; i < P->buffer[P->head].len; i++){
             D->payload[i] = P->buffer[P->head].payload[i]; 
        }
        resetPacket(&(P->buffer[P->head])); //resets the packet after it is pulled
        P->head = (P->head + 1) % PACKET_BUFFER_LENGTH ;
        P->modFlag = 0;
        return SUCCESS;
    }
    P->modFlag = 0;
    return ERROR;
}

/**
 * @Function int packetBufferIsEmpty(packetBuffer* P)
 * @param P, packet buffer
 * @return SUCCESS or ERROR
 * @brief checks to see if the packet buffer is empty
 * @author Riley Altman */
int packetBufferIsEmpty(packetBuffer* P){
    if(P->head == P->tail){
        return 1;
    }
    return 0;
}

/**
 * @Function int packetBufferIsFull(packetBuffer* P)
 * @param P, packet buffer
 * @return SUCCESS or ERROR
 * @brief checks to see if the packet buffer is full
 * @author Riley Altman */
int packetBufferIsFull(packetBuffer* P){
    if((P->tail + 1) % PACKET_BUFFER_LENGTH == P->head){
        return 1;
    }
    return 0;
}

/**
 * @Function unsigned int packetBufferGetLength(packetBuffer* P)
 * @param P, packet buffer
 * @return size of packet buffer
 * @brief gets the length of the packet buffer
 * @author Riley Altman */
unsigned int packetBufferGetLength(packetBuffer* P){
    if(P->tail >= P->head){
        return (P->tail - P->head);
    }
    return PACKET_BUFFER_LENGTH + (P->tail - P->head);
}

/*******************************************************************************
 *  PRIVATE FUNCTIONS FOR CIRCULAR BUFFER
 ******************************************************************************/

/**
 * @Function int circAddData(circBuffer* C, unsigned char inData)
 * @param C, packet buffer, inData, data to be added to the buffer
 * @return SUCCESS or ERROR
 * @brief adds the given char inData to the circular buffer
 * @author Riley Altman */
int circAddData(circBuffer* C, unsigned char inData){
    C->modFlag = 1;
    if(!circIsFull(C)){
        C->data[C->tail] = inData ;
        C->tail = (C->tail + 1) % MAX_BUFFER_LENGTH ;
        C->modFlag = 0;
        if(IBFlag){ //interrupt was triggered during the previous operations
            IBFlag = 0;
            IFS0bits.U1TXIF = 1; //force interrupt
        }
        return SUCCESS; //success
    }
    C->modFlag = 0;
    if(IBFlag){ //interrupt was triggered during the previous operations
        IBFlag = 0;
        IFS0bits.U1TXIF = 1; //force interrupt
    }
    return ERROR; //failed to add new data
}

/**
 * @Function char circPullData(circBuffer* C)
 * @param C, pointer to buffer
 * @return char that was pulled off the buffer
 * @brief pulls a char off the buffer that was in the tail position
 * @author Riley Altman */
char circPullData(circBuffer* C){
    C->modFlag = 1;
    if(!circIsEmpty(C)){
        char pulled = C->data[C->head];
        C->head = (C->head + 1) % MAX_BUFFER_LENGTH ;
        C->modFlag = 0;
        if(IBFlag){ //interrupt was triggered during the previous operations
            IBFlag = 0;
            IFS0bits.U1TXIF = 1; //force interrupt
        }
        return pulled;
    }
    C->modFlag = 0;
    if(IBFlag){ //interrupt was triggered during the previous operations
        IBFlag = 0;
        IFS0bits.U1TXIF = 1; //force interrupt
    }
    return ERROR;
    
}

/**
 * @Function int circIsEmpty(circBuffer* C)
 * @param C, pointer to buffer
 * @return 1 or 0
 * @brief if the buffer is empty, returns 1, if the buffer isn't empty, returns 1
 * @author Riley Altman */
int circIsEmpty(circBuffer* C){
    if(C->head == C->tail){
        return 1;
    }
    return 0;
}

/**
 * @Function int circIsFull(circBuffer* C)
 * @param C, pointer to buffer
 * @return 1 or 0
 * @brief returns 1 if the buffer is full, 0 is the buffer is not full
 * @author Riley Altman */
int circIsFull(circBuffer* C){
    if((C->tail + 1) % MAX_BUFFER_LENGTH == C->head){
        return 1;
    }
    return 0;
}

/**
 * @Function unsigned int circGetLength(circBuffer* C)
 * @param C, circular buffer
 * @return size of the circular buffer
 * @brief gets the length of the packet buffer
 * @author Riley Altman */
unsigned int circGetLength(circBuffer* C){
    if(C->tail >= C->head){
        return (C->tail - C->head);
    }
    return MAX_BUFFER_LENGTH + (C->tail - C->head);
}

#ifdef IF_TESTING

int main(){
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);
    
    short shortTestValue = 0xDEAD;
    short shortResultValue;
    int intTestValue = 0xDEADBEEF;
    int intResultValue;
    
    shortResultValue = Protocol_ShortEndednessConversion(shortTestValue);
    sprintf(testMessage, "Short Endedness Conversion: IN: 0x%X OUT: 0x%X", shortTestValue&0xFFFF, shortResultValue&0xFFFF);
    Protocol_SendDebugMessage(testMessage);
    
    
    intResultValue = Protocol_IntEndednessConversion(intTestValue);
    sprintf(testMessage, "Int Endedness Conversion: IN: 0x%X OUT: 0x%X", intTestValue, intResultValue);
    Protocol_SendDebugMessage(testMessage);
    
    
    unsigned int pingValue = 0xfff;
    while (1) {
        if (Protocol_IsMessageAvailable()) {
            if (Protocol_ReadNextID() == ID_PING) {
                // send pong in response here
                Protocol_GetPayload(&pingValue);
                pingValue = Protocol_IntEndednessConversion(pingValue);
                pingValue>>=1;
                pingValue = Protocol_IntEndednessConversion(pingValue);
                Protocol_SendMessage(4, ID_PONG, &pingValue);
            }
        }
    }
    while (1);
}

#endif


