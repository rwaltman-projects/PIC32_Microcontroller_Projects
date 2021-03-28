#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <GenericTypeDefs.h>
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx340f512h.h>

#include "BOARD.h"
#include "NonVolatileMemory.h"

//#define IF_TESTING_NVM

#define DEV_ADDRESS 0x50 //7 bit address of EEPROM device
#define WRITE_BIT 0x00 
#define READ_BIT 0x01

#define WRITE_ADDRESS (DEV_ADDRESS << 1) | WRITE_BIT
#define READ_ADDRESS (DEV_ADDRESS << 1) | READ_BIT

/**
 * @Function NonVolatileMemory_Init(void)
 * @param None
 * @return SUCCESS or ERROR
 * @brief initializes I2C for usage */
int NonVolatileMemory_Init(void){
    I2C1CONbits.ON = 0;
    I2C1BRG = 0x00C5;
    I2C1CONbits.ON = 1;
}

/**
 * @Function NonVolatileMemory_ReadByte(int address)
 * @param address, device address to read from
 * @return value at said address
 * @brief reads one byte from device
 * @warning Default value for this EEPROM is 0xFF */
unsigned char NonVolatileMemory_ReadByte(int address){
    unsigned char highByte = (address & 0xFF00) >> 8;
    unsigned char lowByte = address & 0x00FF;
    unsigned char recievedByte = 0x00;
    //start message
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1); 
    
    //writing control byte (EEPROM address and r/w bit - reading bit = 0)
    I2C1TRN = WRITE_ADDRESS; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    //sending address high byte
    I2C1TRN = highByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //sending address low byte
    I2C1TRN = lowByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //repeated start
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1); 
    
    //writing control byte (EEPROM address and r/w bit - reading bit = 1)
    I2C1TRN = READ_ADDRESS; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    I2C1CONbits.RCEN = 1;
    while(I2C1STATbits.RBF == 0); //wait for received
    recievedByte = I2C1RCV; //copy data from receive buffer
    
    I2C1CONbits.ACKDT = 1; //set ACK data to NACK
    I2C1CONbits.ACKEN = 1; //send NACK
    while(I2C1CONbits.ACKEN == 1);
    
    I2C1CONbits.PEN = 1; //sending stop signal
    while(I2C1CONbits.PEN == 1); 
    
    return recievedByte;
}

/**
 * @Function char NonVolatileMemory_WriteByte(int address, unsigned char data)
 * @param address, device address to write to
 * @param data, value to write at said address
 * @return SUCCESS or ERROR
 * @brief writes one byte to device */
char NonVolatileMemory_WriteByte(int address, unsigned char data){
    unsigned char highByte = (address & 0xFF00) >> 8;
    unsigned char lowByte = address & 0x00FF;
    
    //start message
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1); 
    
    //writing control byte (EEPROM address and r/w bit - reading bit = 0)
    I2C1TRN = WRITE_ADDRESS; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    //sending address high byte
    I2C1TRN = highByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //sending address low byte
    I2C1TRN = lowByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //sending data to be stored
    I2C1TRN = data; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    I2C1CONbits.PEN = 1; //sending stop signal
    while(I2C1CONbits.PEN == 1); 
    
    return SUCCESS;
}

/**
 * @Function int NonVolatileMemory_ReadPage(int page, char length, unsigned char data[])
 * @param page, page value to read from
 * @param length, value between 1 and 64 bytes to read
 * @param data, array to store values into
 * @return SUCCESS or ERROR
 * @brief reads bytes in page mode, up to 64 at once
 * @warning Default value for this EEPROM is 0xFF */
int NonVolatileMemory_ReadPage(int page, char length, unsigned char data[]){
    page = page << 6; //shifts the page to a valid address, each page has 64 bytes
    unsigned char highByte = (page & 0xFF00) >> 8;
    unsigned char lowByte = page & 0x00FF;
    int i;
    
    //start message
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1); 
    
    //writing control byte (EEPROM address and r/w bit - reading bit = 0)
    I2C1TRN = WRITE_ADDRESS; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    //sending address high byte
    I2C1TRN = highByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //sending address low byte
    I2C1TRN = lowByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //repeated start
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1); 
    
    //writing control byte (EEPROM address and r/w bit - reading bit = 1)
    I2C1TRN = READ_ADDRESS; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    //collect data from 0 to length - 1
    for(i = 0; i < length - 1; i++){
        I2C1CONbits.RCEN = 1;
        while(I2C1STATbits.RBF == 0); //wait for received
        data[i] = I2C1RCV; //copy data from receive buffer
        
        I2C1CONbits.ACKDT = 0; //set ACK data to ACK
        I2C1CONbits.ACKEN = 1; //send ACK
        while(I2C1CONbits.ACKEN == 1);
    }
    
    //collect the last byte
    I2C1CONbits.RCEN = 1;
    while(I2C1STATbits.RBF == 0); 
    data[i] = I2C1RCV; 
        
    //send NACK
    I2C1CONbits.ACKDT = 1;
    I2C1CONbits.ACKEN = 1;
    while(I2C1CONbits.ACKEN == 1);
    
    I2C1CONbits.PEN = 1; //sending stop signal
    while(I2C1CONbits.PEN == 1); 
    
    return SUCCESS;
}

/**
 * @Function char int NonVolatileMemory_WritePage(int page, char length, unsigned char data[])
 * @param address, device address to write to
 * @param data, value to write at said address
 * @return SUCCESS or ERROR
 * @brief writes one byte to device */
int NonVolatileMemory_WritePage(int page, char length, unsigned char data[]){
    page = page << 6; //shifts the page to a valid address, each page has 64 bytes
    unsigned char highByte = (page & 0xFF00) >> 8;
    unsigned char lowByte = page & 0x00FF;
    int i;
    //start message
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1); 
    
    //writing control byte (EEPROM address and r/w bit - reading bit = 0)
    I2C1TRN = WRITE_ADDRESS; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    
    //sending address high byte
    I2C1TRN = highByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //sending address low byte
    I2C1TRN = lowByte; 
    while(I2C1STATbits.TRSTAT == 1);
    while(I2C1STATbits.ACKSTAT == 1);
    
    //sending data to be stored
    for(i = 0; i < length; i++){
        I2C1TRN = data[i]; 
        while(I2C1STATbits.TRSTAT == 1);
        while(I2C1STATbits.ACKSTAT == 1); //receive ACK
    }

    I2C1CONbits.PEN = 1; //sending stop signal
    while(I2C1CONbits.PEN == 1); 
    
    return SUCCESS;
}



#ifdef IF_TESTING_NVM
#include "Protocol.h"
#include "MessageIDs.h"
#include "FreeRunningTimer.h"

typedef struct{
    unsigned int address;
    unsigned char data;
}writeBytePacket;

typedef struct{
    unsigned int address;
    unsigned char data[64];
}writePagePacket;

int main(){
    BOARD_Init();
    LEDS_INIT();
    Protocol_Init();
    FreeRunningTimer_Init();
    NonVolatileMemory_Init();
    
    unsigned int previous;
    unsigned int current;
    
    
    char testMessage[MAXPAYLOADLENGTH];
    sprintf(testMessage, "Protocol Test Compiled at %s %s", __DATE__, __TIME__);
    Protocol_SendDebugMessage(testMessage);

    unsigned char data[64];
    unsigned int address = 0;
    
    unsigned char tempChar = 0x00;
    
    writeBytePacket writeBytePayload;
    writePagePacket writePagePayload;
    
    previous = FreeRunningTimer_GetMilliSeconds();
    while (1){
        current = FreeRunningTimer_GetMilliSeconds();
        if(current-previous > 10){ //ensures that messages aren't read to quickly, leaves time for the pages to update 
            previous = FreeRunningTimer_GetMilliSeconds();
            if (Protocol_IsMessageAvailable()) {
                int messageID = Protocol_ReadNextID();
                
                if(messageID == ID_NVM_READ_BYTE){
                    
                    address = 0;
                    Protocol_GetPayload(&address);
                    address = Protocol_IntEndednessConversion(address);
                    tempChar = NonVolatileMemory_ReadByte(address);
                    Protocol_SendMessage(1,ID_NVM_READ_BYTE_RESP, &tempChar);
                    
                }else if(messageID == ID_NVM_WRITE_BYTE){
                    
                    Protocol_GetPayload(&writeBytePayload);
                    writeBytePayload.address = Protocol_IntEndednessConversion(writeBytePayload.address);
                    NonVolatileMemory_WriteByte(writeBytePayload.address, writeBytePayload.data);
                    Protocol_SendMessage(1,ID_NVM_WRITE_BYTE_ACK, &writeBytePayload.data);
                    
                }else if(messageID == ID_NVM_READ_PAGE){
                    
                    address = 0;
                    Protocol_GetPayload(&address);
                    address = Protocol_IntEndednessConversion(address);
                    NonVolatileMemory_ReadPage(address, 64, data);
                    Protocol_SendMessage(64,ID_NVM_READ_PAGE_RESP, data);
                    
                }else if(messageID == ID_NVM_WRITE_PAGE){   
                    
                    Protocol_GetPayload(&writePagePayload);
                    writePagePayload.address = Protocol_IntEndednessConversion(writePagePayload.address);
                    NonVolatileMemory_WritePage(writePagePayload.address, 64, writePagePayload.data);
                    Protocol_SendMessage(0,ID_NVM_WRITE_PAGE_ACK, &tempChar);
                    
                }else{
                    sprintf(testMessage, "Invalid ID");
                    Protocol_SendDebugMessage(testMessage);
                }
            }
        }
    }
}

#endif