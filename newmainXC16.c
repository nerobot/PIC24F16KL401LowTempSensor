/*
 * File:   newmainXC16.c
 * Author: scdag
 *
 * Created on 22 July 2018, 11:32
 */

#include "config.h"
#include "PIC24F16KL401.h"
#include "xc.h"
#include <libpic30.h>
#include "uart1.h"
#include "i2c1.h"
#include "mcp9808.h"
#include "spi2.h"
#include "RFM69.h"

typedef enum {
    GATEWAY_ID,
    WEATHER_STATION_ID,	
    TEST_STATION_1_ID,
    TEST_STATION_2_ID
} id_t;

// Data structure that will contain all data being sent via the RFM69 RF module.
struct dataStruct{
    uint16_t    temp;
    uint8_t     hour;
    uint8_t     minute;
    uint8_t     second;
    uint8_t     day;
    uint8_t     month;
    uint8_t     year;
} data;

#define NETWORKID       0   					// Must be the same for all nodes
#define MYNODEID        TEST_STATION_2_ID   	// My node ID
#define TONODEID        GATEWAY_ID              // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY       RF69_433MHZ

// AES encryption (or not):
#define ENCRYPT         true // Set to "true" to use encryption
#define ENCRYPTKEY      "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

int main(void) {
    // Setting up needed variables.
    
    // init uart1
    initU1();
    putU1S("uart1 init\n\r");
    
    // init i2c1
    i2c_init();
    putU1S("i2c1 init\n\r");
    
    // init mcp9808
    while(!initMCP()){
        putU1S("mcp init failed\n\r");
    }
    putU1S("mcp init\n\r");
    
    
    // spi init
    spiInit(SPI_MODE0);
    putU1S("spi init\n\r");
    
    // init rfm
    RFM69(0,0,0);           // This should probably be improved!
    while (RFM69Initialize(FREQUENCY, MYNODEID, NETWORKID) == 0){}
    encrypt(ENCRYPTKEY);
    putU1S("RFM69 initialised.\n\r");         
    
    while(1){
        // Reading the temperature
        data.temp = readTemp();
        // Sendding the temperature. Requesting ACK
        send(GATEWAY_ID, (const void*)(&data), sizeof(data), 1);
        
        putU1((char)(data.temp >> 8));
        putU1((char)(data.temp & 0xff));
        
        __delay_ms(1000);
    }
    
    return 0;
}
