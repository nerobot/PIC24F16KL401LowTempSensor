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
#include "PCF8563.h"

// Data structure that will contain all data being sent via the RFM69 RF module.
// Serial gateway needs updating whenever this gets updated.
struct dataStruct{
    uint16_t    temp;
    uint8_t     hour;
    uint8_t     minute;
    uint8_t     second;
    uint8_t     day;
    uint8_t     month;
    uint8_t     year;
} data;

typedef enum {
    GATEWAY_ID,
    WEATHER_STATION_ID,	
    TEST_STATION_1_ID,
    TEST_STATION_2_ID
} id_t;

#define NETWORKID       0   					// Must be the same for all nodes
#define MYNODEID        TEST_STATION_2_ID   	// My node ID
#define TONODEID        GATEWAY_ID              // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY       RF69_433MHZ

// AES encryption (or not):
#define ENCRYPT         true                    // Set to "true" to use encryption
#define ENCRYPTKEY      "TOPSECRETPASSWRD"      // Use the same 16-byte key on all nodes

uint8_t timerMinute = 1;

void __attribute__((interrupt,auto_psv)) _INT2Interrupt(void) //External Interrupt 2
{
    _INT2IF = 0;
}

int main(void) {
    // Setting up needed variables.
    
    TRISA = 0x0000;
    TRISB = 0x0000;
    
    _HLVDEN = 0;
    PMD4bits.HLVDMD = 0;
    PMD1 = 0xffff;
    PMD2 = 0xffff;
    PMD3 = 0xffff;
    PMD4 = 0xffff;
    
    _T1MD = 0;
    //_T2MD = 0;
    
    //        _T3MD = 0;
   // _T4MD = 0;
    _SSP1MD = 0;
    _U1MD = 0;
    _SSP2MD = 0;
    
    // Setting up IO
    LEDTRIS = 0;
    LEDANS = 0;
    LED = 0;
    
    RTCINTTRIS = 1;
    
    _TRISA3 = 0;
    _ANSA3 = 0;
    _RA3 = 1;
    
    // init uart1
    initU1();
    putU1S("uart1 init\n\r");
    
    // init i2c1
    i2cInit();
    putU1S("i2c1 init\n\r");
    
    // init mcp9808
    while(!initMCP()){
        putU1S("mcp init failed\n\r");
    }
    putU1S("mcp init\n\r");
    mcpShutdown();
    
    
    // spi init
    spiInit(SPI_MODE0);
    putU1S("spi init\n\r");
    
    // init rfm
    RFM69(0,0,0);           // This should probably be improved!
    putU1S("RFM69 initialised 1.\n\r");   
    while (RFM69Initialize(FREQUENCY, MYNODEID, NETWORKID) == 0){}
    putU1S("RFM69 initialised 2.\n\r");   
    encrypt(ENCRYPTKEY);
    putU1S("RFM69 initialised 3.\n\r");     
    
    // init RTC
 
    pcf8563_init();
    zeroClock();
    //putU1S("RTC init\n\r");
    //setAlarm(timerMinute++, 100, 100, 100);
   // putU1S("alarm set\r\n");
    _INT2EP = 1;
    _INT2IE = 1;
    _INT2IP = 0b111;
    
    
    
    while(1){
        _T1MD = 0;
        _SSP1MD = 0;
        _SSP2MD = 0;
        
        i2cInit();
        spiInit(SPI_MODE0);
        _RA3 = 1;
        mcpWake();
        data.temp = 0;
        __delay_ms(1000);
        putU1S("alarm triggered\n\r");
        LED ^= 1;
        clearAlarm();
        setAlarm(timerMinute++, 100, 100, 100);
        if (timerMinute >= 60){
            timerMinute = 0;
        }
        putU1S("alarm set again\n\r");

        getDateTime();   
        data.hour = getHour();
        //putU1(getHour());
        data.minute = getMinute();
        //utU1(getMinute());
        data.second = getSecond();
        
        data.temp = readTemp();
        mcpShutdown();
        send(GATEWAY_ID, (const void*)(&data), sizeof(data), 1);
        RFM69sleep();
        _RA3 = 0;
        _T1MD = 1;
        _SSP1MD = 1;
        _SSP2MD = 1;
        
        TRISBbits.TRISB9 = 0;
        TRISBbits.TRISB8 = 0;
        Sleep();
        //putU1(getSecond());   
       // __delay_ms(1000);
        //Sleep();
        //if (RTCIN == 0){
            
        //}
        
        /*
        // Reading the temperature
        data.temp = readTemp();              
        
        //putU1((char)(data.temp >> 8));
        //putU1((char)(data.temp & 0xff));
       // putU1S("Obtaining time\n\r");
        getDateTime();
        
 
        putU1(getHour());
        putU1(getMinute());
        putU1(getSecond());
        
        data.hour = getHour();
        data.minute = getMinute();
        data.second = getSecond();
        
        // Sending the temperature
        //send(GATEWAY_ID, (const void*)(&data), sizeof(data), 0);
        __delay_ms(1000);*/
    }
    
    return 0;
}
