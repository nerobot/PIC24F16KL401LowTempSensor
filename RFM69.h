/* 
 * File:   RFM69.h
 * Author: scdag
 *
 * Created on 25 May 2018, 18:39
 */

#ifndef RFM69_H
#define	RFM69_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "spi2.h"
    
#define bool    uint8_t
#define _interruptPin PORTBbits.RB14                //_PORTRB14


//#define CSEE    _RB13       // select line for Serial EEPROM    
#define RF69_SPI_CS             _RB13   // SS is the SPI slave select pin
#define RF69_IRQ_PIN          2         // INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
    
#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE
 
// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
    // 8 / 2^19 = 
//#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)
#define RF69_FSTEP          15.25879  //  8 / 2^19 = 15.25 LOOK INTO THIS!!!
    
// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

//#define RF69_LISTENMODE_ENABLE  //comment this line out to compile sketches without the ListenMode (saves ~2k)

#if defined(RF69_LISTENMODE_ENABLE)
  // By default, receive for 256uS in listen mode and idle for ~1s
  #define  DEFAULT_LISTEN_RX_US 256
  #define  DEFAULT_LISTEN_IDLE_US 1000000
#endif

    static volatile uint8_t _slaveSelectPin;
 //   static volatile uint8_t _interruptPin;
    static volatile uint8_t _interruptNum;
    static volatile uint8_t _address;
    static volatile uint8_t _promiscuousMode;
    static volatile uint8_t _powerLevel;
    static volatile uint8_t _isRFM69HW;
#if defined (SPCR) && defined (SPSR)
    uint8_t _SPCR;
    uint8_t _SPSR;
#endif
    
    static volatile uint8_t DATA[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
    static volatile uint8_t DATALEN;
    static volatile uint8_t SENDERID;
    static volatile uint8_t TARGETID; // should match _address
    static volatile uint8_t PAYLOADLEN;
    static volatile uint8_t ACK_REQUESTED;
    static volatile uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
    static volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception)
    static volatile uint8_t _mode; // should be protected?
    
    
    static volatile bool _inISR;

void RFM69(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW);
bool RFM69Initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
void writeReg(uint8_t addr, uint8_t val);
uint8_t readReg(uint8_t addr);

 void encrypt(const char* key);
 
  void setMode(uint8_t mode);
  void setHighPowerRegs(bool onOff);
  
  void setHighPower(bool onOFF); // has to be called after initialize() for RFM69HW
  
  void select();
  void unselect();
  
  void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
  bool receiveDone();
  
  void RFM69sleep();
  
  void setPowerLevel(uint8_t level); // reduce/increase transmit power level


#ifdef	__cplusplus
}
#endif

#endif	/* RFM69_H */

