#include <xc.h>
#include "spi2.h"

void spiInit(uint8_t SpiMode){
    // Set appropriate GPIO pins related with the MSSP and SPI2
    TRISBbits.TRISB15 = 0;      // GPIO RB15 set as an output (SCK2)
    TRISAbits.TRISA1 = 0;       // GPIO RA1 set as an output (SDO2)
    TRISAbits.TRISA0 = 1;       // GPIO RA0 set as an input  (SDI2)
    // Disable analog features for the PORTB
    ANSBbits.ANSB15 = 0;        // Disable analog features for the GPIO RB15
    ANSAbits.ANSA1 = 0;         // Disable analog features for the GPIO RA1
    ANSAbits.ANSA0 = 0;         // Disable analog features for the GPIO RA0
    
    // Configure MSSP in the fucntion of SPI1 - Connected to SSP2 (SSP1 used for I2C)
    //SSP2CON1 - Control register
    SSP2CON1bits.SSPEN = 0;     // Disable SPI1  module
    SSP2CON1bits.WCOL = 0;      // Clear collision detect bit
    SSP2CON1bits.SSPM = 0b0010; // SPI Master mode, Clock = FOSC/32 
    
    switch(SpiMode)
    {
    case 0:
      SSP2CON1bits.CKP = 0;   // Clock polarity select bit 0 = Idle state for clock is a low level
      SSP2STATbits.CKE = 1;   // SPI Clock Select bit - 1 = Transmit occurs on transition from active to Idle clock state
      break;
    case 1:
      SSP2CON1bits.CKP = 0;   // Clock polarity select bit - Idle state for clock is a low level
      SSP2STATbits.CKE = 0;   // SPI Clock Select bit - 0 = Transmit occurs on transition from Idle to active clock state
      break;
    case 2:
      SSP2CON1bits.CKP = 1;   // Clock polarity select bit 1 = Idle state for clock is a high level
      SSP2STATbits.CKE = 1;   // SPI Clock Select bit - 1 = Transmit occurs on transition from active to Idle clock state
      break; 
    case 3:
      SSP2CON1bits.CKP = 1;   // Clock polarity select bit 1 = Idle state for clock is a high level
      SSP2STATbits.CKE = 0;   // SPI Clock Select bit - 0 = Transmit occurs on transition from Idle to active clock state
      break;  
    default:
      SSP2CON1bits.CKP = 0;   // Clock polarity select bit - Idle state for clock is a low level
      SSP2STATbits.CKE = 1;   // SPI Clock Select bit - 1 = Transmit occurs on transition from active to Idle clock state
      break;
}
    
    SSP2STATbits.SMP = 0;       // Sample bit 0 = Input data is sampled at the middle of the data output time
    SSP2CON1bits.SSPEN = 1;     // Enable SPI1  module
}

uint8_t spiTransmit(uint8_t data){
    SSP2CON1bits.WCOL = 0;      // Clear collision detect bit
    SSP2BUF = data;             // Load data to the SPI sending buffer
    while (SSP2STATbits.BF == 0){}  // Wait until all data is sent
    return SSP2BUF;
}

uint8_t WriteSPI2(uint8_t data){
    SSP2BUF = data;                     // Write to buffer for TX
    while (SSP2STATbits.BF == 0){}  // Wait for transfer to complete
    return SSP2BUF;                     // Read the received value and return it
}

uint8_t ReadSR( void)
{
    // Check the Serial EEPROM status register
    int i;  
    CSEE = 0;               // select the Serial EEPROM
    WriteSPI2( SEE_RDSR);   // send a READ STATUS COMMAND
    i = WriteSPI2( 0);      // send/receive
    CSEE = 1;               // deselect to terminate command
    return i;
} //ReadSR

void WriteEnable( void)
{
    // send a Write Enable command
    CSEE = 0;               // select the Serial EEPROM
    WriteSPI2( SEE_WEN);    // write enable command
    CSEE = 1;               // deselect to complete the command
}//WriteEnable   

void iWriteSEE( uint16_t address, uint16_t data)
{ // write a 16-bit value starting at an even address

    // wait until any work in progress is completed
    while ( ReadSR() & 0x1);    // check the WIP flag
    
    // Set the Write Enable Latch
    WriteEnable();
    
    // perform a 16-bit write sequence (2 byte page write)
    CSEE = 0;                   // select the Serial EEPROM
    WriteSPI2( SEE_WRITE);      // write command
    WriteSPI2( address>>8);     // address MSB first
    WriteSPI2( address & 0xfe); // address LSB (word aligned)
    WriteSPI2( data >>8);       // send msb
    WriteSPI2( data & 0xff);    // send lsb
    CSEE = 1;
}//iWriteSEE

uint16_t iReadSEE( uint16_t address)
{ // read a 16-bit value starting at an even address

    int lsb, msb;

    // wait until any work in progress is completed
    while ( ReadSR() & 0x1);    // check the WIP flag
    
    // perform a 16-bit read sequence (two byte sequential read)
    CSEE = 0;                   // select the Serial EEPROM
    WriteSPI2( SEE_READ);       // read command
    WriteSPI2( address>>8);     // address MSB first
    WriteSPI2( address & 0xfe); // address LSB (word aligned)
    msb = WriteSPI2( 0);        // send dummy, read msb
    lsb = WriteSPI2( 0);        // send dummy, read lsb
    CSEE = 1;
    return ( (msb<<8)+ lsb);    
}//iReadSEE

