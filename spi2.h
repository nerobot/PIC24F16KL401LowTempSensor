/* 
 * File:   spi2.h
 * Author: stevend
 *
 * Created on 23 July 2018, 08:15
 */

#ifndef SPI2_H
#define	SPI2_H

#ifdef	__cplusplus
extern "C" {
#endif

#define CSEE    _RB13       // select line for Serial EEPROM
    
// Define modes of SPI bus 
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
    
// 25LC256 Serial EEPROM commands
#define SEE_WRSR    1       // write status register    
#define SEE_WRITE   2       // write command
#define SEE_READ    3       // read command
#define SEE_WDI     4       // write disable
#define SEE_RDSR    5       // read status register
#define SEE_WEN     6       // write enable

void spiInit(uint8_t SpiMode);
uint8_t spiTransmit(uint8_t data);
void iWriteSEE( uint16_t address, uint16_t data);
uint16_t iReadSEE( uint16_t address);
uint8_t WriteSPI2(uint8_t data);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI2_H */

