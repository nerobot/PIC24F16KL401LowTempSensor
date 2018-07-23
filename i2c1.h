/* 
 * File:   i2c1.h
 * Author: stevend
 *
 * Created on 23 July 2018, 07:39
 */

#ifndef I2C1_H
#define	I2C1_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// Oscillator config
#define XTAL_FREQ       8000000UL               // Raw oscillator freq
#define FCY             XTAL_FREQ / 2           // Most but not all PIC24s
    
#define true            1
#define false           0
#define bool            uint8_t

void i2c_init(void);

// Initiating an I2C start condition:
void i2c_start_condition( void );

// Initiating an I2C stop condition:
void i2c_stop_condition( void );

// Writing a bit in I2C:
void i2c_write_bit( uint8_t b );

// Reading a bit in I2C:
uint8_t i2c_read_bit( void );

//    if( i2c_write_byte( address << 1, true, false ) )   // start, send address, write

// Writing a byte with I2C:
bool i2c_write_byte( uint8_t B,
                     bool start,
                     bool stop );

// Reading a byte with I2C:
uint8_t i2c_read_byte( bool ack,
                       bool stop );

// Sending a byte with I2C:
bool i2c_send_byte( uint8_t address,
                    uint8_t data );

// Receiving a byte with a I2C:
uint8_t i2c_receive_byte( uint8_t address );
// Sending a byte of data with I2C:
bool i2c_send_byte_data( uint8_t address,
                         uint8_t reg,
                         uint8_t data );

// Receiving a byte of data with I2C:
uint8_t i2c_receive_byte_data( uint8_t address,
                               uint8_t reg );

uint16_t i2c_receive_16bit_data(uint8_t address, uint8_t reg);

bool i2c_write_16bit_data(uint8_t address, uint8_t reg, uint16_t data);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C1_H */

