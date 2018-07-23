#include <xc.h>
#include "i2c1.h"
#include <libpic30.h>


#define SCL            (1 << 8)   // bit 6
#define SDA            (1 << 9)   // bit 7

#define I2C_ANSEL      ANSB
#define I2C_INPORT     PORTB
#define I2C_TRIS       TRISB

#define I2C_SET_SCL     I2C_TRIS |= SCL;
#define I2C_SET_SCL    I2C_TRIS |= SCL;
#define I2C_CLR_SCL    I2C_TRIS &= ~SCL;
#define I2C_SET_SDA    I2C_TRIS |= SDA;
#define I2C_CLR_SDA    I2C_TRIS &= ~SDA;
#define I2C_DELAY      __delay_ms(100);   // compiler-specific function: MPLAB X v3

void i2c_init(void)
{
    ODCB &= ~( SCL | SDA );
    
    ANSB &= ~( SCL | SDA );
    
    I2C_ANSEL &= ~( SCL | SDA );   // set as digital IO

    I2C_SET_SCL
    I2C_SET_SDA

    I2C_DELAY
}



// Initiating an I2C start condition:
void i2c_start_condition( void )
{
    I2C_SET_SCL
    I2C_SET_SDA
    I2C_DELAY
    I2C_CLR_SDA
    I2C_DELAY
    I2C_CLR_SCL
    I2C_DELAY
    /*I2C_SET_SDA
    I2C_SET_SCL
    I2C_CLR_SDA
    I2C_CLR_SCL
    I2C_DELAY*/
}

// Initiating an I2C stop condition:
void i2c_stop_condition( void )
{
    I2C_CLR_SDA
    I2C_DELAY
    I2C_SET_SCL
    I2C_DELAY
    I2C_SET_SDA
    I2C_DELAY
}

// Writing a bit in I2C:
void i2c_write_bit( uint8_t b )
{
    if( b > 0 )
        I2C_SET_SDA
    else
        I2C_CLR_SDA

    I2C_DELAY
    I2C_SET_SCL
    I2C_DELAY
    I2C_CLR_SCL
}

// Reading a bit in I2C:
uint8_t i2c_read_bit( void )
{
    uint8_t b;
    
    I2C_SET_SDA
    I2C_DELAY
    I2C_SET_SCL
    I2C_DELAY
       
    if( I2C_INPORT & SDA ) b = 1;
    else b = 0;
    
    I2C_CLR_SCL
    
    return b;
}

// Writing a byte with I2C:
bool i2c_write_byte( uint8_t B,
                     bool start,
                     bool stop )
{
    uint8_t ack = 0;

    if(start)
        i2c_start_condition();

    uint8_t i;
    for( i = 0; i < 8; i++ )
    {
        i2c_write_bit( B & 0x80 );   // write the most-significant bit
        B <<= 1;
    }
    
    ack = !i2c_read_bit();

    if(stop)
        i2c_stop_condition();
    
    return ack;
}

// Reading a byte with I2C:
uint8_t i2c_read_byte( bool ack,
                       bool stop )
{
    uint8_t B = 0;

    uint8_t i;
    for( i = 0; i < 8; i++ )
    {
        B <<= 1;
        B |= i2c_read_bit();
    }

    if( ack ) i2c_write_bit(0);
    else i2c_write_bit(1);

    if( stop ) i2c_stop_condition();

    return B;
}

// Sending a byte with I2C:
bool i2c_send_byte( uint8_t address,
                    uint8_t data )
{
    //if( i2c_write_byte( address, true, false ) )   // start, send address, write
    if( i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
        // send data, stop
        if( i2c_write_byte( data, false, true ) )  return true;
    }
    
    i2c_stop_condition();   // make sure to impose a stop if NAK'd
    return false;
}

// Receiving a byte with a I2C:
uint8_t i2c_receive_byte( uint8_t address )
{
    //if( i2c_write_byte( ( address) | 0x01, true, false ) )   // start, send address, read
    if( i2c_write_byte( ( address << 1 ) | 0x01, true, false ) )   // start, send address, read
    {
        return i2c_read_byte( false, true );
    }

    return 0;   // return zero if NAK'd
}

// Sending a byte of data with I2C:
bool i2c_send_byte_data( uint8_t address,
                         uint8_t reg,
                         uint8_t data )
{
    //if( i2c_write_byte( address, true, false ) )   // start, send address, write
    if( i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
        if( i2c_write_byte( reg, true, false ) )   // send desired register
        {
            if( i2c_write_byte( data, false, true ) ) return true;   // send data, stop
        }
    }

    i2c_stop_condition();
    return false;
}

// Receiving a byte of data with I2C:
uint8_t i2c_receive_byte_data( uint8_t address,
                               uint8_t reg )
{
    //            putU1S("Sending addressn\r");
    
    //if( i2c_write_byte( address, true, false ) )   // start, send address, write
    if( i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
    //                putU1S("Sent asddress\n\r");                   
        if( i2c_write_byte( reg, false, false ) )   // send desired register
        {
      //      putU1S("Sent desired register \n\r");
            //if( i2c_write_byte( ( address), true, false ) )   // start again, send address, read
            if( i2c_write_byte( ( address << 1) | 0x01, true, false ) )   // start again, send address, read
            {
      //          putU1S("REading byte\n\r");
                return i2c_read_byte( false, true );   // read data
                //putU1S("")
            }
        }
    }

    i2c_stop_condition();
    return 0;   // return zero if NACKed
}

uint16_t i2c_receive_16bit_data(uint8_t address, uint8_t reg){
    uint16_t data = 0;
    //if( i2c_write_byte( (address), true, false ) )   // start, send address, write
    if( i2c_write_byte( (address << 1) & 0b11111110, true, false ) )   // start, send address, write
    {
        if( i2c_write_byte( reg, false, false ) )   // send desired register
        {
            //if( i2c_write_byte( ( address), true, false ) )   // start again, send address, read
            if( i2c_write_byte( ( address << 1) | 0x01, true, false ) )   // start again, send address, read
            {
                data = (i2c_read_byte( true, false ) << 8);   // read data
                data += (i2c_read_byte( false, true) & 0x00ff );   // read data
                return data;
            }
        }
    }

    i2c_stop_condition();
    return 0;   // return zero if NACKed
}

bool i2c_write_16bit_data(uint8_t address, uint8_t reg, uint16_t data){    
    //if( i2c_write_byte( (address), true, false ) )   // start, send address, write
    if( i2c_write_byte( (address << 1) | 0b00000001, true, false ) )   // start, send address, write
    {
        if( i2c_write_byte( reg, false, false ) )   // send desired register
        {
            //if( i2c_write_byte( ( address), true, false ) )   // start again, send address, read
            if( i2c_write_byte( ( address << 1) | 0x01, true, false ) )   // start again, send address, read
            {
                i2c_write_byte( true, false , data>>8);   // read data
                i2c_write_byte( false, true, data & 0xff);   // read data
                return 1;
            }
        }
    }

    i2c_stop_condition();
    return 0;   // return zero if NACKed
}
