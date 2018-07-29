/* 
 * File:   uart1.h
 * Author: stevend
 *
 * Created on 23 July 2018, 07:07
 */

#ifndef UART1_H
#define	UART1_H

#ifdef	__cplusplus
extern "C" {
#endif

// Used to turn the uart on or off as needed
//#define DEBUG    
    
// Oscillator config    
#define XTAL_FREQ       8000000UL               // Raw oscillator freq
#define FCY             XTAL_FREQ / 2           // Most but not all PIC24s
    
// UART1 config
#define BR              9600                    // Baud rate
#define bRate           ((FCY/BR)/16)-1
#define U1MODE_HIGH     0b10000000              // Bits 8-15
#define U1MODE_LOW      0b00000000              // Bits 0-7
#define u1Mode          (U1MODE_HIGH << 8) + U1MODE_LOW
#define U1STA_HIGH      0b00010100              // Bits 8-15
#define U1STA_LOW       0b00000000              // Bits 0-7
#define u1Sta           (U1STA_HIGH << 8) + U1STA_LOW      

void initU1();                                  //! Initialise the Uart1 module    
void putU1(char c);                             //! Put a single byte on the uart
void putU1S(char *s);                           //! Print a string of unknown length, ending in \0
void putU1S2(char *s, uint8_t size);            //! Print a string of known length

#ifdef	__cplusplus
}
#endif

#endif	/* UART1_H */

