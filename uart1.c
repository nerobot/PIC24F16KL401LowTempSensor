#include <xc.h>
#include "uart1.h"

#ifdef DEBUG

    void initU1(){
        U1BRG = bRate;
        U1MODE = u1Mode;
        U1STA = u1Sta;
    }

    void putU1(char c){
        while(U1STAbits.UTXBF){}     // wait while Tx buffer is full
        U1TXREG = c;
    }

    void putU1S(char *s){
        while (*s != '\0'){
            putU1(*s++);
        }
    }

    void putU1S2(char *s, uint8_t size){
        uint8_t sizeT = 0;
        while (sizeT++ < size){
            putU1(*s++);
        }
    }
#else
    void initU1(uint8_t bRate, uint16_t u1Mode, uint16_t u1Sta){
    }

    void putU1(char c){
    }

    void putU1S(char *s){
    }

    void putU1S2(char *s, uint8_t size){
    } 
#endif

