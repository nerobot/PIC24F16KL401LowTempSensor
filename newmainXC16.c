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

int main(void) {
    
    //_TRISB4 = 0;
    //_ANSB4 = 0;
    //PORTBbits.RB4 = 0;
    
    while(1){
    //    PORTBbits.RB4 ^= 1;
        __delay_ms(1000);
    }
    
    return 0;
}
