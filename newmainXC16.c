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

int main(void) {
    
    //_TRISB4 = 0;
    //_ANSB4 = 0;
    //PORTBbits.RB4 = 0;
    
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
    
    uint16_t temp;
    
    while(1){
    //    PORTBbits.RB4 ^= 1;
        temp = readTemp();
        putU1((char)(temp >> 8));
        putU1((char)(temp & 0xff));
        __delay_ms(1000);
    }
    
    return 0;
}
