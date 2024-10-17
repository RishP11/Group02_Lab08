/*
 * EE 690/615 Embedded Systems Lab No. 08
 * Group 02 :
 * 210020009 - Ganesh Panduranga Karamsetty
 * 210020036 - Rishabh Pomaje
 * Program to setup the I2C module on the muC and communicate with itself or other muC.
*/
#define CLOCK_HZ    16000000

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void CLK_enable( void );
void PORT_F_init( void );
void PORT_E_init( void );
void PORT_B_init( void );
void i2c_init( void );
void i2c_dataTx( uint8_t dataByte ) ;

int main( void )
{
    while(1){
        ;
    }
}
