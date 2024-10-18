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

void PORT_E_init( void )
{
    GPIO_PORTE_LOCK_R = 0x4C4F434B ;                            // Unlock commit register
    GPIO_PORTE_CR_R = 0xF1 ;                                    // Make PORT-E configurable
    GPIO_PORTE_DEN_R = (1 << 5) | (1 << 4) ;                                   // Set PORT-E pins as digital pins
//    GPIO_PORTE_DIR_R = (1 << 5) | (1 << 4) ;                                   // Set PORT-E pin directions
    GPIO_PORTE_PUR_R = (1 << 5) | (1 << 4) ;                                   // Pull-Up-Resistor Register
    GPIO_PORTE_AFSEL_R = (1 << 5) | (1 << 4) ;                                 // Alternate function select for the PE0 and PE1
    GPIO_PORTE_PCTL_R = 0x330000 ;                                  // Selecting the peripheral for the driving AFSEL
    GPIO_PORTB_ODR_R |= (1 << 5) ;                                        // Set SDA line in Open Drain Configuration
}

void PORT_F_init( void )
{
    GPIO_PORTF_LOCK_R = 0x4C4F434B ;                            // Unlock commit register
    GPIO_PORTF_CR_R = 0xF1 ;                                    // Make PORT-F configurable
    GPIO_PORTF_DEN_R = 0x1F ;                                   // Set PORT-F pins as digital pins
    GPIO_PORTF_DIR_R = 0x0E ;                                   // Set PORT-F pin directions
    GPIO_PORTF_PUR_R = 0x11 ;                                   // Pull-Up-Resistor Register
    GPIO_PORTF_DATA_R = 0x00 ;                                  // Clearing previous data
}

void CLK_enable( void )
{
    SYSCTL_RCGCI2C_R |= (1 << 2) | (1 << 0) ;   // Provide clocks to Modules 0 and 2
    SYSCTL_RCGCGPIO_R |= (1 << 5) | (1 << 4) | (1 << 1) ; // Clocks to Ports B, E, and F
}
