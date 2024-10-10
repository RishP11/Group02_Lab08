/*
 * EE 690/615 Embedded Systems Lab No. 08
 * Group 02 :
 * 210020009 - Ganesh Panduranga Karamsetty
 * 210020036 - Rishabh Pomaje
 * Program to setup the I2C module on the muC and interface an OLED device via I2C.
*/
#define CLOCK_HZ    16000000

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void CLK_enable( void );
void PORT_E_init( void );

int main(void)
{

}

void CLK_enable( void )
{

}

void PORT_E_init( void )
{
    GPIO_PORTE_LOCK_R = 0x4C4F434B ;                            // Unlock commit register
    GPIO_PORTE_CR_R = 0xF1 ;                                    // Make PORT-E configurable
    GPIO_PORTE_DEN_R = 0x03 ;                                   // Set PORT-E pins as digital pins
    GPIO_PORTE_DIR_R = 0x02 ;                                   // Set PORT-E pin directions
    GPIO_PORTE_PUR_R = 0x02 ;                                   // Pull-Up-Resistor Register
    GPIO_PORTE_AFSEL_R = 0x03 ;                                 // Alternate function select for the PE0 and PE1
    GPIO_PORTE_PCTL_R = 0x11 ;                                  // Selecting the peripheral for the driving AFSEL
}
