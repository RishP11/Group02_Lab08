/*
 * EE 690/615 Embedded Systems Lab Number 08
 * Group 02 :
 * 210020009 - Ganesh Panduranga Karamsetty
 * 210020036 - Rishabh Pomaje
 * Program to setup the UART module on the muC and communicate with PC.
*/
#define CLOCK_HZ    16000000

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

void CLK_enable( void );
void PORT_F_init( void );
void PORT_A_init( void );
void UART0_setup( void );
void UART_Tx( char );
char UART_Rx( void );

int main(void)
{
    CLK_enable();                                               // Enable all the required Clocks
    PORT_F_init();                                              // Setup Port F to interface with LEDs and Switches
    PORT_A_init();                                              // Setup Port E to interface with the UART
    UART0_setup();                                              // Setup UART Module 0

    while(1){
        char rxData = UART_Rx() ;                               // Check if there is any Rx message in FIFO
            // GPIO_PORTF_DATA_R = |_|_|_|SW1|G|B|R|SW2|
            if (rxData == 'R'){
                GPIO_PORTF_DATA_R = 0x02 ;                      // Received Data is 'R' :: Turn on Red LED.
                UART_Tx(rxData) ;
            }
            else if (rxData == 'G'){
                GPIO_PORTF_DATA_R = 0x08 ;                      // Received Data is 'G' :: Turn on Green LED.
                UART_Tx(rxData) ;
            }
            else if (rxData == 'B'){
                GPIO_PORTF_DATA_R = 0x04 ;                      // Received Data is 'B' :: Turn on Blue LED.
                UART_Tx(rxData) ;
            }
            else if (rxData != 0){
                GPIO_PORTF_DATA_R = 0x00 ;                      // Unrecognized LED :: Turn off led
                UART_Tx(rxData) ;
            }
    }
}

void CLK_enable( void )
{
    // Setting up the UART clocks
    SYSCTL_RCGCUART_R |= (1 << 0) ;                             // Enabling the clock to UART module 0
    SYSCTL_RCGCGPIO_R |= (1 << 0) ;                             // Enable clock to GPIO_A
    SYSCTL_RCGCGPIO_R |= (1 << 5) ;                             // Enable clock to GPIO_F
}

void UART_Tx( char data )
{
    while((UART0_FR_R & (1 << 3)) != 0){                        // |7-TXFE|6-RXFF|5-TXFF|4-RXFE|3-BUSY|2-...|1-...|0-CTS|
         ;                                                      // Check for BUSY bit and wait for Tx-FIFO to become free
    }
    UART0_DR_R = data ;                                         // Place the Tx-message in the Data Register
}

char UART_Rx( void )
{
    if((UART0_FR_R & 0x40) != 0){                               // |7-TXFE|6-RXFF|5-TXFF|4-RXFE|3-BUSY|2-...|1-...|0-CTS|
        char rxData = UART0_DR_R ;
        return rxData ;                                         // Read the Rx-message from the Data Register since Rx-FIFO is full
    }
    else{
        return 0x00 ;                                           // Otherwise return Null Packet.
    }
}

void UART0_setup( void )
{
    UART0_CTL_R = 0x00 ;                                        // Disabling the UART

    // Calculations for Baud Rate Divisor
    int UARTSysClk = CLOCK_HZ ;                                 // Using system clock for UART module
    int clk_div = 16 ;                                          // Clock divider depending on the communication rate
    int baud_rate = 9600 ;                                      // Baud rate for communication

    float BRD = (1.0 * UARTSysClk) / (clk_div * baud_rate) ;    // Baud rate divisor (BRD)
    int BRDI = BRD ;                                            // Integer part of BRD
    BRD = BRD - BRDI ;                                          // Extracting the fractional part of BRD
    int BRDF = 64 * BRD + 0.5 ;                                 // Fractional part of the BRD to write to the register

    // Configuring the UART
    UART0_IBRD_R = BRDI ;                                       // Integer part of the BRD :: 16-bits
    UART0_FBRD_R = BRDF ;                                       // Fractional part of the BRD :: 6 bits
    UART0_LCRH_R |= (1 << 6) | (1 << 5) | (1 << 1) ;            // |7-SPS|6,5-WLEN|4-FEN|3-STP2|2-EPS|1-PEN|0-BRK|
    UART0_CC_R = 0x00 ;                                         // Clock source of the register
    UART0_ECR_R = 0xFF ;                                        // Clear any existing errors in RSR
    UART0_CTL_R |= (1 << 9) | (1 << 8) | (1 << 0) ;             // |9-RXE|8-TXE|7-LBE|6-...|5-HSE|4-EOT|3-SMART|2-SIRLP|1-SIREN|0-UARTEN|
}

void PORT_A_init( void )
{
    GPIO_PORTA_LOCK_R = 0x4C4F434B ;                            // Unlock commit register
    GPIO_PORTA_CR_R = 0xF1 ;                                    // Make PORT-A configurable
    GPIO_PORTA_DEN_R = 0x03 ;                                   // Set PORT-A pins as digital pins
    GPIO_PORTA_DIR_R = 0x02 ;                                   // Set PORT-A pin directions
    GPIO_PORTA_PUR_R = 0x02 ;                                   // Pull-Up-Resistor Register
    GPIO_PORTA_AFSEL_R = 0x03 ;                                 // Alternate function select for the PA0 and PA1
    GPIO_PORTA_PCTL_R = 0x11 ;                                  // Selecting the peripheral for the driving AFSEL
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

