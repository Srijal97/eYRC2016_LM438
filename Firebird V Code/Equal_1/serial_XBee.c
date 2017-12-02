/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: serial_XBee.c
* Theme: Launch a module
* Functions: uart0_init(void)
* ISRs: ISR(USART0_RX_vect) 
* Global Variables: None
*/

#include "main.h"

extern volatile unsigned char data;  //to store received data from UDR0
extern volatile int flag_instruction;//no of instructions pending

/*
* Function Name:uart0_init
* Input: None
* Output: Function To Initialize UART0
* Parameters: baud rate:9600 (error 0.0%)
			  char size: 8 bit
			  parity: Disabled
* Example Call: uart0_init()
*/
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

/*
* ISR for serial communication via XBee
* Output: Function To Initialize UART0
* Parameters: baud rate:9600 (error 0.0%)
			  char size: 8 bit
			  parity: Disabled
*ISR is triggered when data is present in the data buffer
*/
ISR(USART0_RX_vect) 		// ISR for receive complete interrupt //, ISR_NOBLOCK
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	flag_instruction++;			//One instruction received 
		
	if(data == 0x20) //ASCII value of space
	{
		stop(); //stop
		//flag_instruction--;
	}
}