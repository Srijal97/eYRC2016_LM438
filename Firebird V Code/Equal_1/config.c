/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: config.c
* Theme: Launch a module
* Functions: motion_pin_config(void), left_encoder_pin_config(void), right_encoder_pin_config (void)
			 buzzer_pin_config(void), left_position_encoder_interrupt_init (void), right_position_encoder_interrupt_init (void)
* Global Variables: None
*/

#include <avr/io.h>
#include <avr/interrupt.h>

/*
* Function Name: motion_pin_config
* Input: None
* Output: Function to configure ports to enable robot's motion
* Logic: DDRs of the pins to L293d ic are set as output
* Example Call: motion_pin_config ()
*/
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function Name: left_encoder_pin_config
* Input: None
* Output: Pin connected to left position encoder is set as input
* Logic: PORTE 4 pin is set as input-xxx0xxxx
* Example Call: left_encoder_pin_config ()
*/
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name: right_encoder_pin_config
* Input: None
* Output: Pin to connected right position encoder is set as input
* Logic: PORTE 4 pin is set as input-xx0xxxxx
* Example Call: right_encoder_pin_config ()
*/
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name: left_position_encoder_interrupt_init
* Input: None
* Output: Enable interrupt 4 for the left side position encoder
* Logic: Given alongside the code
* Example Call: left_position_encoder_interrupt_init ()
*/
void left_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x01; //xxxx xx01  INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

/*
* Function Name: right_position_encoder_interrupt_init
* Input: None
* Output: Enable interrupt 5 for the left side position encoder
* Logic: Given alongside the code
* Example Call: right_position_encoder_interrupt_init ()
*/
void right_position_encoder_interrupt_init (void)
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x04; //xxxx 01xx - ISCn1 = 0 ISCn0 = 1 -> Any logical change on INTn generates an interrupt request 
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

/*
* Function Name: buzzer_pin_config
* Input: None
* Output: Function to initialize Buzzer
* Logic: Setting PORTC3 as output which is the pin connedted to buzzer
* Example Call: buzzer_pin_config()
*/
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}