/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: buzzer.c
* Theme: Launch a module
* Functions: buzzer_on(void), buzzer_off(void), buzzer_beep(int i)
* Global Variables: None
*/

#define F_CPU 14745600
#include <avr/io.h>
#include <util/delay.h>

/*
* Function Name:buzzer_on
* Input: None
* Output: Buzzer turns on
* Logic: Turns the buzzer on by writing 1 to PC3
* Example Call: buzzer_on ()
*/
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PORTC;
	port_restore = port_restore | 0x08; //xxxx 1xxx
	PORTC = port_restore;
}

/*
* Function Name:buzzer_off
* Input: None
* Output: Buzzer turns off
* Logic: Turns the buzzer off by writing 0 to PC3
* Example Call: buzzer_off()
*/
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PORTC;
	port_restore = port_restore & 0xF7; //xxxx 0xxx
	PORTC = port_restore;
}

/*
* Function Name:buzzer_beep
* Input: i -> integer which stores the number of times to beep
* Output: Buzzer beeps for 'i' times
* Logic: Turns the buzzer on and off by writing 1 & 0 to PC3
* Example Call: buzzer_beep(3)
*/
void buzzer_beep(int i)
{
	for(;i>=1;i--)
	{
		buzzer_on();
		_delay_ms(50);
		buzzer_off();
		_delay_ms(50);
	}
}
