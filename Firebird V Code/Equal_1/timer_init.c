/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: timer_init.c
* Theme: Launch a module
* Functions: timer5_init(void), timer1_init(void)
* Global Variables: None
*/

#include "main.h"

/*
* Function Name:timer1_init
* Input: None
* Output: TIMER1 initialization in 10 bit fast PWM mode for servo motor control
* Parameters: 1) Prescale:256
			  2) PWM 10bit fast, TOP=0x03FF
			  3) Timer Frequency:52.25Hz
* Example Call: timer1_init()
*/
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with

 OCR1AH = 0x03;	//Output compare Register high value for horizontal servo (S1)
 OCR1AL = 0xFF;	//Output Compare Register low Value For horizontal servo (S1)
 OCR1BH = 0x03;	//Output compare Register high value for vertical servo (S2)
 OCR1BL = 0xFF;	//Output Compare Register low Value For vertical servo (S2)
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3

 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
* Function Name:timer5_init
* Input: None
* Output: Timer 5 initialized in PWM mode for velocity control
* Parameters: 1) Prescale:256
			  2) PWM 8bit fast, TOP=0x00FF(WGM2:0 = 3)
			  3) Timer Frequency:225.000Hz
* Example Call: timer5_init()
*/
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with //8-bit clock
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with  //8-bit clock
	OCR5AH = 0x03;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x03;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x03;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
* Function Name:timer4_init
* Input: None
* Output: TIMER4 initialization in 10 bit fast PWM mode for servo motor control
* Parameters: 1) Prescale:256
			  2) PWM 10bit fast, TOP=0x03FF
			  3) Timer Frequency:52.25Hz
* Example Call: timer4_init()
*/
void timer4_init(void)
{
 TCCR4B = 0x00; //stop
 TCNT4H = 0xFC; //Counter high value to which OCR4xH value is to be compared with
 TCNT4L = 0x01;	//Counter low value to which OCR4xH value is to be compared with

 OCR4BH = 0x03;	//Output compare Register high value for back_horizontal_servo
 OCR4BL = 0xFF;	//Output Compare Register low Value For back_horizontal_servo
 OCR4CH = 0x03;	//Output compare Register high value for back_vertical_servo
 OCR4CL = 0xFF;	//Output Compare Register low Value For back_vertical_servo

 ICR4H  = 0x03;	
 ICR4L  = 0xFF;
 TCCR4A = 0xAB; /*{COM4A1=1, COM4A0=0; COM4B1=1, COM4B0=0; COM4C1=1 COM4C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM41=1, WGM40=1} Along With WGM42 in TCCR4B for Selecting FAST PWM Mode*/
 TCCR4C = 0x00;
 TCCR4B = 0x0C; //WGM42=1; CS42=1, CS41=0, CS40=0 (Prescaler=256)
}

