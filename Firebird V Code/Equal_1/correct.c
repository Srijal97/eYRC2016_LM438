/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: correct.c
* Theme: Launch a module
* Functions: 
	correct_left(unsigned int),
	correct_right(unsigned int),
	correct_left_2(unsigned int),
	correct_right_2(unsigned int)
* Global Variables: temp
*/

#include "main.h"

unsigned char temp; //Variable to store PORTA before correction and set the value after correction.
extern unsigned int TurnSpeedLeft;
extern unsigned int TurnSpeedRight;

/*
* Function Name:correct_left
* Input: degrees -> unsigned int which stores the degrees to turn
* Output: Turns the bot left by the specified number of degrees. 
		  Both wheels spin in opposite directions.
		  The motion of the bot after the function execution will be the same as before execution.
* Logic: 1. Store direction(PORTA) in temp
		 2. Turn left
		 3. Restore the motion of the bot
* Example Call: correct_left(10)
*/
void correct_left(unsigned int degrees)
{
	temp = PORTA;
	
	unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	if(var == 0x00)
	{
		velocity(TurnSpeedLeft, TurnSpeedRight);
	}
	
	left_degrees(degrees);
	start_encoder_feedback_correction_later();
	PORTA = temp;
}

/*
* Function Name:correct_right
* Input: degrees -> unsigned int which stores the degrees to turn
* Output: Turns the bot left by the specified number of degrees.
		  Both wheels spin in opposite directions.
		  The motion of the bot after the function execution will be the same as before execution.
* Logic: 1. Store direction(PORTA) in temp
		 2. Turn right
		 3. Restore the motion of the bot* Example Call: correct_right(10)
* Example Call: correct_right(10)
*/
void correct_right(unsigned int degrees)
{
	temp = PORTA;

	unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	if(var == 0x00)
	{
		velocity(TurnSpeedLeft, TurnSpeedRight);
	}

	right_degrees(degrees);
	start_encoder_feedback_correction_later();
	PORTA = temp;
}

/*
* Function Name:correct_left_2
* Input: degrees -> unsigned int which stores the degrees to turn
* Output: Turns the bot left by the specified number of degrees. 
		  Only the right wheel spins forward.
		  The motion of the bot after the function execution will be the same as before execution.
* Logic: 1. Store direction(PORTA) in temp
		 2. Turn left
		 3. Restore the motion of the bot
* Example Call: correct_left_2(10)
*/
void correct_left_2(unsigned int degrees)
{
	unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	if(var == 0x00)
	{
		velocity(TurnSpeedLeft, TurnSpeedRight);
	}
	
	temp = PORTA;
	soft_left_degrees(degrees);
	start_encoder_feedback_correction_later();
	PORTA = temp;
}

/*
* Function Name:correct_right_2
* Input: degrees -> unsigned int which stores the degrees to turn
* Output: Turns the bot left by the specified number of degrees.
		  Only the left wheel spins forward.
		  The motion of the bot after the function execution will be the same as before execution.
* Logic: 1. Store direction(PORTA) in temp
		 2. Turn right
		 3. Restore the motion of the bot
* Example Call: correct_right_2(10)
*/
void correct_right_2(unsigned int degrees)
{
	unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	if(var == 0x00)
	{
		velocity(TurnSpeedLeft, TurnSpeedRight);
	}
	
	temp = PORTA;
	soft_right_degrees(degrees);
	start_encoder_feedback_correction_later();
	PORTA = temp;
}