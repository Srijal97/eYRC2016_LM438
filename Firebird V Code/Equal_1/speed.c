/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: speed.c
* Theme: Launch a module
* Functions: velocity (unsigned int, unsigned int), modify_speed(void);
* Global Variables: factor_initial_value,factor, time_limit, forward_speed_left, forward_speed_right, count, CorrectLinearSpeedLeft, CorrectLinearSpeedRight, CorrectTurnSpeedLeft, CorrectTurnSpeedRight
*/

#include "main.h"

const unsigned int factor_initial_value = 1;  //factor by which the speed of the motor is changed on each speed correction
volatile unsigned int factor = 1;			  //used to turn off encoder speed corrections(off when 0), same as factor_initial_value
volatile unsigned int time_limit = 5; //15	  //do not vary speeds of time_left/time_right<time_limit
volatile unsigned int forward_speed_left = 255;  //Stores the speed of the left motor for forward direction
volatile unsigned int forward_speed_right = 255; //Stores the speed of the right motor for forward direction
volatile unsigned int reverse_speed_left = 255;  //Stores the speed of the left motor for reverse direction
volatile unsigned int reverse_speed_right = 255; //Stores the speed of the right motor for reverse direction
volatile unsigned int count = 60;			  //no of encoder pulses to be generated between each speed correction
int adjust_count = 0;

extern volatile int flag_right;				    //flag to check if right encoder has generated "count" no of pulses
extern volatile int flag_left;				    //flag to check if left encoder has generated "count" no of pulses
extern volatile unsigned int CorrectCountLeft;  //to keep track of left position encoder
extern volatile unsigned int CorrectCountRight; //to keep track of right position encoder
extern volatile unsigned int StopWatch;			//This variable is used to measure the time taken for the wheels to rotate by a specified number of encoder pulses
extern volatile unsigned int time_left;			//no of "stopwatch" ticks taken for "CorrectCountLeft" to reach "count"
extern volatile unsigned int time_right;		//no of "stopwatch" ticks taken for "CorrectCountRight" to reach "count"
extern volatile unsigned int stop_correction_stopwatch; //Counter to count if the no of ticks has reached the stop_time
extern const int stop_time; //Variable to turn off the encoder feedback correction for a set amount of time (time = stop_time/225)
extern unsigned int CorrectLinearSpeedLeft;
extern unsigned int CorrectLinearSpeedRight;
extern unsigned int CorrectTurnSpeedLeft;
extern unsigned int CorrectTurnSpeedRight;

extern const int stop_time; //Variable to turn off the encoder feedback correction for a set amount of time (time = stop_time/225)
extern volatile unsigned int stop_correction_stopwatch; //Counter to count if the no of ticks has reached the stop_time


/*
* Function Name: velocity
* Input: left_motor_speed, left_motor_speed -> values between 0-255
* Output: Speed of the motors is changed 
* Logic: Change the values of the OCRs if the PWM generating timer to change the duty cycle
		 if input speed>255 the speed is set to 255
* Example Call: velocity(200,150)
*/
void velocity(unsigned int left_motor_speed, unsigned int right_motor_speed)
{
		//lcd_print(1,1,left_motor_speed,3);
		//lcd_print(1,8,right_motor_speed,3);
	
	if(left_motor_speed<=255)
	{
		OCR5AL = (unsigned char)left_motor_speed;	
	}
	else
	{
		OCR5AL = 0xFF;
	}
	
	if(right_motor_speed<=(unsigned char)255)
	{
		OCR5BL = (unsigned char)right_motor_speed;
	}
	else
	{
		OCR5BL = 0xFF;
	}
	
	//print_all_speeds();
}

/*
* Function Name: modify_speed
* Input: None
* Output: Duty cycle of the power to the motors is changed to adjust for speed difference of both the motors

* Logic: -The function is used to keep the bot moving in a straight line.
		 - Due to the motors being non-ideal they rotate at different speeds.
		 -"StopWatch" is incremented 225 times per second
		 -The value of "StopWatch" is recorded in time_left/time_right when the left/right encoders produce
		  a specified number of pulses(count).
		 -Based on time_left/time_right we can get the speed of the robot.
		 -This time is used to increment/decrement the speed of both in a closed loop feedback system such that they 
		  try to remain equal.

* Example Call: modify_speed()
*/
void modify_speed(void)
{
	adjust_count++;
	
	volatile unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	if(var==0x06 && factor != 0) //only if forward
	{
		if((time_left-time_right)>time_limit)  //To avoid unnecessary corrections 
		{
			if(time_left>time_right) //right motor is faster
			{
				forward_speed_left+=factor;  //increment speed of left motor
				forward_speed_right-=factor; //decrement speed of right motor
				velocity(forward_speed_left,forward_speed_right);
			}
		
			if(time_right>time_left) //left motor is faster
			{
				forward_speed_left-=factor;   //decrement speed of left motor
				forward_speed_right+=factor;  //increment speed of right motor
				velocity(forward_speed_left,forward_speed_right);
			}
		}
	}
	
	else if (var==0x09 && factor != 0)	//only if reverse
	{
		if((time_left-time_right)>time_limit)  //To avoid unnecessary corrections
		{
			if(time_left>time_right) //right motor is faster
			{
				reverse_speed_left+=factor;  //increment speed of left motor
				reverse_speed_right-=factor; //decrement speed of right motor
				velocity(reverse_speed_left,reverse_speed_right);
			}
			
			if(time_right>time_left) //left motor is faster
			{
				reverse_speed_left-=factor;   //decrement speed of left motor
				reverse_speed_right+=factor;  //increment speed of right motor
				velocity(reverse_speed_left,reverse_speed_right);
			}
		}
	}
	
// 	if(adjust_count == 1000)
// 	{
// 		adjust_count = 0;
// 		save_speeds_to_eeprom();
// 		save_correct_linear_speeds_to_eeprom();
// 		save_correct_turn_speeds_to_eeprom();
// 	}
	
	//Reinitialize encoder feedback corrections
	time_right = 0;
	time_left = 0;
	CorrectCountRight = 0;
	CorrectCountLeft = 0;
	flag_right = 0;
	flag_left = 0;
	StopWatch = 0;
}

/*
* Function Name: stop_encoder_feedback_correction
* Input: None
* Output: Stops the encoder feedback correction
* Logic: Stop encoder feedback correction by making factor 0
* Example Call: stop_encoder_feedback_correction()
*/
void stop_encoder_feedback_correction(void)
{
	factor = 0; 
	stop_correction_stopwatch = stop_time; //So it doesn't start again
}

/*
* Function Name: start_encoder_feedback_correction_later
* Input: None
* Output: Stops the encoder feedback correction and starts it after a while
* Logic: stop_correction_stopwatch = 0; encoder feedback correction will start again when stop_correction_stopwatch = stop_time
* Example Call: start_encoder_feedback_correction_later()
*/
void start_encoder_feedback_correction_later()
{
	stop_encoder_feedback_correction();
	stop_correction_stopwatch = 0;
}

/*
* Function Name: start_encoder_feedback_correction_later
* Input: None
* Output: Start encoder feedback correction
* Logic: Reinitialize encoder feedback correction
* Example Call: start_encoder_feedback_correction_now()
*/
void start_encoder_feedback_correction_now(void)
{
	stop_correction_stopwatch=stop_time; //just in case
	
	//Reinitialize encoder feedback correction
	time_right = 0;
	time_left = 0;
	CorrectCountRight = 0;
	CorrectCountLeft = 0;
	flag_right = 0;
	flag_left = 0;
	StopWatch = 0;
	factor = factor_initial_value; //Restart encoder feedback corrections
}

//Prints all speed to LCD
void print_all_speeds()
{
	lcd_print(1,1,forward_speed_left,3);
	lcd_print(1,8,forward_speed_right,3);
	lcd_print(2,1,reverse_speed_left,3);
	lcd_print(2,8,reverse_speed_right,3);
}

/*
* Function Name: CheckIfBothSpeedsAreProper
* Input: left_count, right_count -> counts of left and right position encoders
* Output: correction speeds are adjusted to make them equal and of desired speeds
* Logic: Increment/Decrement speeds based on count
* Example Call: CheckIfBothSpeedsAreProper(2, 3)
*/
void CheckIfBothSpeedsAreProper(int left_count, int right_count)
{	
				if (left_count < 2 && right_count<2)
				{
					CorrectLinearSpeedLeft+=1;
					CorrectLinearSpeedRight+=1;
					velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
// 					lcd_print(1,1,CorrectLinearSpeedLeft,3);
// 					lcd_print(1,8,CorrectLinearSpeedRight,3);
// 					lcd_cursor(2,1);
// 					lcd_string("Inc&Inc Linear");
				}	
				
				else if (left_count>2 && right_count>2)
				{
					CorrectLinearSpeedLeft-=1;
					CorrectLinearSpeedRight-=1;
					velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
// 					lcd_print(1,1,CorrectLinearSpeedLeft,3);
// 					lcd_print(1,8,CorrectLinearSpeedRight,3);
// 					lcd_cursor(2,1);
// 					lcd_string("Dec&Dec Linear");
				}
				
				else if (left_count > 2 && right_count<2)
				{
					CorrectLinearSpeedLeft-=1;
					CorrectLinearSpeedRight+=1;
					velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
// 					lcd_print(1,1,CorrectLinearSpeedLeft,3);
// 					lcd_print(1,8,CorrectLinearSpeedRight,3);
// 					lcd_cursor(2,1);
// 					lcd_string("Dec&Inc Linear");
				}
				
				else if (left_count < 2 && right_count>2)
				{
					CorrectLinearSpeedLeft+=1;
					CorrectLinearSpeedRight-=1;
					velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
// 					lcd_print(1,1,CorrectLinearSpeedLeft,3);
// 					lcd_print(1,8,CorrectLinearSpeedRight,3);
// 					lcd_cursor(2,1);
// 					lcd_string("Inc&Dec Linear");
				}
}

/*
* Function Name: CheckIfBothTurnSpeedsAreProper
* Input: left_count, right_count -> counts of left and right position encoders
* Output: correction speeds are adjusted to make them equal and of desired speeds
* Logic: Increment/Decrement speeds based on count
* Example Call: CheckIfBothTurnSpeedsAreProper(2, 3)
*/
void CheckIfBothTurnSpeedsAreProper(int left_count, int right_count)
{
	if (left_count<2 && right_count<2)
	{
		CorrectTurnSpeedLeft+=1;
		CorrectTurnSpeedRight+=1;
		velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
// 		lcd_print(1,1,CorrectTurnSpeedLeft,3);
// 		lcd_print(1,8,CorrectTurnSpeedRight,3);
// 		lcd_cursor(2,1);
// 		lcd_string("Inc&Inc  TURN ");
	}
	
	else if (left_count>2 && right_count>2)
	{
		CorrectTurnSpeedLeft-=1;
		CorrectTurnSpeedRight-=1;
		velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
// 		lcd_print(1,1,CorrectTurnSpeedLeft,3);
// 		lcd_print(1,8,CorrectTurnSpeedRight,3);
// 		lcd_cursor(2,1);
// 		lcd_string("Dec&Dec  TURN ");
	}
	
	else if (left_count > 2 && right_count<2)
	{
		CorrectTurnSpeedLeft-=1;
		CorrectTurnSpeedRight+=1;
		velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
// 		lcd_print(1,1,CorrectTurnSpeedLeft,3);
// 		lcd_print(1,8,CorrectTurnSpeedRight,3);
// 		lcd_cursor(2,1);
// 		lcd_string("Dec&Inc  TURN ");
	}
	
	else if (left_count<2 && right_count>2)
	{
		CorrectTurnSpeedLeft+=1;
		CorrectTurnSpeedRight-=1;
		velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
// 		lcd_print(1,1,CorrectTurnSpeedLeft,3);
// 		lcd_print(1,8,CorrectTurnSpeedRight,3);
// 		lcd_cursor(2,1);
// 		lcd_string("Inc&Dec  TURN ");
	}
}