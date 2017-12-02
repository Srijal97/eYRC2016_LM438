/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: interrupt.c
* Theme: Launch a module
* Functions: None
* ISRs:  ISR(INT5_vect), ISR(INT4_vect), ISR(TIMER5_OVF_vect)
* Global Variables: stop_time, stop_correction_stopwatch
*/

#include "main.h"

const int stop_time = 500; //Variable to turn off the encoder feedback correction for a set amount of time (time = stop_time/225)
volatile unsigned int stop_correction_stopwatch = 0; //Counter to count if the no of ticks has reached the stop_time

extern volatile int flag_right;					//flag to check if right encoder has generated "count" no of pulses
extern volatile int flag_left;					//flag to check if left encoder has generated "count" no of pulses
extern volatile unsigned int count;			    //no of encoder pulses to be generated between each speed correction
extern volatile unsigned int CorrectCountLeft;  //to keep track of left position encoder
extern volatile unsigned int CorrectCountRight; //to keep track of right position encoder
extern volatile unsigned int TurnCountRight;	//no of encoder pulses while turning
extern volatile unsigned int TurnCountLeft;		//no of encoder pulses while turning
extern volatile unsigned int LinearCountRight;  //no of encoder pulses while moving linearly
extern volatile unsigned int LinearCountLeft;	//no of encoder pulses while moving linearly
extern volatile unsigned int StopWatch;			//used to measure the time taken for the wheels to rotate by a specified number of encoder pulses
extern volatile unsigned int StopWatch2;	
extern volatile unsigned int time_left;			//no of "stopwatch" ticks taken for "CorrectCountLeft" to reach "count"
extern volatile unsigned int time_right;		//no of "stopwatch" ticks taken for "CorrectCountRight" to reach "count"
extern volatile unsigned int factor;			//used to turn off encoder speed corrections(off when 0), same as factor_initial_value
extern const unsigned int factor_initial_value; //factor by which the speed of the motor is changed on each speed correction
volatile unsigned int SlowTurnCountLeft = 0;	//no of encoder pulses while moving linearly slowly
volatile unsigned int SlowLinearCountLeft = 0;	//no of encoder pulses while moving linearly slowly
volatile unsigned int SlowTurnCountRight = 0;	//no of encoder pulses while turning slowly
volatile unsigned int SlowLinearCountRight = 0;	////no of encoder pulses while turning slowly
/*
* ISR for right position encoder
* The input from the right position encoder is connected to PORTE5 which is INT5
* Function: Increments various counters for measuring linear distance, angular distance and speed
* Logic: INT5 is set to trigger with falling edge.
		 On each trigger TurnCountRight & LinearCountRight are incremented
		 CorrectCountRight is incremented only when flag_right!=1 && bot is going forward or reverse
*/
ISR(INT5_vect)
{
	TurnCountRight++;
	LinearCountRight++;
	SlowTurnCountRight++;
	SlowLinearCountRight++;
	
	 volatile unsigned char var1 = PORTA & 0x0F;
	if (flag_right!=1 && (var1==0x06 || var1==0x09)) //If going forward or reverse && flag_right!=1
	{
		CorrectCountRight++;
		
		if(CorrectCountRight==count)  //Correct speed when CorrectCountRight=count
		{
			flag_right = 1;			 //CorrectCountRight has reach the required no of counts
			time_right = StopWatch;  //Took this many ticks to reach the required no of counts
			if (flag_left == 1)		 //If both wheels have reached the required no of counts
			{
				modify_speed();  
			}
		}
	}
}

/*
* ISR for right position encoder
* The input from the right position encoder is connected to PORTE4 which is INT4
* Function: Increments various counters for measuring linear distance, angular distance and speed
* Logic: INT4 is set to trigger with falling edge.
		 On each trigger TurnCountLeft & LinearCountLeft are incremented
		 CorrectCountLeft is incremented only when flag_left!=1 && bot is going forward or reverse
*/
ISR(INT4_vect)
{
	TurnCountLeft++;
	LinearCountLeft++;
	SlowTurnCountLeft++;
	SlowLinearCountLeft++;
	
	volatile unsigned char var2 = PORTA & 0x0F;
	if(flag_left != 1 && (var2==0x06 || var2==0x09)) //If going forward or reverse && flag_left!=1
	{
		CorrectCountLeft++;
		
		if(CorrectCountLeft==count) //Correct speed when CorrectCountLeft=count
		{
			flag_left = 1;			//CorrectCountLeft has reach the required no of counts
			time_left = StopWatch;  //Took this many ticks to reach the required no of counts
			if (flag_right == 1)	//If both wheels have reached the required no of counts
			{
				modify_speed();
			}
		}
	}
}

/*
* Function: This ISR is used for timing timing purposes
* Logic: This ISR is triggered when timer 5 overflows
		 Timer 5 is a 225 Hz timer
		 Therefore this ISR is triggered 225 times in one second
*/
ISR(TIMER5_OVF_vect)
{
	volatile unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	StopWatch2++;
	
	if(var==0x06 || var==0x09) //only if forward or reverse
	{
		StopWatch++; //This variable is used to measure the time taken for the wheels to rotate by a specified number of encoder pulses
	}
			
	if(stop_correction_stopwatch < stop_time) //If encoder feedback corrections are stopped
	{
		stop_correction_stopwatch++;  
		if(stop_correction_stopwatch==stop_time) //If it is time to start encoder feedback corrections again
		{
			start_encoder_feedback_correction_now();
		}
	}	
}