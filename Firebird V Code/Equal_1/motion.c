/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: motion.c
* Theme: Launch a module
* Functions:
		forward (void),
		back (void),
		left (void),
		right (void),
		stop (void),
		soft_left (void),
		soft_right (void),
		soft_left_2 (void),
		soft_right_2 (void),
		forward_mm(unsigned int),
		back_mm(unsigned int),
		left_degrees(unsigned int),
		right_degrees(unsigned int),
		soft_left_degrees(unsigned int),
		soft_right_degrees(unsigned int),
		soft_left_2_degrees(unsigned int),
		soft_right_2_degrees(unsigned int)
* Global Variables: 
	slow_linear_speed_left
	slow_linear_speed_right
	TurnSpeedLeft
	TurnSpeedRight
	slow_turn_speed_left
	slow_turn_speed_right
	deaccelerate_for_degrees
*/

#include "main.h"

unsigned int TurnSpeedLeft = 200;
unsigned int TurnSpeedRight = 200;

//Used for deacceleration
unsigned int slow_turn_speed_left = 100; 
unsigned int slow_turn_speed_right = 100;
int deaccelerate_for_degrees = 30;

extern int accleration_off_flag;

//correction angles
extern unsigned int very_tiny_angle;
extern unsigned int tiny_angle;

extern const unsigned char done; //done command
extern volatile unsigned int CorrectCountLeft;  //to keep track of left position encoder
extern volatile unsigned int CorrectCountRight; //to keep track of right position encoder
extern volatile unsigned int TurnCountRight;	 //no of encoder pulses while turning
extern volatile unsigned int TurnCountLeft;	 //no of encoder pulses while turning
extern volatile unsigned int LinearCountRight;  //no of encoder pulses while moving linearly
extern volatile unsigned int LinearCountLeft;	 //no of encoder pulses while moving linearly

extern volatile unsigned char data;			 //to store received data from UDR1

extern unsigned int stop_correction_stopwatch;//This variable is used to measure the time taken for the wheels to rotate by a specified number of encoder pulses
extern volatile unsigned int factor;		  //used to turn off encoder speed corrections(off when 0), same as factor_initial_value

extern volatile unsigned int forward_speed_left;  //Stores the speed of the left motor
extern volatile unsigned int forward_speed_right; //Stores the speed of the right motor
extern volatile unsigned int reverse_speed_left;  //Stores the speed of the left motor
extern volatile unsigned int reverse_speed_right; //Stores the speed of the right motor

//Here used for deacceleration
extern unsigned int CorrectLinearSpeedLeft;
extern unsigned int CorrectLinearSpeedRight;

extern volatile int flag_instruction;		   //no of instructions pending
extern int adjust_count;

unsigned int DeaccelerationReqdShaftCountInt = 2*80 / 5.338;  //Vary 2*X/5.338
unsigned int slow_linear_speed_left = 100;
unsigned int slow_linear_speed_right = 100;

/*
* Function Name motion_set
* Input: Direction -> 8-bit value corresponding to the direction of the motors
* Output: Motors spin in the specified direction
* Logic: Sets the lower nibble of PORTA to the lower nibble of Direction
		 Lower nibble of a Port a controls the direction of the motors since they ate connected to the direction pins of L293d
* Example Call: motion_set(0x06) -> for forward
*/
static void motion_set(unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibble as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibble to 0
	PortARestore |= Direction; 	// adding lower nibble for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}

/*
* Function Name:forward
* Input: None
* Output: The bot moves forward
* Logic: Writes 0x06 to lower nibble of PORTA xxxx 0110
* Example Call: forward()
*/
void forward (void)
{
	motion_set(0x06);
}

/*
* Function Name:back
* Input: None
* Output: The bot moves backwards
* Logic: Writes 0x09 to lower nibble of PORTA xxxx 1001
* Example Call: back()
*/
void back (void)
{
	motion_set(0x09);
}

/*
* Function Name:left
* Input: None
* Output: The bot turns anticlockwise, ie Left wheel backward, Right wheel forward
* Logic: Writes 0x05 to lower nibble of PORTA xxxx 0101
* Example Call: left()
*/
void left (void)
{
	motion_set(0x05);
}

/*
* Function Name:right
* Input: None
* Output: The bot turns clockwise, ie Left wheel forward, Right wheel backward
* Logic: Writes 0x0A to lower nibble of PORTA xxxx 1011
* Example Call: right()
*/
void right (void)
{
	motion_set(0x0A);
}

/*
* Function Name:soft_left
* Input: None
* Output: The bot turns counter-clockwise, ie Left wheel stationary, Right wheel forward
* Logic: Writes 0x04 to lower nibble of PORTA xxxx 0100
* Example Call: soft_left()
*/
void soft_left (void)
{
	motion_set(0x04);
}

/*
* Function Name:soft_right
* Input: None
* Output: The bot turns clockwise, ie Left wheel forward, Right wheel is stationary
* Logic: Writes 0x02 to lower nibble of PORTA xxxx 0010
* Example Call: soft_right()
*/
void soft_right (void)
{
	motion_set(0x02);
}

/*
* Function Name:soft_left_2
* Input: None
* Output: The bot turns counter-clockwise, ie Left wheel backward, right wheel stationary
* Logic: Writes 0x01 to lower nibble of PORTA xxxx 0001
* Example Call: soft_left_2()
*/
void soft_left_2 (void)
{
	motion_set(0x01);
}

/*
* Function Name:soft_right_2
* Input: None
* Output: The bot turns clockwise, ie Left wheel stationary, Right wheel backward
* Logic: Writes 0x08 to lower nibble of PORTA xxxx 1000
* Example Call: soft_right_2()
*/
void soft_right_2 (void)
{
	motion_set(0x08);
}

/*
* Function Name:angle_rotate
* Input: Degrees -> No of degrees to turn
* Output: Stops the bot after it turns the specified number of degrees
* Logic: 1.Convert the degrees to rotate into a no of encoder pulses
		 2.Set TurnCount to 0
		 3.Wait for TurnCount to become equal to the required count or "stop" is received via the XBee module
		 4.Stop the Bot
* Example Call: angle_rotate(90)
*/
static void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned int ReqdShaftCountInt = 0;
	
	ReqdShaftCount = 2*(float) Degrees/ 4.090; //division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	TurnCountRight = 0;
	TurnCountLeft = 0;
	int deaccelerate_flag = 0;
	int deaccelerate_count = 0;
	int deaccelerate_degrees = 0;
	int average_count = 0;
	
	//Calculate the number of encoder pulses after which to deaccelerate
	if(Degrees >= 45)
	{
		velocity(TurnSpeedLeft, TurnSpeedRight);
		deaccelerate_degrees = Degrees-deaccelerate_for_degrees;
		deaccelerate_flag = 1;
		deaccelerate_count = 2*(float) deaccelerate_degrees/ 4.090; // division by resolution to get shaft count
		deaccelerate_count = (unsigned int) deaccelerate_count;
	}

	while(data != 0x20) //flag_instruction has been decremented in the USART interrupt if data = 0x20
	{
		average_count = (TurnCountRight+TurnCountLeft)/2;
		if(average_count >= ReqdShaftCountInt)
		break;

		//Deaccelerate by slowing speed
		if(deaccelerate_flag == 1 && ((TurnCountRight >= deaccelerate_count) || (TurnCountLeft >= deaccelerate_count)))
		{
			velocity(slow_turn_speed_left, slow_turn_speed_right);
			deaccelerate_flag = 0;
		}
	}

	stop(); //Stop robot

	unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot

	if(Degrees>=70 && var == 0x06)
	{
		velocity(forward_speed_left, forward_speed_right);
	}
	else if(Degrees>=70 && var == 0x09)
	{
		velocity(reverse_speed_left, reverse_speed_right);
	}
}

/*
* Function Name:linear_distance_mm
* Input: DistanceInMM -> distance to move
* Output: Stops the bot after it moves the specified distance
* Logic: 1.Convert the distance to move into a no of encoder pulses
		 2.Set LinearCount to 0
		 3.Wait for LinearCount to become equal to the required count or "stop" is received via the XBee module
		   -Check and execute corrections received via XBee
		 4.Stop the Bot
* Example Call: linear_distance_mm(1000) -> Move one meter
*/
static void linear_distance_mm(unsigned int DistanceInMM)
{
		LinearCountRight = 0;
		LinearCountLeft = 0;
	stop_encoder_feedback_correction(); //During start and during acceleration 
	
	float ReqdShaftCount = 0;
	unsigned int ReqdShaftCountInt = 0;

	ReqdShaftCount = 2*DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;

	unsigned char var = PORTA & 0x0F;  //Mask the upper nibble to get the direction of the bot
	/*Accelerate upto forward_speed_left & forward_speed_right to eliminate the slippage of the wheels
	  and hence avoid offsetting the angle of the bot.
	  When the bot starts with full speed directly due to the varying friction on the wheels
	  and the different motor properties the bot experiences a change in its angle.
	*/
	if (var==0x06)
	{
		//Acceleration
		if(DistanceInMM>=60 && accleration_off_flag != 1)
		{
// 			float left_inc = forward_speed_left/25;
// 			float right_inc = forward_speed_right/25;
// 
// 			for(int i=0; i<=25;i++)
// 			{
// 				int left_var = left_inc*i;
// 				int right_var = right_inc*i;
// 
// 				velocity(left_var,right_var);
// 				_delay_ms(20);
// 			}
			velocity(forward_speed_left,forward_speed_right);
		}
		else if(accleration_off_flag != 1)
		velocity(forward_speed_left,forward_speed_right);
	}

	if(var==0x09)
	{
		//Acceleration
		if(DistanceInMM>=60 && accleration_off_flag != 1)
		{
// 			float left_inc = reverse_speed_left/25;
// 			float right_inc = reverse_speed_right/25;
// 
// 			for(int i=0; i<=25;i++)
// 			{
// 				int left_var = left_inc*i;
// 				int right_var = right_inc*i;
// 
// 				velocity(left_var,right_var);
// 				_delay_ms(20);
// 			}
			velocity(reverse_speed_left,reverse_speed_right);
		}
	 
		else if(accleration_off_flag != 1)
		velocity(reverse_speed_left,reverse_speed_right);
	}

	int average_count = 0;
	
	start_encoder_feedback_correction_later();	//After .5-1 seconds maybe?

	//Wait for the LinearCountRight to be equal to the ReqdShaftCountInt
	while(data != 0x20) //flag_instruction has been decremented in the USART interrupt if data = 0x20
	{
		average_count = (LinearCountRight+LinearCountLeft)/2;
		
		//Deacceleration
		if(average_count > (ReqdShaftCountInt-DeaccelerationReqdShaftCountInt))
		{
			stop_encoder_feedback_correction();
			//velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
			velocity(slow_linear_speed_left,slow_linear_speed_right);
		}
		
		if(average_count > ReqdShaftCountInt)
		{
			break;
		}

		if(flag_instruction > 1)//If any data is received
		{
			//This is to check for if any corrections are received if the bot has deviated from its path
			if(data == 0x6B)  //ASCII value of k
			{
				correct_left(tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x6C)  //ASCII value of l
			{
				correct_right(tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x6F)  //ASCII value of o //does not work
			{
				correct_left(very_tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x70)  //ASCII value of p //does not work
			{
				correct_right(very_tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x4B)  //ASCII value of K
			{
				correct_left_2(tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x4C)  //ASCII value of L
			{
				correct_right_2(tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x4F)  //ASCII value of O
			{
				correct_left_2(very_tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}

			else if(data == 0x50)  //ASCII value of P
			{
				correct_right_2(very_tiny_angle);
				_delay_ms(50);
				UDR0 = done;
				flag_instruction--;
			}
		}
	}
	
	
	stop(); //Stop robot
	
	if(DistanceInMM >= 1000)
	{
		//adjust_count = 0;
		save_speeds_to_eeprom();
		//save_correct_linear_speeds_to_eeprom();
		//save_correct_turn_speeds_to_eeprom();
	}
}

/*
* Function Name:forward_mm
* Input: DistanceInMM -> distance to move forward
* Output: Moves the bot forward by the specified distance
* Logic: 1.Move forward
		 2.Stop after specified distance is reached
* Example Call: forward_mm(1000) -> Move forward one meter
*/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

/*
* Function Name:back_mm
* Input: DistanceInMM -> distance to move back
* Output: Moves the bot backwards by the specified distance
* Logic: 1.Move backwards
		 2.Stop after specified distance is reached
* Example Call: back_mm(1000) -> Move back one meter
*/
void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

/*
* Function Name: left_degrees
* Input: Degrees -> Angle to rotate
* Output: Turns the bot left by the specified degrees, ie Left wheel backward, Right wheel forward
* Logic: 1.Turn counter-clockwise
		 2.Stop after specified degrees is reached
* Example Call: left_degrees(90) -> Turn left by 90 degrees
*/
void left_degrees(unsigned int Degrees)
{
	left(); //Turn left
	angle_rotate(Degrees);
}

/*
* Function Name: right_degrees
* Input: Degrees -> Angle to rotate
* Output: Turns the bot right by the specified degrees, ie right wheel backward, left wheel forward
* Logic: 1.Turn clockwise
		 2.Stop after specified degrees is reached
* Example Call: right_degrees(90) -> Turn right by 90 degrees
*/
void right_degrees(unsigned int Degrees)
{
	right(); //Turn right
	angle_rotate(Degrees);
}

/*
* Function Name: soft_left_degrees
* Input: Degrees -> Angle to rotate
* Output: Turns the bot left by the specified degrees, ie Right wheel forward only
* Logic: 1.Turn
		 2.Stop after specified degrees is reached
* Example Call: soft_left_degrees(90) -> Turn left by 90 degrees
*/
void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*
* Function Name: soft_right_degrees
* Input: Degrees -> Angle to rotate
* Output: Turns the bot right by the specified degrees, ie Left wheel forward only
* Logic: 1.Turn
		 2.Stop after specified degrees is reached
* Example Call: soft_right_degrees(90) -> Turn right by 90 degrees
*/
void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*
* Function Name: soft_left_2_degrees
* Input: Degrees -> Angle to rotate
* Output: Turns the bot left by the specified degrees, ie Left wheel backwards only
* Logic: 1.Turn
		 2.Stop after specified degrees is reached
* Example Call: soft_left_2_degrees(90) -> Turn left by 90 degrees
*/
void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*
* Function Name: soft_right_2_degrees
* Input: Degrees -> Angle to rotate
* Output: Turns the bot right by the specified degrees, ie Right wheel backwards only
* Logic: 1.Turn
		 2.Stop after specified degrees is reached
* Example Call: soft_right_2_degrees(90) -> Turn left by 90 degrees
*/
void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

/*
* Function Name: stop
* Input: Degrees -> Angle to rotate
* Output: Stops the bot
* Logic: Stop the motors
* Example Call: stop()
*/
void stop (void)
{
	motion_set(0x00);
	//velocity(0,0);
}