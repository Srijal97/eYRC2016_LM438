/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: main.c
* Theme: Launch a module
* Functions: init_ports(), init_devices(), int main()
* Global Variables: done, done_linear, correct_forward_dist, correct_forward_small_dist, correct_reverse_dist,
					correct_reverse_small_dist, very_tiny_angle, tiny_angle, stopping_dist, left_turn_stopping_angle,
					right_turn_stopping_angle, left_180_turn_stopping_angle, right_180_turn_stopping_angle,
					data, flag_instruction, CorrectCountLeft, CorrectCountRight, TurnCountRight, TurnCountLeft, LinearCountRight, LinearCountLeft,
					StopWatch, time_right, time_left, flag_right, flag_left
*/

#include "main.h"

unsigned int turn_dist_linear = 42;

int accleration_off_flag = 0;

//constants to set for correction distances
	const unsigned int correct_forward_dist = 5;
	const unsigned int correct_forward_small_dist = 3; //Cannot decrease any more than 3 
	const unsigned int correct_reverse_dist = 5;
	const unsigned int correct_reverse_small_dist = 3; //Cannot decrease any more than 3

	unsigned int very_tiny_angle = 4;
	unsigned int tiny_angle = 5;

	int CorrectTurnSpeedFactor = 150; //Increase for slower speeds
	unsigned int CorrectTurnSpeedLeft = 80;
	unsigned int CorrectTurnSpeedRight = 80;
	int CorrectLinearSpeedFactor = 75; //Increase for slower speeds
	unsigned int CorrectLinearSpeedLeft = 90; //Not used since speed values loaded from eeprom
	unsigned int CorrectLinearSpeedRight = 90;//Not used since speed values loaded from eeprom
	
//Variables to calibrate turn angle and liner distance
	int stopping_dist = 24;
	int left_turn_stopping_angle = 10;
	int right_turn_stopping_angle = 10;
	int left_180_turn_stopping_angle = 10;
	int right_180_turn_stopping_angle = 10;

extern volatile unsigned int forward_speed_left;  //Stores the speed of the left motor
extern volatile unsigned int forward_speed_right; //Stores the speed of the right motor
extern volatile unsigned int reverse_speed_left;  //Stores the speed of the left motor
extern volatile unsigned int reverse_speed_right; //Stores the speed of the right motor

volatile unsigned int CorrectCountLeft = 0;  //to keep track of left position encoder
volatile unsigned int CorrectCountRight = 0; //to keep track of right position encoder
volatile unsigned int TurnCountRight = 0;	 //no of encoder pulses while turning
volatile unsigned int TurnCountLeft = 0;	 //no of encoder pulses while turning
volatile unsigned int LinearCountRight = 0;  //no of encoder pulses while moving linearly
volatile unsigned int LinearCountLeft = 0;	 //no of encoder pulses while moving linearly
volatile unsigned int StopWatch = 0;		 //This variable is used to measure the time taken for the wheels to rotate by a specified number of encoder pulses
volatile unsigned int StopWatch2 = 0;
volatile unsigned int time_right = 0;		 //no of "stopwatch" ticks taken for "CorrectCountRight" to reach "count"
volatile unsigned int time_left = 0;		 //no of "stopwatch" ticks taken for "CorrectCountLeft" to reach "count"
volatile int flag_right = 0;				 //flag to check if right encoder has generated "count" no of pulses
volatile int flag_left = 0;					 //flag to check if left encoder has generated "count" no of pulses
volatile int start = 1;

extern volatile unsigned int SlowTurnCountRight;	//no of encoder pulses while turning
extern volatile unsigned int SlowTurnCountLeft;		//no of encoder pulses while turning
extern volatile unsigned int SlowLinearCountRight;  //no of encoder pulses while moving linearly
extern volatile unsigned int SlowLinearCountLeft;	//no of encoder pulses while moving linearly

extern volatile unsigned int stop_correction_stopwatch; //Counter to count if the no of ticks has reached the stop_time
extern volatile unsigned int factor;					//used to turn off encoder speed corrections(off when 0), same as factor_initial_value

volatile unsigned char data;				 //to store received data from UDR0
volatile int flag_instruction = 0;

//Done command which are sent to python code
const unsigned char done = 0x7E;
const unsigned char done_linear = 0x7C;

//Variables which store the angles of the gripper
extern int horizontal_arm_angle;
extern int required_horizontal_arm_angle;
extern int vertical_arm_angle;
extern int required_vertical_arm_angle;

//Variables which store the extreme angles of the gripper
extern unsigned char vertical_upper_limit;
extern unsigned char vertical_lower_limit;
extern unsigned char horizontal_upper_limit;
extern unsigned char horizontal_lower_limit;
extern unsigned char vertical_resting_angle;
extern unsigned char back_vertical_upper_limit;
extern unsigned char back_vertical_lower_limit;
extern unsigned char back_horizontal_upper_limit;
extern unsigned char back_horizontal_lower_limit;
extern unsigned char back_vertical_resting_angle;

int ticktick = 150;

/*
* Function Name:init_ports
* Input: None
* Output: None
* Logic: Calls the functions which will initializes the data direction registers
		 of the ports for various peripherals
		 ie: 1)Direction pins of motor-driver ic
		     2)Encoders
			 3)LCD
			 4)Mosfet for turning on/off various sensors
* Example Call: init_ports()
*/
void init_ports()
{
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	lcd_port_config();
	MOSFET_switch_config();
	buzzer_pin_config();
	//spi_pin_config();
}

/*
* Function Name:init_devices
* Input: None
* Output: None
* Logic: Calls the functions which will initialize the various components of the robot
		 ie: 1)All the ports
		     2)Left an right encoders
			 3)Timers for motor speed control and servo control
			 4)LCD
* Example Call: init_devices()
*/
void init_devices()
{
	cli(); //Clears the global interrupt
	init_ports(); //Initializes all the ports
	servo_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	timer1_init();
	timer4_init();
	timer5_init();
	uart0_init(); //Initailize UART1 for serial communiaction
	lcd_start();
	//spi_init();
	TIMSK5|=0x01;
	sei();   // Enables the global interrupt
}

/*
* Function Name:main
* Input: None
* Output: None
* Logic: Main function of the program.
		 Main function checks to see if any data is received serially over the XBee modules i a while(1) loop.
		 If any commands are received it executes the function corresponding to the ASCII value of the char data.
* Example Call: main()
*/
int main()
{
	buzzer_beep(1);
	init_devices();
	turn_off_all_proxy_sensors();
	load_speeds_from_eeprom();
	load_correct_linear_speeds_from_eeprom();
	load_correct_turn_speeds_from_eeprom();
	velocity(forward_speed_left,forward_speed_right);
	
	back_close_jaws();
	
	vertical_servo(vertical_resting_angle);
	back_vertical_servo(back_vertical_resting_angle);
	_delay_ms(1000);
	vertical_servo_free();
	back_vertical_servo_free();
	
	open_jaws();
	
	while(1)
	{
		if(flag_instruction)
		{	
				if(data == 0x20) //ASCII value of space
				{
					stop(); //stop
					
				}
			
			else if(data == 0x6B)  //ASCII value of k
			{
				correct_left(tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x6C)  //ASCII value of l
			{
				correct_right(tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x6F)  //ASCII value of o
			{
				correct_left(very_tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x70)  //ASCII value of p
			{
				correct_right(very_tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x4B)  //ASCII value of K
			{
				correct_left_2(tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x4C)  //ASCII value of L
			{
				correct_right_2(tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x4F)  //ASCII value of O
			{
				correct_left_2(very_tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x50)  //ASCII value of P
			{
				correct_right_2(very_tiny_angle);
				//_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x74) //ASCII value of t
			{
				velocity(ticktick,ticktick);
				forward_mm(correct_forward_dist);
				//_delay_ms(50);
				UDR0 = done_linear;
			}

			else if(data == 0x79) //ASCII value of y
			{
				velocity(ticktick,ticktick);
				forward_mm(correct_forward_small_dist);
				//_delay_ms(50);
				UDR0 = done_linear;
			}

			else if(data == 0x67) //ASCII value of g
			{
				velocity(ticktick,ticktick);
				back_mm(correct_reverse_dist);
				//_delay_ms(50);
				UDR0 = done_linear;
			}

			else if(data == 0x68) //ASCII value of h
			{
				velocity(ticktick,ticktick);
				back_mm(correct_reverse_small_dist);
				//_delay_ms(50);
				UDR0 = done_linear;
			}

			else if(data == 0x77) //ASCII value of w
			{
				velocity(forward_speed_left, forward_speed_right);
				forward();  //forward
			}

			else if(data == 0x65) //ASCII value of e
			{
				accleration_off_flag = 1;
				velocity(forward_speed_left*.75,forward_speed_right*.75); //set speed
				forward_mm(turn_dist_linear);
				_delay_ms(300);
				left_degrees(90-left_turn_stopping_angle);
				_delay_ms(300);
				velocity(reverse_speed_left*.75,reverse_speed_right*.75);
				back_mm(turn_dist_linear);//reverse
				_delay_ms(300);
				accleration_off_flag = 0;
				
				UDR0 = done;
			}

			else if(data == 0x72) //ASCII value of r
			{
				accleration_off_flag = 1;
				velocity(forward_speed_left*.75,forward_speed_right*.75); //set speed
				forward_mm(turn_dist_linear);
				_delay_ms(300);
				right_degrees(90-left_turn_stopping_angle);
				_delay_ms(300);
				velocity(reverse_speed_left*.75,reverse_speed_right*.75);
				back_mm(turn_dist_linear);//reverse
				_delay_ms(300);
				accleration_off_flag = 0;
				
				UDR0 = done;
			}

			else if(data == 0x73) //ASCII value of s
			{
				velocity(reverse_speed_left, reverse_speed_right);
				back(); //back
			}
	//##########################################		
			
						else if(data == 0x54) //ASCII value of T
						{
							stop_encoder_feedback_correction();
							velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
							right();
						}

						else if(data == 0x47) //ASCII value of G
						{
							stop_encoder_feedback_correction();
							velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
							left();
						}

						else if(data == 0x45) //ASCII value of E
						{
							stop_encoder_feedback_correction();
							velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
							forward();
						}
						
						else if(data == 0x46) //ASCII value of F
						{
							stop_encoder_feedback_correction();
							velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
							back();
						}			
	//##########################################		
			else if(data == 0x3E) //ASCII value of >
			{
				stop_encoder_feedback_correction();
				velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
				right();
				_delay_ms(200);
				
				while (data == 0x3E)
				{				
					SlowTurnCountLeft = 0;
					SlowTurnCountRight = 0;
					StopWatch2 = 0;
					
					while(StopWatch2<CorrectTurnSpeedFactor && data == 0x3E);	//StopWatch2<1xx :Increase xx for slower speeds
					CheckIfBothTurnSpeedsAreProper(SlowTurnCountLeft, SlowTurnCountRight);
				}
				
				if(data == 0x20) //ASCII value of space
				{
					stop(); //stop
					//flag_instruction--;
				}
			}

			else if(data == 0x3C) //ASCII value of <
			{
				stop_encoder_feedback_correction();
				velocity(CorrectTurnSpeedLeft,CorrectTurnSpeedRight);
				left();
				_delay_ms(200);
				
				while (data == 0x3C)
				{
					SlowTurnCountLeft = 0;
					SlowTurnCountRight = 0;
					StopWatch2 = 0;
					
					while(StopWatch2<CorrectTurnSpeedFactor && data == 0x3C);	//StopWatch2<1xx :Increase xx for slower speeds
					CheckIfBothTurnSpeedsAreProper(SlowTurnCountLeft, SlowTurnCountRight);
				}
				
				if(data == 0x20) //ASCII value of space
				{
					stop(); //stop
					//flag_instruction--;
				}
			}

			else if(data == 0x3A) //ASCII value of :
			{
				stop_encoder_feedback_correction();
				velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
				forward();
				//_delay_ms(200);	
					
				//while(data != 0x20)
				while (data == 0x3A)
				{
					SlowLinearCountLeft = 0;
					SlowLinearCountRight = 0;
					StopWatch2 = 0;
					
					while(StopWatch2<CorrectLinearSpeedFactor && data == 0x3A);	//StopWatch2<1xx :Increase xx for slower speeds
					//_delay_ms(50);	
					CheckIfBothSpeedsAreProper(SlowLinearCountLeft, SlowLinearCountRight);
				}
				
				if(data == 0x20) //ASCII value of space
				{
					stop(); //stop
				}
			}
			
			else if(data == 0x3B) //ASCII value of ;
			{
				stop_encoder_feedback_correction();
				velocity(CorrectLinearSpeedLeft,CorrectLinearSpeedRight);
				back();
				//_delay_ms(200);	
							
				//while(data != 0x20)
				while(data == 0x3B)
				{
						SlowLinearCountLeft = 0;
						SlowLinearCountRight = 0;
						StopWatch2 = 0;
						
						while(StopWatch2<CorrectLinearSpeedFactor && data == 0x3B); //StopWatch2<1xx :Increase xx for slower speeds
						//_delay_ms(50);
						CheckIfBothSpeedsAreProper(SlowLinearCountLeft, SlowLinearCountRight);
				}
				
				if(data == 0x20) //ASCII value of space
				{
					stop(); //stop
					//flag_instruction--;
				}
			}
			
			else if(data == 0x61)  //ASCII value of a
			{
				left_degrees(90-left_turn_stopping_angle);
				_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x64)  //ASCII value of d
			{
				right_degrees(90-right_turn_stopping_angle);
				_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x41)  //ASCII value of A
			{
				left_degrees(180-left_180_turn_stopping_angle);
				_delay_ms(50);
				UDR0 = done;
			}

			else if(data == 0x44)  //ASCII value of D
			{
				right_degrees(180-right_180_turn_stopping_angle);
				_delay_ms(50);
				UDR0 = done;
			}

			else if((data & 0xF0) == 0x30 && (data & 0x0F)<=0x09 && (data & 0x0F)>0x00)  //ASCII value of 1-9
			{
				unsigned int dist = 203*(unsigned int)(data&0x0F);
				forward_mm(dist-stopping_dist);
				_delay_ms(50);
				UDR0 = done_linear;
			}

			else if((data & 0xF0) == 0x20 && (data & 0x0F)<=0x09 && (data & 0x0F)>0x00)  //0F21-0F29
			{
				unsigned int dist = 203*(unsigned int)(data&0x0F);
				back_mm(dist-stopping_dist);
				_delay_ms(50);
				UDR0 = done_linear;
			}

			else if(data == 0x6D) //ASCII value of m
			{
				pickup_and_keep();
				UDR0 = done;
			}

			else if(data == 0x6E) //ASCII value of n 
			{
				keep_from_storage();
				UDR0 = done;
			}
			
			else if(data == 0x75) //ASCII value of u
			{
				open_jaws();
				//_delay_ms(100);
				UDR0 = done;
			}
			
			else if(data == 0x55) //ASCII value of U
			{
				back_open_jaws();
				//_delay_ms(100);
				UDR0 = done;
			}
			
			else if(data == 0x69) //ASCII value of i
			{
				close_jaws();
				//_delay_ms(100);
				UDR0 = done;
			}
			
			else if(data == 0x49) //ASCII value of I
			{
				back_close_jaws();
				//_delay_ms(100);
				UDR0 = done;
			}

			else if(data == 0x62) //ASCII value of b
			{
				//grab from storage and keep on the ground
				storage_to_ground();

				UDR0 = done;
			}

			else if(data == 0x76) //ASCII value of v
			{
				//pickup from ground and keep in storage
				ground_to_storage();

				UDR0 = done;
			}

			else if(data == 0x7A) //ASCII value of z
			{
				//pickup block from ground and hold
				pickup();

				UDR0 = done;
			}

			else if(data == 0x78) //ASCII value of x
			{
				//keep block on ground
				keep();

				UDR0 = done;
			}
			else if(data == 0x3D) //ASCII value of =
			{
				//Increment the angle of horizontal servo by one
				horizontal_servo_inc();
			}

			else if(data == 0x2B) //ASCII value of +
			{
				//Increment the angle of vertical servo by one
				vertical_servo_inc();
				vertical_servo_free();
			}

			else if(data == 0x5F) //ASCII value of _
			{
				//Decrement the angle of vertical servo by one
				vertical_servo_dec();
				vertical_servo_free();
			}

			else if(data == 0x2D) //ASCII value of -
			{
				//Increment the angle of horizontal servo by 10
				horizontal_servo_dec();
			}

			else if(data == 0x5D)  //ASCII value of ]
			{
				horizontal_servo_inc10();
			}

			else if(data == 0x5B)  //ASCII value of [
			{
				horizontal_servo_dec10();
			}

			else if(data == 0x7D)  //ASCII value of }
			{
				vertical_servo_inc10();
				vertical_servo_free();
			}

			else if(data == 0x7B)  //ASCII value of {
			{
				vertical_servo_dec10();
				vertical_servo_free();
			}

			else if(data == 0x2C) //ASCII value of ,
			{
				buzzer_on();
			}

			else if(data == 0x2E) //ASCII value of .
			{
				buzzer_off();
			}

			else if(data == 0x56) //ASCII value of V
			{
			  //Storage to deep storage
			  storage_to_deep_storage();
			  UDR0 = done;
			}

			else if(data == 0x42) //ASCII value of B
			{
			  //Deep storage to storage
			  deep_storage_to_storage();
			  UDR0 = done;
			}
			
			else if(data == 0x4D) //ASCII value of M
			{
				ground_to_back_storage();
				UDR0 = done;
			}
			
			else if(data == 0x4E) //ASCII value of N
			{
				back_storage_to_ground();
				UDR0 = done;
			}

			else if(data == 0x63) //ASCII value of c
			{
				ground_to_deep_storage();
				UDR0 = done;
			}
			
			else if(data == 0x43) //ASCII value of C
			{
				deep_storage_to_ground();
				UDR0 = done;
			}
			
			else if(data == 0x58)	  //ASCII value of Y
			{
				storage_to_back_storage();
				UDR0 = done;
			}
			
			else if(data == 0x5A)	  //ASCII value of Z
			{
				back_storage_to_storage();
				UDR0 = done;
			}
			
// 			else if(data == 0x60)	  //ASCII value of `
// 			{
// 				lcd_clear();
// 			
// 				unsigned char data_received = spi_master_tx_and_rx(6);
// 				
// 				lcd_home();
// 				unsigned char var = 162;
// 				
// 				if (data_received >var)
// 				lcd_string("NO");
// 				
// 				else if (data_received <=var)
// 				lcd_string("YES");
// 				
// 				lcd_print(2, 1, data_received, 3);
// 				
// 				_delay_ms(1000);
// 				lcd_clear();
// 			}	
			
			else if (data == 0x52) //ASCII value of R
			{			
				load_speeds_from_eeprom();
				load_correct_turn_speeds_from_eeprom();
				load_correct_linear_speeds_from_eeprom();
				//print_all_speeds();
			}
				
			else if (data == 0x57) //ASCII value of W
			{		
				//lcd_clear();
				//print_all_speeds();			
				//_delay_ms(1000);	
				save_speeds_to_eeprom();
				save_correct_linear_speeds_to_eeprom();
				save_correct_turn_speeds_to_eeprom();
			}	
		
			flag_instruction--;
			
			if(start == 1)
			{
				close_jaws();
				start = 0;
			}
		}
	}
}