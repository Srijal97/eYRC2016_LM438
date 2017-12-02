/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: servo.c
* Theme: Launch a module
* Functions: 
		 servo_init(void),
		 vertical_servo(unsigned char degrees),
		 horizontal_servo(unsigned char degrees),
		 extra_servo(unsigned char degrees),
		 vertical_servo_free(void),
		 horizontal_servo_free(void), 
		 extra_servo_free(void),
		 close_jaws(void),
		 open_jaws(void),
		 free_all_servos(void),
		 vertical_servo_inc(void),
		 horizontal_servo_inc(void),
		 vertical_servo_dec(void),
		 horizontal_servo_dec(void),
		 vertical_servo_inc10(void),
		 horizontal_servo_inc10(void),
		 vertical_servo_dec10(void),
		 horizontal_servo_dec10(void)
		 storage_to_ground( );
		 ground_to_storage( );
		 pickup( );
		 keep( );
		 storage_to_deep_storage( );
		 deep_storage_to_storage( );
		 ground_to_deep_storage( );
		 deep_storage_to_ground( );
		 storage_to_back_storage( );
		 back_storage_to_storage( );
		 ground_to_back_storage( );
		 back_storage_to_ground( );
		 pickup_and_keep( );
		 keep_from_storage( );
		
* Global Variables: 
		  horizontal_arm_angle,
		  required_horizontal_arm_angle,
		  vertical_arm_angle,
		  required_vertical_arm_angle,
		  vertical_resting_angle,
		  vertical_upper_limit,
		  vertical_lower_limit, 
		  horizontal_upper_limit,
		  horizontal_lower_limit
		  back_horizontal_arm_angle;
		  back_required_horizontal_arm_angle;
		  back_vertical_arm_angle;
		  back_required_vertical_arm_angle;
		  back_vertical_resting_angle;
		  back_vertical_upper_limit;		
		  back_vertical_lower_limit;		
		  back_horizontal_upper_limit;
		  back_horizontal_lower_limit;	
*/

#include "main.h"

int block_number = 0;

//Variables which store the angles of the gripper
int horizontal_arm_angle = 0;
int required_horizontal_arm_angle;
int vertical_arm_angle = 0;
int required_vertical_arm_angle;

int back_horizontal_arm_angle = 0;
int back_required_horizontal_arm_angle;
int back_vertical_arm_angle = 0;
int back_required_vertical_arm_angle;

//Variables which store the extreme angles of the grippers
unsigned char vertical_resting_angle = 100;
unsigned char vertical_upper_limit = 205;		//UP
unsigned char vertical_lower_limit = 0;		//DOWN
unsigned char horizontal_upper_limit = 105;		//OPEN
unsigned char horizontal_lower_limit = 52;		//CLOSE

unsigned char back_vertical_resting_angle = 120;
unsigned char back_vertical_upper_limit = 190;		//DOWN
unsigned char back_vertical_lower_limit = 0;		//UP
unsigned char back_horizontal_upper_limit = 100;	//CLOSE
unsigned char back_horizontal_lower_limit = 65;	//OPEN

/*
* Function Name: horizontal_servo_pin_config
* Input: None
* Output: Configure PORTB 5 pin for the operation of the servo of the gripper arms
* Example Call: horizontal_servo_pin_config()
*/
static void horizontal_servo_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
* Function Name: vertical_servo_pin_config
* Input: None
* Output: Configure PORTB 6 pin for operation of the vertical movement servo of the gripper arms
* Example Call: vertical_servo_pin_config ()
*/
static void vertical_servo_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*
* Function Name: back_horizontal_servo_pin_config
* Input: None
* Output: Configure PORTH 4 pin for the operation of the servo of the gripper arms
* Example Call: back_horizontal_servo_pin_config()
*/
static void back_horizontal_servo_pin_config (void)
{
	DDRH  = DDRH | 0x10;  //making PORTH 4 pin output
	PORTH = PORTH | 0x10; //setting PORTH 4 pin to logic 1
}

/*
* Function Name: back_vertical_servo_pin_config
* Input: None
* Output: Configure PORTH 5 pin for operation of the vertical movement servo of the gripper arms
* Example Call: back_vertical_servo_pin_config()
*/
static void back_vertical_servo_pin_config(void)
{
	DDRH  = DDRH | 0x20;  //making PORTH 5 pin output
	PORTH = PORTH | 0x20; //setting PORTH 5 pin to logic 1
}

/*
* Function Name: servo3_pin_config
* Input: None
* Output: None
* Logic: Configure PORTB 7 pin for operation of the servo 3
* Example Call: servo3_pin_config ()
*/
/*
//Configure PORTB 7 pin for servo motor 3 operation
static void servo3_pin_config (void)
{
	DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}
*/

/*
* Function Name: servo_init
* Input: None
* Output: call the functions which will configure the pins for the servo motors
* Example Call: servo_init()
*/
void servo_init(void)
{
	horizontal_servo_pin_config(); //Configure PORTB 5 pin for servo motor operation
	vertical_servo_pin_config(); //Configure PORTB 6 pin for servo motor operation
	//servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
	back_horizontal_servo_pin_config(); //Configure PORTH 4 pin for servo motor operation
	back_vertical_servo_pin_config(); //Configure PORTH 5 pin for servo motor operation
}

/*
* Function Name: vertical_servo
* Input: degrees->unsigned char which will store the angle to be set by the servo.
		 degrees has to be between the vertical_upper_limit & vertical_lower_limit
		 for the safety of the robotic gripper.
* Output: Function will rotate vertical servo by a specified angle
		  in the multiples of 1.86 degrees.
* Logic: Vary the OCR (output compare register) to change the duty cycle (ontime) and hence the angle
* Example Call: vertical_servo()
*/
void vertical_servo(unsigned char degrees)
{
	if(degrees <= vertical_upper_limit && degrees>=vertical_lower_limit)
	{
		float PositionPanServo = 0;
		PositionPanServo = ((float)degrees / 1.86) + 35.0;
		OCR1BH = 0x00;
		OCR1BL = (unsigned char) PositionPanServo;
	}
}

/*
* Function Name: horizontal_servo
* Input: degrees->unsigned char which will store the angle to be set by the servo.
		 degrees has to be between the horizontal_upper_limit & horizontal_lower_limit
		 for the safety of the robotic gripper.
* Output: Function will rotate horizontal servo by a specified angle
		  in the multiples of 1.86 degrees. 
* Logic: Vary the OCR (output compare register) to change the duty cycle (ontime) and hence the angle	 
* Example Call: horizontal_servo()
*/
void horizontal_servo(unsigned char degrees)
{
	if(degrees <= horizontal_upper_limit && degrees>=horizontal_lower_limit)
	{
		float PositionTiltServo = 0;
		PositionTiltServo = ((float)degrees / 1.86) + 35.0;
		OCR1AH = 0x00;
		OCR1AL = (unsigned char) PositionTiltServo;		
	}
}

/*
* Function Name: back_vertical_servo
* Input: degrees->unsigned char which will store the angle to be set by the servo.
		 degrees has to be between the back_vertical_upper_limit & back_vertical_lower_limit
		 for the safety of the robotic gripper.
* Output: Function will rotate vertical servo by a specified angle
		  in the multiples of 1.86 degrees.
* Logic: Vary the OCR (output compare register) to change the duty cycle (ontime) and hence the angle
* Example Call: back_vertical_servo()
*/
void back_vertical_servo(unsigned char degrees)
{
	if(degrees <= back_vertical_upper_limit && degrees>=back_vertical_lower_limit)
	{
		float PositionPanServo = 0;
		PositionPanServo = ((float)degrees / 1.86) + 35.0;
		OCR4CH = 0x00;
		OCR4CL = (unsigned char) PositionPanServo;
	}
}

/*
* Function Name: back_horizontal_servo
* Input: degrees->unsigned char which will store the angle to be set by the servo.
		 degrees has to be between the back_horizontal_upper_limit & back_horizontal_lower_limit
		 for the safety of the robotic gripper.
* Output: Function will rotate horizontal servo by a specified angle
		  in the multiples of 1.86 degrees. 
* Logic: Vary the OCR (output compare register) to change the duty cycle (ontime) and hence the angle	 
* Example Call: back_horizontal_servo()
*/
void back_horizontal_servo(unsigned char degrees)
{
	if(degrees <= back_horizontal_upper_limit && degrees>=back_horizontal_lower_limit)
	{
		float PositionTiltServo = 0;
		PositionTiltServo = ((float)degrees / 1.86) + 35.0;
		OCR4BH = 0x00;
		OCR4BL = (unsigned char) PositionTiltServo;		
	}
}

/*
* Function Name: extra_servo
* Input: degrees->unsigned char which will store the angle to be set by the servo.
* Output: Function will rotate servo by a specified angle
		  in the multiples of 1.86 degrees.	 
* Logic: Vary the OCR (output compare register) to change the duty cycle (ontime) and hence the angle	  
* Example Call: extra_servo()
void extra_servo(unsigned char degrees)
{
	vertical_arm_angle = degrees;
	float PositionServo = 0;
	PositionServo = ((float)degrees / 1.86) + 35.0;
	OCR1CH = 0x00;
	OCR1CL = (unsigned char) PositionServo;
}
*/

/*
* Function Name: vertical_servo_free
* Input: None
* Output: function unlocks the servo motor from the any angle
		  and make them free
* Logic:  Give 100% duty cycle at the PWM. This function can be used to
		 reduce the power consumption of the motor if it is holding load against the gravity.
* Example Call: vertical_servo_free()
*/
void vertical_servo_free (void)
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off	
}

/*
* Function Name: horizontal_servo_free
* Input: None
* Output: Function unlocks the servo motor from the any angle
		  and make them free
* Logic: Give 100% duty cycle at the PWM. This function can be used to
		 reduce the power consumption of the motor if it is not holding an object.
* Example Call: horizontal_servo_free()
*/
void horizontal_servo_free (void)
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

/*
* Function Name: extra_servo_free
* Input: None
* Output: Function unlocks the servo motor from the any angle
		  and make them free
* Logic: Give 100% duty cycle at the PWM. This function can be used to
		 reduce the power consumption of the motor.
* Example Call: extra_servo_free()
void extra_servo_free(void) //makes servo 3 free rotating
{
	OCR1CH = 0x03;
	OCR1CL = 0xFF; //Servo 3 off
}
*/

/*
* Function Name: back_vertical_servo_free
* Input: None
* Output: function unlocks the servo motor from the any angle
		  and make them free
* Logic:  Give 100% duty cycle at the PWM. This function can be used to
		 reduce the power consumption of the motor if it is holding load against the gravity.
* Example Call: back_vertical_servo_free()
*/
void back_vertical_servo_free (void)
{
	OCR4CH = 0x03;
	OCR4CL = 0xFF; //Servo off	
}

/*
* Function Name: back_horizontal_servo_free
* Input: None
* Output: Function unlocks the servo motor from the any angle
		  and make them free
* Logic: Give 100% duty cycle at the PWM. This function can be used to
		 reduce the power consumption of the motor if it is not holding an object.
* Example Call: back_horizontal_servo_free()
*/
void back_horizontal_servo_free (void)
{
	OCR4BH = 0x03;
	OCR4BL = 0xFF; 
}


/*
* Function Name: free_all_servos
* Input: None
* Output: Function unlocks the all the servo motors from the any angle.
* Example Call: free_all_servos()
*/
void free_all_servos(void)
{
	vertical_servo_free();
	horizontal_servo_free();
	back_vertical_servo_free();
	back_horizontal_servo_free();
	//extra_servo_free();
}

/*
* Function Name: open_jaws
* Input: None
* Output: Opens the gripper arms by changing the angle of the servo motor.
* Logic: The angle is changed incrementally to provide a sweeping action instead of a fast jerk.
* Example Call: open_jaws()
*/
void open_jaws(void)
{
	required_horizontal_arm_angle = horizontal_upper_limit;
	
	while(required_horizontal_arm_angle!=horizontal_arm_angle)
	{		
		if(horizontal_arm_angle<required_horizontal_arm_angle)
		horizontal_arm_angle+=1;
		else if(required_horizontal_arm_angle<horizontal_arm_angle)
		horizontal_arm_angle-=1;
		horizontal_servo(horizontal_arm_angle);
		_delay_ms(4);
	}
	//horizontal_servo_free();
}

/*
* Function Name: close_jaws
* Input: None
* Output: Closes the gripper arms by changing the angle of the servo motor.
* Logic: The angle is changed incrementally to provide a sweeping action instead of a fast jerk.
* Example Call: close_jaws()
*/
void close_jaws(void)
{
	required_horizontal_arm_angle = horizontal_lower_limit;
	
	while(required_horizontal_arm_angle!=horizontal_arm_angle)
	{
		if(required_horizontal_arm_angle<horizontal_arm_angle)
		horizontal_arm_angle-=1;
		else if(horizontal_arm_angle<required_horizontal_arm_angle)
		horizontal_arm_angle+=1;
		horizontal_servo(horizontal_arm_angle);
		
		_delay_ms(4);
	}
	horizontal_servo_free();
}

/*
* Function Name: back_open_jaws
* Input: None
* Output: Opens the gripper arms by changing the angle of the servo motor.
* Logic: The angle is changed incrementally to provide a sweeping action instead of a fast jerk.
* Example Call: back_open_jaws()
*/
void back_open_jaws(void)
{
	back_required_horizontal_arm_angle = back_horizontal_lower_limit;
	
	while(back_required_horizontal_arm_angle!=back_horizontal_arm_angle)
	{
		if(back_horizontal_arm_angle<back_required_horizontal_arm_angle)
		back_horizontal_arm_angle+=1;
		else if(back_horizontal_arm_angle>back_required_horizontal_arm_angle)
		back_horizontal_arm_angle-=1;
		back_horizontal_servo(back_horizontal_arm_angle);
		_delay_ms(4);
	}
	//back_horizontal_servo_free();
}

/*
* Function Name: close_jaws
* Input: None
* Output: Closes the gripper arms by changing the angle of the servo motor.
* Logic: The angle is changed incrementally to provide a sweeping action instead of a fast jerk.
* Example Call: close_jaws()
*/
void back_close_jaws(void)
{
	back_required_horizontal_arm_angle = back_horizontal_upper_limit;
	
	while(back_required_horizontal_arm_angle!=back_horizontal_arm_angle)
	{		
		if(back_horizontal_arm_angle>back_required_horizontal_arm_angle)
		back_horizontal_arm_angle-=1;
		else if(back_horizontal_arm_angle<back_required_horizontal_arm_angle)
		back_horizontal_arm_angle+=1;
		back_horizontal_servo(back_horizontal_arm_angle);
		_delay_ms(4);
	}
	back_horizontal_servo_free();
}

/*
* Function Name: vertical_servo_inc
* Input: None
* Output: Increments the angle of the vertical servo motor by one.
		  Used for testing and debugging purposes.
* Example Call: vertical_servo_inc()
*/
void vertical_servo_inc(void)
{
	vertical_servo(++vertical_arm_angle);
	_delay_ms(100);
}

/*
* Function Name: horizontal_servo_inc
* Input: None
* Output: Increments the angle of the horizontal servo motor by one.
		 Used for testing and debugging purposes.
* Example Call: horizontal_servo_inc()
*/
void horizontal_servo_inc(void)
{
	horizontal_servo(++horizontal_arm_angle);	
	_delay_ms(100);
}

/*
* Function Name: vertical_servo_dec
* Input: None
* Output: Decrements the angle of the vertical servo motor by one.
		  Used for testing and debugging purposes.
* Example Call: vertical_servo_dec()
*/
void vertical_servo_dec(void)
{
	vertical_servo(--vertical_arm_angle);
	_delay_ms(100);
}

/*
* Function Name: horizontal_servo_dec
* Input: None
* Output: Decrements the angle of the horizontal servo motor by one.
		  Used for testing and debugging purposes.
* Example Call: horizontal_servo_dec()
*/
void horizontal_servo_dec(void)
{
	horizontal_servo(--horizontal_arm_angle);
	_delay_ms(100);
}

/*
* Function Name: vertical_servo_inc10
* Input: None
* Output: Increments the angle of the vertical servo motor by 10.
		  Used for testing and debugging purposes.
* Example Call: vertical_servo_inc10()
*/
void vertical_servo_inc10(void)
{
	vertical_arm_angle+=10;
	vertical_servo(vertical_arm_angle);
	_delay_ms(300);
}

/*
* Function Name: horizontal_servo_inc10
* Input: None
* Output: Increments the angle of the horizontal servo motor by 10.
		  Used for testing and debugging purposes.
* Example Call: horizontal_servo_inc10()
*/
void horizontal_servo_inc10(void)
{
	horizontal_arm_angle+=10;
	horizontal_servo(horizontal_arm_angle);	
	_delay_ms(200);
}

/*
* Function Name: vertical_servo_dec10
* Input: None
* Output: Decrements the angle of the vertical servo motor by 10.
		  Used for testing and debugging purposes.
* Example Call: vertical_servo_dec10()
*/
void vertical_servo_dec10(void)
{
	vertical_arm_angle-=10;
	vertical_servo(vertical_arm_angle);
	_delay_ms(300);
}

/*
* Function Name: horizontal_servo_dec10
* Input: None
* Output: Decrements the angle of the horizontal servo motor by 10.
		  Used for testing and debugging purposes.
* Example Call: horizontal_servo_dec10()
*/
void horizontal_servo_dec10(void)
{
	horizontal_arm_angle-=10;
	horizontal_servo(horizontal_arm_angle);
	_delay_ms(200);
}

/*
* Function Name: storage_to_ground
* Input: None
* Output: Moves the block from storage to ground
* Example Call: storage_to_ground()
*/
void storage_to_ground(void)
{
	open_jaws();
	_delay_ms(200);
	vertical_servo(vertical_upper_limit);
	_delay_ms(800);
	close_jaws();
	_delay_ms(100);
	vertical_servo(vertical_lower_limit);
	_delay_ms(1500);
	open_jaws();
	_delay_ms(100);
	vertical_servo(vertical_resting_angle);
	_delay_ms(1000);
	close_jaws();
	vertical_servo_free();
	//horizontal_servo_free();	
}

/*
* Function Name: ground_to_storage
* Input: None
* Output: Moves the block from ground to storage
* Example Call: ground_to_storage()
*/
void ground_to_storage(void)
{
	open_jaws();
	_delay_ms(200);
	vertical_servo(vertical_lower_limit);
	_delay_ms(800);
	close_jaws();
	_delay_ms(100);
	vertical_servo(vertical_upper_limit);
	_delay_ms(1500);
	open_jaws();
	_delay_ms(100);
	vertical_servo(vertical_resting_angle);
	_delay_ms(1000);
	vertical_servo_free();
	close_jaws();
	horizontal_servo_free();
}

/*
* Function Name: pickup
* Input: None
* Output: Picks up the block
* Example Call: pickup()
*/
void pickup(void)
{
	open_jaws();
	_delay_ms(200);
	vertical_servo(vertical_lower_limit);
	_delay_ms(600);
	close_jaws();
	_delay_ms(100);
	vertical_servo(vertical_resting_angle);
	_delay_ms(600);
	vertical_servo_free();
}

/*
* Function Name: keep
* Input: None
* Output: Keeps the block on the floor held in its arms
* Example Call: keep()
*/
void keep(void)
{
	vertical_servo(vertical_lower_limit);
	_delay_ms(600);
	open_jaws();
	_delay_ms(100);
	vertical_servo(vertical_resting_angle);
	_delay_ms(600);
	close_jaws();
	vertical_servo_free();
}

/*
* Function Name: storage_to_deep_storage
* Input: None
* Output: The back arms pick up the block placed in storage
* Example Call: storage_to_deep_storage()
*/
void storage_to_deep_storage(void)
{
	back_open_jaws();
	back_vertical_servo(back_vertical_lower_limit);   //On the storage platform
	_delay_ms(700);
	back_close_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_resting_angle);
	_delay_ms(700);
	back_vertical_servo_free();
}

/*
* Function Name: deep_storage_to_storage
* Input: None
* Output: The back arms keep the block to storage
* Example Call: deep_storage_to_storage()
*/
void deep_storage_to_storage(void)
{
	back_vertical_servo(back_vertical_lower_limit);   //On the storage platform
	_delay_ms(900);
	back_open_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_resting_angle);
	_delay_ms(600);
	back_close_jaws();
	back_vertical_servo_free();
}

/*
* Function Name: ground_to_deep_storage
* Input: None
* Output: The arms move the block from ground>storage>back_arms
* Example Call: ground_to_deep_storage()
*/
void ground_to_deep_storage(void)
{
	ground_to_storage();
// 	open_jaws();
// 	//_delay_ms(200);
// 	vertical_servo(vertical_lower_limit);
// 	_delay_ms(800);
// 	close_jaws();
// 	_delay_ms(100);
// 	vertical_servo(vertical_upper_limit);
// 	_delay_ms(1500);
// 	open_jaws();
// 	_delay_ms(100);
// 	vertical_servo(vertical_resting_angle);
// 	_delay_ms(1000);
// 	vertical_servo_free();
// 	horizontal_servo_free();	
	
	storage_to_deep_storage();
	//back_open_jaws();
	//back_vertical_servo(back_vertical_resting_angle); //stow
// 	back_vertical_servo(back_vertical_lower_limit);   //On the storage platform
// 	_delay_ms(600);
// 	back_close_jaws();
// 	_delay_ms(100);
// 	back_vertical_servo(back_vertical_resting_angle);
// 	_delay_ms(600);
// 	back_vertical_servo_free();
}

/*
* Function Name: deep_storage_to_ground
* Input: None
* Output: The arms move the block from back_arms>storage>ground
* Example Call: deep_storage_to_ground()
*/
void deep_storage_to_ground(void)
{
	deep_storage_to_storage();
	//deep storage to storage
	//back_close_jaws();
	//back_vertical_servo(back_vertical_resting_angle);
// 	back_vertical_servo(back_vertical_lower_limit);   //On the storage platform
// 	_delay_ms(900);
// 	back_open_jaws();
// 	_delay_ms(100);
// 	back_vertical_servo(back_vertical_resting_angle);
// 	_delay_ms(600);
// 	back_vertical_servo_free();
// 	back_horizontal_servo_free();
	
	storage_to_ground();
// 	open_jaws();
// 	//_delay_ms(200);
// 	vertical_servo(vertical_upper_limit);
// 	_delay_ms(800);
// 	close_jaws();
// 	_delay_ms(100);
// 	vertical_servo(vertical_lower_limit);
// 	_delay_ms(1500);
// 	open_jaws();
// 	_delay_ms(100);
// 	vertical_servo(vertical_resting_angle);
// 	_delay_ms(1000);
// 	vertical_servo_free();
// 	horizontal_servo_free();
}

/*
* Function Name: storage_to_back_storage
* Input: None
* Output: The arms move the block from storage to back storage
* Example Call: storage_to_back_storage()
*/
void storage_to_back_storage()
{
	back_open_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_lower_limit);
	_delay_ms(600);
	back_close_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_upper_limit);
	_delay_ms(1300);
	back_open_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_resting_angle);
	_delay_ms(600);
	back_close_jaws();
	back_vertical_servo_free();
}

/*
* Function Name: storage_to_back_storage
* Input: None
* Output: The arms move the block from back storage to storage
* Example Call: storage_to_back_storage()
*/
void back_storage_to_storage()
{
	back_open_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_upper_limit);
	_delay_ms(600);
	back_close_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_lower_limit);
	_delay_ms(1300);
	back_open_jaws();
	_delay_ms(100);
	back_vertical_servo(back_vertical_resting_angle);
	_delay_ms(600);
	back_close_jaws();
	back_vertical_servo_free();
}

/*
* Function Name: ground_to_back_storage
* Input: None
* Output: The arms move the block from ground to back storage
* Example Call: ground_to_back_storage()
*/
void ground_to_back_storage(void)
{
	ground_to_storage();
	storage_to_back_storage();
}

/*
* Function Name: back_storage_to_ground
* Input: None
* Output: The arms move the block from back storage to ground
* Example Call: back_storage_to_ground()
*/
void back_storage_to_ground(void)
{
	back_storage_to_storage();
	storage_to_ground();
}

/*
* Function Name: pickup_and_keep
* Input: None
* Output: The bot takes in a block based on block_number
* Example Call: pickup_and_keep()
*/
void pickup_and_keep()
{
	++block_number;
	
	if(block_number == 1)
	{
		ground_to_storage();
		storage_to_back_storage();
	}
	
	else if(block_number == 2)
	{
		ground_to_storage();
		storage_to_deep_storage();
	}
	
	else if(block_number == 3)
	{
		pickup();
	}
	
	else
	{
		block_number--;
	}
}

/*
* Function Name: keep_from_storage
* Input: None
* Output: The bot outputs a block based on block_number
* Example Call: keep_from_storage()
*/
void keep_from_storage()
{
	if (block_number == 1)
	{
		back_storage_to_storage();
		storage_to_ground();
		--block_number;
	}
	
	else if(block_number == 2)
	{
		deep_storage_to_storage();
		storage_to_ground();
		--block_number;
	}
	
	else if(block_number == 3)
	{
		keep();
		--block_number;
	}
}