/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: eeprom.c
* Theme: Launch a module
* Functions: 
	 read_forward_speed_left();
	 read_forward_speed_right();
	 read_reverse_speed_left();
	 read_reverse_speed_right();
	 load_speeds_from_eeprom();
	 save_speeds_to_eeprom();
	 load_correct_linear_speeds_from_eeprom();
	 save_correct_linear_speeds_to_eeprom();
	 load_correct_turn_speeds_from_eeprom();
	 save_correct_turn_speeds_to_eeprom();
* Global Variables: forward_speed_left_address, forward_speed_right_address, reverse_speed_left_address, reverse_speed_right_address
*/

#include <avr/eeprom.h>

//Addresses of the eeprom memory
#define forward_speed_left_address 0x00;
#define forward_speed_right_address 0x02;
#define reverse_speed_left_address 0x04;
#define reverse_speed_right_address 0x06;

extern volatile unsigned int forward_speed_left;  //Speed of the left motor
extern volatile unsigned int forward_speed_right; //Speed of the right motor
extern volatile unsigned int reverse_speed_left;  //Speed of the left motor
extern volatile unsigned int reverse_speed_right; //Speed of the right motor
extern unsigned int CorrectLinearSpeedLeft;		  //Speed of the Left motor while performing linear corrections
extern unsigned int CorrectLinearSpeedRight;	  //Speed of the Right motor while performing linear corrections
extern unsigned int CorrectTurnSpeedLeft;		  //Speed of the Left motor while performing angle corrections
extern unsigned int CorrectTurnSpeedRight;		  //Speed of the Right motor while performing angle corrections

/*
* Function Name: save_forward_speeds
* Input: forward_speed_left, forward_speed_right -> 16-bit unsigned integers which are the forward speeds of the motors
* Output: Speeds are saved at locations 0x0000, 0x0002 of the eeprom respectively
* Logic: eeprom_update_word checks if the values have changed, if they have they are updated. This is to minimize the writes to the eeprom and hence increade the life of the eeprom memory
* Example Call: save_forward_speeds(110,90)
*/
static void save_forward_speeds(uint16_t forward_speed_left, uint16_t forward_speed_right)
{
	eeprom_update_word(( uint16_t *)0x0000, forward_speed_left);
	eeprom_update_word(( uint16_t *)0x0002, forward_speed_right);
}

/*
* Function Name: save_reverse_speeds
* Input: reverse_speed_left, reverse_speed_right -> 16-bit unsigned integers which are the reverse speeds of the motors
* Output: Speeds are saved at locations 0x0004, 0x0006 of the eeprom respectively
* Logic: eeprom_update_word checks if the values have changed, if they have they are updated. This is to minimize the writes to the eeprom and hence increade the life of the eeprom memory
* Example Call: save_reverse_speeds(90,110)
*/
static void save_reverse_speeds(uint16_t reverse_speed_left, uint16_t reverse_speed_right)
{
	eeprom_update_word(( uint16_t *)0x0004, reverse_speed_left);
	eeprom_update_word(( uint16_t *)0x0006, reverse_speed_right);	
}

/*
* Function Name: read_forward_speed_left
* Input: None
* Output: Reads and returns speed saved at address 0x0000 of the eeprom
* Logic: eeprom_read_word function in eeprom.h library
* Example Call: read_forward_speed_left()
*/
static unsigned int read_forward_speed_left(void)
{
	uint16_t WordOfData ;
	WordOfData = eeprom_read_word (( uint16_t *)0x0000);
	return (unsigned int)WordOfData;
}

/*
* Function Name: read_forward_speed_right
* Input: None
* Output: Reads and returns speed saved at address 0x0002 of the eeprom
* Logic: eeprom_read_word function in eeprom.h library
* Example Call: read_forward_speed_right()
*/
static unsigned int read_forward_speed_right(void)
{
	uint16_t WordOfData ;
	WordOfData = eeprom_read_word (( uint16_t *)0x0002);
	return (unsigned int)WordOfData;
}

/*
* Function Name: read_reverse_speed_left
* Input: None
* Output: Reads and returns speed saved at address 0x0004 of the eeprom
* Logic: eeprom_read_word function in eeprom.h library
* Example Call: read_reverse_speed_left()
*/
static unsigned int read_reverse_speed_left(void)
{
	uint16_t WordOfData ;
	WordOfData = eeprom_read_word (( uint16_t *)0x0004);
	return (unsigned int)WordOfData;
}

/*
* Function Name: read_reverse_speed_right
* Input: None
* Output: Reads and returns speed saved at address 0x0006 of the eeprom
* Logic: eeprom_read_word function in eeprom.h library
* Example Call: read_reverse_speed_right()
*/
static unsigned int read_reverse_speed_right(void)
{
	uint16_t WordOfData ;
	WordOfData = eeprom_read_word (( uint16_t *)0x0006);
	return (unsigned int)WordOfData;
}

/*
* Function Name: load_speeds_from_eeprom
* Input: None
* Output: Reads and updates forward_speed_left, forward_speed_right, reverse_speed_left, reverse_speed_right
* Logic: calls functions which will read the values
* Example Call: load_speeds_from_eeprom()
*/
void load_speeds_from_eeprom(void)
{
	forward_speed_left = read_forward_speed_left();
	forward_speed_right =  read_forward_speed_right();
	reverse_speed_left = read_reverse_speed_left();
	reverse_speed_right = read_reverse_speed_right();
}

/*
* Function Name: save_speeds_to_eeprom
* Input: None
* Output: forward and reverse speeds are saved in eeprom
* Logic: calls functions which will save the values
* Example Call: save_speeds_to_eeprom()
*/
void save_speeds_to_eeprom(void)
{
	save_forward_speeds(forward_speed_left,forward_speed_right);
	save_reverse_speeds(reverse_speed_left,reverse_speed_right);
	//buzzer_beep(3);
}

/*
* Function Name: save_correct_linear_speeds_to_eeprom
* Input: None
* Output: CorrectLinearSpeedLeft and CorrectLinearSpeedRight are saved at address 0x000A and 0x000C of the eeprom
* Logic: eeprom_update_word will update the values
* Example Call: save_correct_linear_speeds_to_eeprom()
*/
void save_correct_linear_speeds_to_eeprom()
{
	eeprom_update_word(( uint16_t *)0x000A, CorrectLinearSpeedLeft);
	eeprom_update_word(( uint16_t *)0x000C, CorrectLinearSpeedRight);
	//buzzer_beep(1);
}

/*
* Function Name: load_correct_linear_speeds_from_eeprom
* Input: None
* Output: CorrectLinearSpeedLeft and CorrectLinearSpeedRight are read from address 0x000A and 0x000C of the eeprom
* Logic: eeprom_read_word of eeprom.h library
* Example Call: load_correct_linear_speeds_from_eeprom()
*/
void load_correct_linear_speeds_from_eeprom()
{
	uint16_t WordOfData ;
	
	WordOfData = eeprom_read_word (( uint16_t *)0x000A);
	CorrectLinearSpeedLeft = (unsigned int)WordOfData;
	
	WordOfData = eeprom_read_word (( uint16_t *)0x000C);
	CorrectLinearSpeedRight = (unsigned int)WordOfData;
}

/*
* Function Name: save_correct_turn_speeds_to_eeprom
* Input: None
* Output: CorrectTurnSpeedLeft and CorrectTurnSpeedRight are saved at address 0x002A and 0x002C of the eeprom
* Logic: eeprom_update_word will update the values
* Example Call: save_correct_turn_speeds_to_eeprom()
*/
void save_correct_turn_speeds_to_eeprom()
{
	eeprom_update_word(( uint16_t *)0x002A, CorrectTurnSpeedLeft);
	eeprom_update_word(( uint16_t *)0x002C, CorrectTurnSpeedRight);
	//buzzer_beep(1);
}

/*
* Function Name: load_correct_turn_speeds_from_eeprom
* Input: None
* Output: CorrectTurnSpeedLeft and CorrectTurnSpeedRight are read from address 0x002A and 0x002C of the eeprom
* Logic: eeprom_read_word of eeprom.h library
* Example Call: load_correct_turn_speeds_from_eeprom()
*/
void load_correct_turn_speeds_from_eeprom()
{
	uint16_t WordOfData ;
	
	WordOfData = eeprom_read_word (( uint16_t *)0x002A);
	CorrectTurnSpeedLeft = (unsigned int)WordOfData;
	
	WordOfData = eeprom_read_word (( uint16_t *)0x002C);
	CorrectTurnSpeedRight = (unsigned int)WordOfData;
}