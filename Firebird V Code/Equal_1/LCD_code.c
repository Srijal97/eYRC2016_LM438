/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: LCD.c
* Theme: Launch a module
* Functions:
		lcd_start(void),
		lcd_port_config (void),
		lcd_string(char*),
		lcd_clear(void),
		lcd_print(char, char , unsigned int , int),
		lcd_cursor(char,char),
		lcd_home(void);

* Global Variables: None
*/

#include "main.h"

#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC

#define sbit(reg,bit)	reg |= (1<<bit)			// Macro defined for Setting a bit of any register.
#define cbit(reg,bit)	reg &= ~(1<<bit)		// Macro defined for Clearing a bit of any register.

static unsigned int temp;
static unsigned int unit;
static unsigned int tens;
static unsigned int hundred;
static unsigned int thousand;
static unsigned int million;

static void lcd_init();
static void lcd_wr_command(unsigned char);
static void lcd_wr_char(char);


/*
* Function Name:lcd_port_config
* Input: None
* Output: None
* Logic: Sets the data direction registers of the pins connected to the LCD port as output 
* Example Call: lcd_port_config()
*/
//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
/*
* Function Name:lcd_set_4bit
* Input: None
* Output: None
* Logic: This function configures the LCD to work in 4 bit mode instead of 8 bit mode to reduce
		 the number of data lines required to be used
* Example Call: lcd_set_4bit
*/
static void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x20;				//Sending 2 to initialise LCD 4-bit mode
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(1);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	
}

/*
* Function Name:lcd_init
* Input: None
* Output: None
* Logic: This function initializes the LCD
* Example Call: lcd_init
*/
static void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28);			//LCD 4-bit mode and 2 lines.
	lcd_wr_command(0x01);
	lcd_wr_command(0x06);
	lcd_wr_command(0x0E);
	lcd_wr_command(0x80);
	
}

/*
* Function Name:lcd_wr_command
* Input: cmd -> The hex value of the command to be given to lcd
* Output: None
* Logic: Outputs the command values to the lcd via PORTC
		 This function is used to give various commands to the LCD to perform various functions 
* Example Call: lcd_wr_command(0x80) To bring cursor at home position
*/
static void lcd_wr_command(unsigned char cmd)
{
	static unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
	
	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

/*
* Function Name:lcd_wr_char
* Input: letter -> char type data which will be printed on the lcd at the cursor's current position 
* Output: Function to write character on LCD
* Logic: Outputs the appropriate values to the lcd via PORTC
* Example Call: lcd_wr_char(0x41) -to write a to the lcd
*/
static void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

/*
* Function Name:lcd_home
* Input: None
* Output: Cursor returns to home position
* Logic: Passes 0x80 to lcd_wr_command
* Example Call: lcd_home()
*/
void lcd_home(void)
{
	lcd_wr_command(0x80);
}

/*
* Function Name:lcd_string
* Input: *str -> Pointer of the char data type. Points to the address of the the first character of the string.
* Output: Prints string on the LCD
* Logic:  Prints each character to the LCD in a while loop until EOF is reached
* Example Call: lcd_string("hello")
*/
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

/*
* Function Name:lcd_cursor
* Input: row,column -> where you want to position the lcd
* Output: Cursor moves to the desired position
* Logic: Position the LCD cursor at "row", "column", by passing the required commands to the lcd_wr_command function.
 c->  1 2 3 4 5 6 7 8  
	1 . . . . . . . . 
	2 . . . . . . . . 
	^
	row
* Example Call: lcd_string(2,3)
*/
void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		default: break;
	}
}

/*
* Function Name:lcd_string
* Input: row, column, value, digits -> value is the integer you want to print
* Output: Integer is printed to the desired location and upto the desired number of digits
* Logic: 
* Example Call: lcd_string("hello")
*/
void lcd_print(char row, char coloumn, unsigned int value, int digits)
{
	unsigned char flag=0;
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	if(digits==5 || flag==1)
	{
		million=value/10000+48;
		lcd_wr_char(million);
		flag=1;
	}
	if(digits==4 || flag==1)
	{
		temp = value/1000;
		thousand = temp%10 + 48;
		lcd_wr_char(thousand);
		flag=1;
	}
	if(digits==3 || flag==1)
	{
		temp = value/100;
		hundred = temp%10 + 48;
		lcd_wr_char(hundred);
		flag=1;
	}
	if(digits==2 || flag==1)
	{
		temp = value/10;
		tens = temp%10 + 48;
		lcd_wr_char(tens);
		flag=1;
	}
	if(digits==1 || flag==1)
	{
		unit = value%10 + 48;
		lcd_wr_char(unit);
	}
	if(digits>5)
	{
		lcd_wr_char('E');
	}
	
}

/*
* Function Name:lcd_start
* Input: None
* Output: LCD is initialized
* Logic: Calls various functions which will initialize the LCD
* Example Call: lcd_start()
*/
void lcd_start(void)
{
		lcd_set_4bit();
		lcd_init();
}

/*
* Function Name:lcd_clear
* Input: None
* Output: LCD is cleared
* Logic: Writes spaces to both the rows of the LCD
* Example Call: lcd_clear()
*/
void lcd_clear(void)
{
	lcd_cursor(1,1);
	lcd_string("                ");
	lcd_cursor(2,1);
	lcd_string("                ");
}