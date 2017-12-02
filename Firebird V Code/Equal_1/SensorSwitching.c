/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: SensorSwitching.c
* Theme: Launch a module
* Functions:
		MOSFET_switch_config(void),
		turn_off_all_proxy_sensors (void)
* Global Variables: None
*/

#include "main.h"

/*
* Function Name:MOSFET_switch_config
* Input: None
* Output: Set the pins as output of the switching mosfets
* Example Call: MOSFET_switch_config()
*/
void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

/*
* Function Name:turn_off_all_proxy_sensors
* Input: None
* Output: Turn off Sharp,IR proximity sensor(2, 3, 4), red LED of the white line sensors
* Logic: Write 1 to all the Mosfets of all the sensors to turn them off
		 This is done to conserve battery power
* Example Call: turn_off_all_proxy_sensors()
*/
void turn_off_all_proxy_sensors (void)
{
	PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
	PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
}