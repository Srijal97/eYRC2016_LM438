/*
* Team Id: LM438
* Author: Chirag Shah
* Filename: main.h
* Theme: Launch a module
* Functions: 
* Global Variables: None
*/

#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// void spi_pin_config();
// void spi_init();
// unsigned char spi_master_tx_and_rx (unsigned char);

void load_speeds_from_eeprom(void);
void save_speeds_to_eeprom(void);
void load_correct_linear_speeds_from_eeprom();
void save_correct_linear_speeds_to_eeprom();
void load_correct_turn_speeds_from_eeprom();
void save_correct_turn_speeds_to_eeprom();

void motion_pin_config (void);
void left_encoder_pin_config (void);
void right_encoder_pin_config (void);
void buzzer_pin_config(void);
void left_position_encoder_interrupt_init (void);
void right_position_encoder_interrupt_init (void);

void forward (void); //both wheels forward
void back (void); //both wheels backward
void left (void); //Left wheel backward, Right wheel forward
void right (void); //Left wheel forward, Right wheel backward
void soft_left (void); //Left wheel stationary, Right wheel forward
void soft_right (void); //Left wheel forward, Right wheel is stationary
void soft_left_2 (void); //Left wheel backward, right wheel stationary
void soft_right_2 (void); //Left wheel stationary, Right wheel backward
void stop (void);

void forward_mm(unsigned int);
void back_mm(unsigned int);
void left_degrees(unsigned int);
void right_degrees(unsigned int);
void soft_left_degrees(unsigned int);
void soft_right_degrees(unsigned int);
void soft_left_2_degrees(unsigned int);
void soft_right_2_degrees(unsigned int);

void correct_left(unsigned int);
void correct_right(unsigned int);
void correct_left_2(unsigned int);
void correct_right_2(unsigned int);

void lcd_start(void);
void lcd_port_config (void);
void lcd_string(char*);
void lcd_clear(void);
void lcd_print(char, char , unsigned int , int);
void lcd_cursor(char,char);
void lcd_home(void);

void velocity (unsigned int, unsigned int);
void modify_speed(void);
void print_all_speeds(void);

void timer1_init();
void timer4_init();
void timer5_init();

void uart0_init(void);

void servo_init(void);
void vertical_servo(unsigned char degrees);
void horizontal_servo(unsigned char degrees);

void back_vertical_servo(unsigned char degrees);
void back_horizontal_servo(unsigned char degrees);

//void extra_servo(unsigned char degrees);
void vertical_servo_free(void); //makes servo vertical servo free rotating
void horizontal_servo_free(void); //makes servo horizontal servo free rotating
void extra_servo_free(void); //makes servo 3 free rotating
void close_jaws(void);
void open_jaws(void);

void back_vertical_servo_free(void); //makes servo free rotating
void back_horizontal_servo_free(void); //makes servo free rotating
void back_extra_servo_free(void); //makes servo free rotating
void back_close_jaws(void);
void back_open_jaws(void);

void free_all_servos(void);

void vertical_servo_inc(void);
void horizontal_servo_inc(void);
void vertical_servo_dec(void);
void horizontal_servo_dec(void);
void vertical_servo_inc10(void);
void horizontal_servo_inc10(void);
void vertical_servo_dec10(void);
void horizontal_servo_dec10(void);

void MOSFET_switch_config(void);
void turn_off_all_proxy_sensors (void);

void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(int);

void storage_to_ground(void);
void ground_to_storage(void);
void pickup(void);
void keep(void);
void storage_to_deep_storage(void);
void deep_storage_to_storage(void);
void ground_to_deep_storage(void);
void deep_storage_to_ground(void);
void storage_to_back_storage(void);
void back_storage_to_storage(void);
void ground_to_back_storage(void);
void back_storage_to_ground(void);
void pickup_and_keep(void);
void keep_from_storage(void);

void stop_encoder_feedback_correction(void);
void start_encoder_feedback_correction_later(void);
void start_encoder_feedback_correction_now(void);

void CheckIfBothSpeedsAreProper(int,int);
void CheckIfBothTurnSpeedsAreProper(int, int);

#endif /* MAIN_H_ */