/*
 * speed.c
 *
 * Created: 11/01/2018 17:34:26
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include "speed.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define N_MAG 1.0
#define D_WHEEL 500.0 // in mm, TO BE DETERMINED
#define PI 3.14
#define COUNT_TO_DISTANCE D_WHEEL*PI/N_MAG
#define LOWPASS_CONSTANT_S 0.1

void speed_init()
{
	//pin
	DDRE &= ~(1<<PE5); //define pin as input
	PORTE &= ~(1<<PE5); //no pull-up 
	//int
	EIMSK &= ~(1<<INT5) ; // interrupt disable to prevent interrupt raise during init
	EICRB |= (1<<ISC50)|(1<<ISC51); // interrupt on rising edge
	EIFR |= (1<<INTF5) ; // clear flag
	EIMSK |= (1<<INT5) ; // interrupt enable
}

void handle_speed_sensor(uint8_t *u8_speed,uint16_t *u16_counter, uint16_t u16_period) // period in ms
{
	volatile uint8_t u8_new_speed = (uint8_t)(((float)*u16_counter)*D_WHEEL*PI/N_MAG/u16_period); // speed calculated in mm/ms
	//*u8_speed = (*u8_speed)*(1-LOWPASS_CONSTANT_S) + LOWPASS_CONSTANT_S*u8_new_speed ;// low pass filter
	*u8_speed = u8_new_speed ;
	*u16_counter = 0 ;
}
