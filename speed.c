/*
 * speed.c
 *
 * Created: 11/01/2018 17:34:26
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.1
 */ 

#include "speed.h"
#include "UniversalModuleDrivers/usbdb.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define N_MAG 2.0
#define D_WHEEL 0.556 // in m
#define PI 3.14
#define DISTANCE D_WHEEL*PI/N_MAG
#define LOWPASS_CONSTANT_S 0.1
#define GEAR_RATIO_1 12.5 //300/24
#define GEAR_RATIO_2 1
#define VOLT_SPEED_CST 102.0 //rmp/V
#define DUTY_CALC1 (0.85*6.0*GEAR_RATIO_1/(PI*D_WHEEL*VOLT_SPEED_CST*2))
#define DUTY_CALC2 (6.0*GEAR_RATIO_2/(PI*D_WHEEL*VOLT_SPEED_CST*2))

const float f32_speed_ratio = (17458.0/N_MAG);

static uint16_t u16_speed_array [4];

void speed_init()
{
	//pin
	DDRD &= ~(1<<PD0); //define pin as input
	PORTD &= ~(1<<PD0); //no pull-up 
	//int
	EIMSK &= ~(1<<INT0) ; // interrupt disable to prevent interrupt raise during init
	EICRA |= (1<<ISC00)|(1<<ISC01); // interrupt on rising edge
	EIFR |= (1<<INTF0) ; // clear flag
	EIMSK |= (1<<INT0) ; // interrupt enable
	
	for (int n=0;n<4;n++)
	{
		u16_speed_array[n] = 0;
	}
}

void handle_speed_sensor(volatile uint16_t *u16_speed, volatile uint16_t *u16_counter) // period in ms
{
	//uint8_t u8_new_speed = (uint8_t)(DISTANCE/(*u16_counter); // speed calculated in mm/ms
	//*u8_speed = (*u8_speed)*(1-LOWPASS_CONSTANT_S) + LOWPASS_CONSTANT_S*u8_new_speed ;// low pass filter
	//static uint8_t u8_array_pointer_old = 0;
	//static uint8_t u8_array_pointer_new = 1;
	
	if (*u16_counter > 5)
	{
		/*u16_speed_array[u8_array_pointer_new] = (uint16_t)(f32_speed_ratio/((float)*u16_counter)); // speed calculated in mm/ms 
		*u16_speed = 0;
		for (int n=0;n<4;n++)
		{
			*u16_speed += u16_speed_array[n];
		}
		*u16_speed = (uint16_t)(*u16_speed/4) ;

		u8_array_pointer_old ++ ;
		u8_array_pointer_new ++ ;
		if (u8_array_pointer_new == 5)
		{
			u8_array_pointer_new = 0;
		}
		if (u8_array_pointer_old == 5)
		{
			u8_array_pointer_old = 0;
		}*/
		*u16_speed = (uint16_t)(f32_speed_ratio/((float)*u16_counter));
		//printf("\r%u %u\n", *u16_speed, *u16_counter);
		*u16_counter = 0 ;
	}	
}

uint8_t compute_synch_duty(volatile uint8_t speed_10ms, ClutchState_t gear, float vbatt) // computing the duty cycle to reach synchronous speed before engaging the gears
{
	uint8_t Duty = 50 ;
	if (gear == GEAR1)
	{
		Duty = (speed_10ms*DUTY_CALC1/vbatt)*100 + 50 ;// Vm/2Vbatt +0.5		
	}
	if (gear == GEAR2)
	{
		Duty = (speed_10ms*DUTY_CALC2/vbatt)*100 + 50 ;// Vm/2Vbatt +0.5	
	}
	if (Duty == 50)
	{
		Duty = 51 ;
	}
	return Duty ;
}
