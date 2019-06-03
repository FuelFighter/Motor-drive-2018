/*
 * speed.c
 *
 * Created: 11/01/2018 17:34:26
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.1
 */ 

#include "speed.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "motor_controller_selection.h"
#include "controller.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define D_WHEEL 0.556 // in m
#define PI 3.14
#define DISTANCE D_WHEEL*PI/NUM_MAGNETS
#define LOWPASS_CONSTANT_S 0.1
#define GEAR_RATIO_1 18.75 //375/24 = 15.6, 375/18 = 20.8
#define GEAR_RATIO_2 18.75 //200/16 = 12.5  (BELT mode)
#define DUTY_CALC1 (1.08*6.0*GEAR_RATIO_1/(PI*D_WHEEL*VOLT_SPEED_CST*2))
#define DUTY_CALC2 (0.9*6.0*GEAR_RATIO_2/(PI*D_WHEEL*VOLT_SPEED_CST*2))

const float f32_speed_ratio = (17467.0/NUM_MAGNETS);

static uint16_t u16_speed_array [4];

void speed_init()
{
	//pin
	DDRE &= ~(1<<PE5); //define pin as input
	PORTE &= ~(1<<PE5); //no pull-up
	//int
	EIMSK &= ~(1<<INT5) ; // interrupt disable to prevent interrupt raise during init
	
	#ifdef SPEED_SENSOR_REED
	EICRB |= (1<<ISC50)|(1<<ISC51); // interrupt on rising edge
	#endif
	
	#ifdef SPEED_SENSOR_HALL
	EICRB |= (1<<ISC50); // interrupt on rising and falling edge
	#endif
		
	EIFR |= (1<<INTF5) ; // clear flag
	EIMSK |= (1<<INT5) ; // interrupt enable
	
	for (int n=0;n<4;n++)
	{
		u16_speed_array[n] = 0;
	}
}

void handle_speed_sensor(volatile uint16_t *u16_speed, volatile uint16_t *u16_counter) // period in 1ms
{
	
	if (*u16_counter > 70)
	{
		*u16_speed = (uint16_t)(f32_speed_ratio/((float)*u16_counter));
		*u16_counter = 0 ;
	}	
}

uint8_t compute_synch_duty(volatile uint8_t speed_10ms, ClutchState_t gear, float vbatt) // computing the duty cycle to reach synchronous speed before engaging the gears
{
	uint8_t Duty = 50 ;
	if (gear == GEAR1)//gear powertrain
	{
		Duty = (speed_10ms*DUTY_CALC1/vbatt)*100 + 50 ;// Vm/2Vbatt +0.5
		if (Duty == 50)
		{
			Duty = 52 ;
		}		
	}
	if (gear == BELTGEAR)//for belt powertrain
	{
		Duty = (speed_10ms*DUTY_CALC2/vbatt)*100 + 50 ;// Vm/2Vbatt +0.5	
	}
	return Duty ;
}
