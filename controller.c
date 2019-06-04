/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: Tanguy Simon for DNV GL Fuel fighter
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/adc.h"
#include "UniversalModuleDrivers/pwm.h"
#include "state_machine.h"
#include "pid.h"
#include "controller.h"

#define MAX_DUTY_BOUND 95
#define MIN_DUTY_BOUND 5

const float Kp=L*2300.0*0.4 ; //1500*L 2300*L
const float Ki=R*100.0*0.7 ; //100*R
const float TimeStep = 0.005 ; //5ms (see timer 0 in main.c)

static float f32_Integrator = 0.0 ;

void reset_I(void)
{
	f32_Integrator = 0;
}

void set_I(uint8_t duty)
{
	f32_Integrator = (duty-50.0)/Ki;
}

void controller(volatile ModuleValues_t *vals){
	
	static float f32_DutyCycle = 50.0 ;
	static float f32_DutyCycleCmd = 50.0;
	float f32_CurrentDelta = 0.0 ;
	static uint8_t b_saturation = 0;
	float f32_throttle_cmd = 0;
	
	if (vals->motor_status == BRAKE_GEAR1 || vals->motor_status == BRAKE_GEAR2)
	{
		f32_throttle_cmd = -vals->u8_brake_cmd ;
	}
	if (vals->motor_status == ACCEL_GEAR1 || vals->motor_status == ACCEL_GEAR2)
	{
		f32_throttle_cmd = vals->u8_accel_cmd ;
	}
	
	if (vals->ctrl_type == CURRENT)
	{
		if (f32_DutyCycle >= MAX_DUTY_BOUND || f32_DutyCycle <= MIN_DUTY_BOUND)
		{
			b_saturation = 1 ;
		} else {
			b_saturation = 0;
		}
		
		f32_CurrentDelta = ((f32_throttle_cmd)-vals->f32_motor_current)	;
		
		if (!b_saturation) // prevents over integration of an error that cannot be dealt with (because the duty cycle reaches a limit) integral windup protection
		{
			f32_Integrator+=f32_CurrentDelta*TimeStep ;
		}
		
		f32_DutyCycle=Kp*f32_CurrentDelta+f32_Integrator*Ki ;
		f32_DutyCycle=f32_DutyCycle+50.0 ;
	
	}else if (vals->ctrl_type == PWM)
	{
		f32_DutyCycle = (float)(vals->u8_duty_cycle);
		if (vals->f32_motor_current > 0.5)
		{
			//f32_DutyCycle -- ;
		}
		if (vals->f32_motor_current < -0.5)
		{
			//f32_DutyCycle ++ ;
		}
	}
	
	
	//bounding of duty cycle for well function of bootstrap capacitors
	if (f32_DutyCycle > MAX_DUTY_BOUND)
	{
		f32_DutyCycle = MAX_DUTY_BOUND;
	}
	
	if (f32_DutyCycle < MIN_DUTY_BOUND)// bounding at 50 to prevent rheostatic braking and backwards motion
	{
		f32_DutyCycle = MIN_DUTY_BOUND;
	}
	
	if (vals->gear_status == GEAR2)
	{
		f32_DutyCycleCmd = (1-f32_DutyCycle);
	}
		
	if (SW_MODE == BIPOLAR)
	{
		OCR3A = (int)((f32_DutyCycleCmd/100.0)*ICR3) ; //PWM_PE3 (non inverted)
		OCR3B = OCR3A ; //PWM_PE4 (inverted)
	}else{//UNIPOLAR
		OCR3A = (int)((f32_DutyCycleCmd/100.0)*ICR3) ; //PWM_PE3
		OCR3B = (int)(ICR3-(f32_DutyCycleCmd/100.0)*ICR3) ; //PWM_PE4
	}
	
	vals->u8_duty_cycle = (uint8_t)f32_DutyCycle ; //exporting the duty cycle to be able to read in on the CAN and USB

}

void drivers_init() // defining pin PB4 as logical output
{
	DDRB |= (1 << PB4) ;
}

void drivers(uint8_t b_state) //when pin PB4 is high : drivers are shut down, when pin is low, drivers are ON (inverted logic) IR2104SPbF drivers
{
	if (b_state == 1)
	{
		PORTB |= (1 << PB4) ;
	}else{
		PORTB &= ~(1 << PB4) ;
	}
}