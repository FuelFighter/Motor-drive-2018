/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: Tanguy Simon for DNV GL Fuel fighter
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/adc.h"
#include "motor_controller_selection.h"
#include "pid.h"
#include "controller.h"

#define RPMTO8BIT 0.051

#define TC 94			//Torque constant
#define SG (0.666)		//Speed/torque gradient
#define SC 102			//Speed constant
#define IMAX 20
#define VCC 50
#define V2PWM 0xFF/VCC
//250W motor
/*
#define V_BATT 20.0
#define R 0.365
#define L 0.000423
*/

//200W motor
#define V_BATT 50.0
#define R 0.608
#define L 0.000423

const float Kp=L*2300.0 ; //1500*L
const float Ki=R*100.0 ; //100*R
const float TimeStep = 0.01 ; //10ms (see timer 0 in main.c)


static float f32_Integrator = 0.0 ;
static float f32_DutyCycleCmd = 50.0 ;
float f32_CurrentDelta = 0.0 ;

static bool b_saturation = false;

void reset_I(void)
{
	f32_Integrator = 0;
}

void controller(float f32_current_cmd, float f32_prev_current, uint8_t * u8_duty){

	if (f32_DutyCycleCmd >= 95 || f32_DutyCycleCmd <= 50)
	{
		b_saturation = true ;
		} else {
		b_saturation = false;
	}
	
	f32_CurrentDelta = (f32_current_cmd-f32_prev_current)	;
	
	if (!b_saturation) // prevents over integration of an error that cannot be dealt with (because the duty cycle reaches a limit) integral windup protection
	{
		f32_Integrator+=f32_CurrentDelta*TimeStep ;
	}
	
	f32_DutyCycleCmd=Kp*f32_CurrentDelta+f32_Integrator*Ki ;
	f32_DutyCycleCmd=f32_DutyCycleCmd+50.0 ;
	
	//bounding of duty cycle for well function of bootstrap capacitors
	if (f32_DutyCycleCmd > 95)
	{
		f32_DutyCycleCmd = 95;
	}
	
	if (f32_DutyCycleCmd < 50)
	{
		f32_DutyCycleCmd = 50;
	}
	
	OCR3A = (int)((f32_DutyCycleCmd/100.0)*ICR3) ; //PWM_PE3 (non inverted)
	OCR3B = OCR3A ; //PWM_PE4 (inverted)
	
	*u8_duty = (uint16_t)f32_DutyCycleCmd ; //exporting the duty cycle to be able to read in on the CAN and USB
}

void drivers_init() // defining pin PB4 as logical output
{
	DDRB |= (1 << PB4) ;
}

void drivers(uint8_t b_state) //when pin PB4 is high : drivers are shut down, when pin is low, drivers are ON (inverted logic) IR2104SPbF drivers
{
	if (b_state)
	{
		PORTB |= (1 << PB4) ;
	}else{
		PORTB &= ~(1 << PB4) ;
	}
}

void manage_motor(ModuleValues_t * vals)
{
	switch(vals->motor_status)
	{
		case BRAKE :
			drivers(1); //drivers turn on
			vals->b_driver_status = 1;
			controller(-vals->u8_throttle_cmd, vals->f32_motor_current,&vals->u8_duty_cycle); //negative throttle cmd
		break;
		
		case ACCEL :
			drivers(1); //drivers turn on
			vals->b_driver_status = 1;
			controller(vals->u8_throttle_cmd, vals->f32_motor_current, &vals->u8_duty_cycle);
		break;
		
		case IDLE :
			drivers(1); //drivers turn on
			vals->b_driver_status = 1;
			controller(0, vals->f32_motor_current, &vals->u8_duty_cycle); //current law running with 0 torque 
			//(integrator naturally following the speed of the car as it decreases, to prevent a big step at the next acceleration.)
		break;
		
		case OFF : // drivers disabled
			drivers(0);//drivers shutdown
			vals->b_driver_status = 0;
			reset_I(); //reset integrator
		break;
		
		case ERR :
			drivers(0);//drivers shutdown
			vals->b_driver_status = 0;
			reset_I(); //reset integrator
		break;
	}
}