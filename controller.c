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
	
	if (!b_saturation) // prevents over integration of an error that cannot be dealt with (because the duty cycle reaches a limit) intgral windup protection
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
	
	*u8_duty = (uint16_t)f32_DutyCycleCmd ;
}

void drivers_init()
{
	DDRB |= (1 << PB4) ;
}

void drivers(uint8_t b_state)
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
		if (vals->f32_batt_volt > 15.0) //if motor controller card powered
	{
		if (vals->motor_status == BRAKE)
		{
			vals->u16_watchdog = WATCHDOG_RELOAD_VALUE ;
			drivers(1); //drivers turn on
			controller(-vals->u8_throttle_cmd, vals->f32_motor_current,&vals->u8_duty_cycle);
		}
	
		if (vals->motor_status == ACCEL)
		{
			vals->u16_watchdog = WATCHDOG_RELOAD_VALUE ;
			drivers(1); //drivers turn on
			controller(vals->u8_throttle_cmd, vals->f32_motor_current, &vals->u8_duty_cycle);
		}
		if (vals->motor_status == IDLE)
		{
			/*if (vals->u16_watchdog == 0)
			{
				drivers(0);//drivers shutdown
				reset_I(); //reset integrator
			}else{
				vals->u16_watchdog -- ;
			}*/
			controller(0.0, vals->f32_motor_current,&vals->u8_duty_cycle);		
		}
	}else{
		drivers(0);//drivers shutdown
		reset_I(); //reset integrator
	}
}