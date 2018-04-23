/*
 * state_machine.c
 *
 * Created: 22/04/2018 16:00:41
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : not hardware specific
 */
#include <stdlib.h>
#include <avr/io.h>
#include "state_machine.h"
#include "controller.h"

#define MAX_VOLT 55.0
#define MIN_VOLT 15.0
#define MAX_AMP 15.0
#define MAX_TEMP 100

static uint8_t b_major_fault = 0;

void state_handler(ModuleValues_t * vals)
{
	uint8_t b_board_powered = (vals->f32_batt_volt >= MIN_VOLT  && vals->f32_batt_volt < 100.0);
	
	if (b_board_powered && (vals->f32_motor_current >= MAX_AMP || vals->f32_batt_volt > MAX_VOLT))
	{
		b_major_fault = 1;
	}
	
	switch(vals->motor_status)
	{
		case OFF:
			//transition 1, CAN
			if (vals->u16_watchdog > 0 && b_board_powered)
			{
				vals->motor_status = IDLE;
			}
			//During
			drivers(0);//drivers shutdown
			vals->b_driver_status = 0;
			reset_I(); //reset integrator
			vals->i8_throttle_cmd = 0;
			vals->u8_duty_cycle = 50;
		
		break;
		
		case IDLE: 
			//transition 5
			if (vals->i8_throttle_cmd > 0)
			{
				vals->motor_status = ACCEL;
			}
			//transition 7
			if (vals->i8_throttle_cmd < 0)
			{
				vals->motor_status = BRAKE;
			}
			drivers(1);//drivers enable
			controller(vals->i8_throttle_cmd, vals->f32_motor_current, &vals->u8_duty_cycle,vals->ctrl_type); //current law running with 0 torque
			//(integrator naturally following the speed of the car as it decreases, to prevent a big step at the next acceleration.)
		break;
		
		case ACCEL:
			//transition 6
			if (vals->i8_throttle_cmd == 0)
			{
				vals->motor_status = IDLE;
			}
			controller(vals->i8_throttle_cmd, vals->f32_motor_current, &vals->u8_duty_cycle,vals->ctrl_type);
		break;
		
		case BRAKE:
			//transition 8
			if (vals->i8_throttle_cmd == 0)
			{
				vals->motor_status = IDLE;
			}
			controller(vals->i8_throttle_cmd, vals->f32_motor_current,&vals->u8_duty_cycle,vals->ctrl_type); //negative throttle cmd
		break;
		
		case ERR:
			if (!b_major_fault && vals->u8_motor_temp < MAX_TEMP)
			{
				//transition 4
				vals->motor_status = IDLE;
			}
			drivers(0);//drivers shutdown
			vals->b_driver_status = 0;
			reset_I(); //reset integrator
			vals->i8_throttle_cmd = 0;
			vals->u8_duty_cycle = 50 ;
		break;	
	}
	
	if ((vals->motor_status == IDLE || vals->motor_status == ACCEL || vals->motor_status == BRAKE) && (vals->u16_watchdog == 0 || !b_board_powered))
	{
		// transition 2
		vals->motor_status = OFF;
	}
	
	if (b_major_fault || vals->u8_motor_temp >= MAX_TEMP) //over current, over voltage, over temp
	{
		//transition 3
		vals->motor_status = ERR;
	}
}