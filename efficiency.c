/*
 * efficiency.c
 *
 * Created: 14.04.2017 12:10:32
 *  Author: Ultrawack
 */ 

#include "motorefficiencies.h"
#include <avr/io.h>
#define MOTOR_ID 1
#define WHEEL_TO_MOTOR1_RPM 10
#define WHEEL_TO_MOTOR2_RPM 14
#define MOTOR_RPM_STEP 50
#define MOTOR_CURRENT_STEP 1
#define MAX_TORQUE_M1 1200
#define STEP_TO_TORQUE 20


uint16_t efficient_gain(uint16_t rpmWheel, uint8_t desired_torque)
{
	uint16_t motor1_rpm_step = (rpmWheel * WHEEL_TO_MOTOR1_RPM)/MOTOR_RPM_STEP;
	uint16_t motor2_rpm_step = (rpmWheel * WHEEL_TO_MOTOR2_RPM)/MOTOR_RPM_STEP;
	
	uint16_t highest_efficiency = 0;
	uint16_t step_efficiency = 0;

	uint8_t DESIRED_TORQUE_STEP = desired_torque/STEP_TO_TORQUE; //Number of increments until desired torque
	
	uint16_t motor1_torque_gain = 0;
	uint16_t motor2_torque_gain = 0;	
	
	for (int TORQUE_STEP = 0; TORQUE_STEP <= DESIRED_TORQUE_STEP; TORQUE_STEP++)
	{
		step_efficiency = motor1[motor1_rpm_step][TORQUE_STEP] + motor2[motor2_rpm_step][DESIRED_TORQUE_STEP-TORQUE_STEP];
		
		
		if (step_efficiency > highest_efficiency)
		{
			highest_efficiency = step_efficiency;
			motor1_torque_gain = TORQUE_STEP*STEP_TO_TORQUE; //Convert to from step to torque

			// If distribution of gain exceed limit of M1
				if (motor1_torque_gain > MAX_TORQUE_M1) {
					motor1_torque_gain = MAX_TORQUE_M1;
				}

			motor2_torque_gain = (DESIRED_TORQUE_STEP-TORQUE_STEP)*STEP_TO_TORQUE; //
		}
	}
	
	if (MOTOR_ID == 1)
	{
		return motor1_torque_gain;
	} else if (MOTOR_ID == 2)
	{
		return motor2_torque_gain;
	} else {
		return 0x00;
	}	
		
}