/*
 * motor_controller_selection.h
 *
 * Created: 5/9/2017 6:57:30 PM
 *  Author: Ole
 */ 


#ifndef MOTOR_CONTROLLER_SELECTION_H_
#define MOTOR_CONTROLLER_SELECTION_H_

#include "UniversalModuleDrivers/can.h"

// To choose motor controller, comment out opposite
#define MOTOR_CONTROLLER_1
//#define MOTOR_CONTROLLER_2

#ifdef MOTOR_CONTROLLER_1
#define MOTOR_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define MOTOR_SELECT(for1, for2) (for2)
#endif

#define MOTOR_CAN_ID					MOTOR_SELECT(MOTOR_1_STATUS_CAN_ID, MOTOR_2_STATUS_CAN_ID)


typedef enum {
	IDLE = 0,
	FW_ACCEL = 1,
	FW_BRAKE = 2,
	BW_ACCEL = 3,
	BW_BRAKE = 4
} MotorControllerState_t;

typedef enum 
{
	FORWARD = 0,
	BACKWARD = 1
} CarDirection_t;

typedef enum
{
	NEUTRAL = 0,
	GEAR1 = 1,
	GEAR2 = 2
} ClutchState_t ;

typedef struct{
	float f32_motor_current;
	float f32_batt_current;
	float f32_batt_volt;
	float f32_energy ;
	uint8_t u8_motor_temp;
	uint8_t u8_car_speed;
	uint8_t u8_throttle_cmd;
	uint8_t u8_duty_cycle ;
	uint16_t u16_watchdog ;
	MotorControllerState_t motor_status; // [||||||statebit2|statebit1]
	CarDirection_t Direction;
	ClutchState_t clutch;
}ModuleValues_t;

#endif /* MOTOR_CONTROLLER_SELECTION_H_ */