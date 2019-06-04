/*
 * state_machine.h
 *
 * Created: 22/04/2018 16:02:28
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : not hardware specific 
*/

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#define WATCHDOG_CAN_RELOAD_VALUE 50
#define WATCHDOG_THROTTLE_RELOAD_VALUE 30

//////////////  TYPES  ///////////////
typedef enum {
	OFF = 0, // power or CAN disconnected
	ACCEL_GEAR1 = 1, //Receiving ACCEL cmd and gear1
	ACCEL_GEAR2 = 2, //Receiving ACCEL cmd and gear2
	BRAKE_GEAR1 = 3, //Receiving BRAKE cmd and gear1
	BRAKE_GEAR2 = 4, //Receiving BRAKE cmd and gear2
	IDLE = 5, //receiving 0 current cmd (car rolling, current loop is running with 0A cmd
	ERR = 6, //error mode
	ENGAGE = 7 //waiting for the clutch to engage
} MotorControllerState_t;

typedef enum
{
	CAN = 1,
	UART = 0
} MsgMode_t;

typedef enum
{
	NEUTRAL = 0,
	GEAR1 = 1,
	GEAR2 = 2,
	BELTGEAR = 3
} ClutchState_t ;

typedef enum
{
	BELT,
	GEAR
}PowertrainType_t;

typedef enum
{
	CURRENT = 0,
	PWM = 1
} ControlType_t ;

typedef struct{
	float f32_motor_current;
	float f32_batt_current;
	float f32_batt_volt;
	float f32_energy ;
	uint8_t u8_motor_temp;
	uint16_t u16_car_speed;
	uint16_t u16_motor_speed;
	uint8_t u8_accel_cmd;
	uint8_t u8_brake_cmd;
	uint8_t u8_duty_cycle ;
	uint16_t u16_watchdog_can ;
	uint16_t u16_watchdog_throttle ;
	MotorControllerState_t motor_status; // [||||||statebit2|statebit1]
	MsgMode_t message_mode;
	ClutchState_t gear_status;
	ClutchState_t gear_required;
	uint8_t b_driver_status;
	ControlType_t ctrl_type;
	PowertrainType_t pwtrain_type;

}ModuleValues_t;

////////////////  PROTOTYPES   /////////////////
void state_handler(volatile ModuleValues_t * vals);

#endif /* STATE_MACHINE_H_ */