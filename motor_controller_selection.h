/*
 * motor_controller_selection.h
 *
 * Created: 5/9/2017 6:57:30 PM
 *  Author: Ole
 */ 


#ifndef MOTOR_CONTROLLER_SELECTION_H_
#define MOTOR_CONTROLLER_SELECTION_H_

#include "UniversalModuleDrivers/can.h"
#include "state_machine.h"

//////////////////////// DEFINES TO CHANGE BEFORE PROGRAMMING /////////////////
//chose the correct PCB connected to the UM you program
#define MC_BOARD_1
// MC_BOARD_2
// MC_BOARD_3

// for the speed, choose the sensor you are using, 
//and the number of magnets (they need to be precisely mounted at equal distances around the disk)
#define SPEED_SENSOR_HALL
// SPEED_SENSOR_REED
#define NUM_MAGNETS 1

//choose the motor you are using
#define MOTOR_RE50_36V
// MOTOR_RE50_48V
// MOTOR_RE65_48V

// According to the position in the car (right or left) the MC has to be selected here (1 = right, 2 = left)
// the actuator board has to be programmed accordingly (with the right CAN IDs, see CAN bus frame description on the drive)
#define MOTOR_CONTROLLER_1
// MOTOR_CONTROLLER_2

//Enabling the UART communication 
//Transmit is always on and is reliable. 
//Reception through UART has lead to some unreliabilities and should be deactivated when unused
#define ENABLE_UART_TX
///////////////////////////////////////////////////////////////////////////////////////////

//  for MC
#ifdef MOTOR_CONTROLLER_1
#define MOTOR_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define MOTOR_SELECT(for1, for2) (for2)
#endif

#define MOTOR_CAN_ID					MOTOR_SELECT(MOTOR_1_STATUS_CAN_ID, MOTOR_2_STATUS_CAN_ID)

//for rx clutch

#ifdef MOTOR_CONTROLLER_1
#define CLUTCH_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define CLUTCH_SELECT(for1, for2) (for2)
#endif

#define E_CLUTCH_CAN_ID					CLUTCH_SELECT(E_CLUTCH_1_CAN_ID, E_CLUTCH_2_CAN_ID)

//for tx clutch

#ifdef MOTOR_CONTROLLER_1
#define CLUTCH_CMD_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define CLUTCH_CMD_SELECT(for1, for2) (for2)
#endif

#define MOTOR_CL_CMD_CAN_ID					CLUTCH_CMD_SELECT(MOTOR_1_CL_CMD_CAN_ID, MOTOR_2_CL_CMD_CAN_ID)

#endif /* MOTOR_CONTROLLER_SELECTION_H_ */