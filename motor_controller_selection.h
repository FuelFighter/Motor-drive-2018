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

// To choose motor controller, comment out opposite
//#define MOTOR_CONTROLLER_1
#define MOTOR_CONTROLLER_2
/////////////////  for motor  ////////////
#ifdef MOTOR_CONTROLLER_1
#define MOTOR_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define MOTOR_SELECT(for1, for2) (for2)
#endif

#define MOTOR_CAN_ID					MOTOR_SELECT(MOTOR_1_STATUS_CAN_ID, MOTOR_2_STATUS_CAN_ID)

/////////////////  for rx clutch  ////////////

#ifdef MOTOR_CONTROLLER_1
#define CLUTCH_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define CLUTCH_SELECT(for1, for2) (for2)
#endif

#define E_CLUTCH_CAN_ID					CLUTCH_SELECT(E_CLUTCH_1_CAN_ID, E_CLUTCH_2_CAN_ID)

/////////////////  for tx clutch  ////////////

#ifdef MOTOR_CONTROLLER_1
#define CLUTCH_CMD_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define CLUTCH_CMD_SELECT(for1, for2) (for2)
#endif

#define MOTOR_CL_CMD_CAN_ID					CLUTCH_CMD_SELECT(MOTOR_1_CL_CMD_CAN_ID, MOTOR_2_CL_CMD_CAN_ID)

#endif /* MOTOR_CONTROLLER_SELECTION_H_ */