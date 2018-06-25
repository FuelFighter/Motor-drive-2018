/*
 * controller.h
 *
 * Created: 18.03.2017 16:06:03
 *  Author: Tanguy Simon for DNV GL Fuel fighter
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <avr/io.h>
#include "pid.h"
#include "state_machine.h"
#include "motor_controller_selection.h"

#ifdef MOTOR_RE50_36V
#define R 0.244
#define L 0.000177
#define VOLT_SPEED_CST 158.0 //rmp/V
#endif

#ifdef MOTOR_RE50_48V
#define R 0.608
#define L 0.000423
#define VOLT_SPEED_CST 102.0 //rmp/V
#endif

#ifdef MOTOR_RE65_48V
#define R 0.365
#define L 0.000161
#define VOLT_SPEED_CST 77.8.0 //rmp/V
#endif

void reset_I(void) ;
void set_I(uint8_t duty) ;
void controller(volatile ModuleValues_t *vals);
void drivers(uint8_t b_state);
void drivers_init();
#endif /* CONTROLLER_H_ */