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

void reset_I(void) ;
void set_I(uint8_t duty) ;
void controller(ModuleValues_t *vals);
void drivers(uint8_t b_state);
void drivers_init();
#endif /* CONTROLLER_H_ */