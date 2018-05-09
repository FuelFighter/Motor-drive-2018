/*
 * speed.h
 *
 * Created: 11/01/2018 17:34:43
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include <avr/io.h>
#include "state_machine.h"

#ifndef SPEED_H_
#define SPEED_H_


void speed_init();
void handle_speed_sensor(volatile uint16_t *u16_speed, volatile uint16_t *u16_counter); //speed in m/s
uint8_t compute_synch_duty(volatile uint8_t speed_ms, ClutchState_t gear, float vbatt);

#endif /* SPEED_H_ */
