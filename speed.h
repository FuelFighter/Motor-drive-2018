/*
 * speed.h
 *
 * Created: 11/01/2018 17:34:43
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include <avr/io.h>
#ifndef SPEED_H_
#define SPEED_H_


void speed_init();
void handle_speed_sensor(uint8_t * u8_speed, uint16_t *u16_counter, uint8_t u8_period); // period in ms

#endif /* SPEED_H_ */