/*
 * sensors.h
 *
 * Created: 10/01/2018 17:30:08
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include <avr/io.h>
#ifndef SENSORS_H_
#define SENSORS_H_

void handle_current_sensor(float *f32_current, uint16_t u16_ADC_reg);
void handle_temp_sensor(uint8_t *u8_temp, uint16_t u16_ADC_reg);



#endif /* SENSORS_H_ */