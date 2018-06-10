/*
 * sensors.h
 *
 * Created: 10/01/2018 17:30:08
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include <avr/io.h>
#include "state_machine.h"

#ifndef SENSORS_H_
#define SENSORS_H_
//// CURRENT TRANSDUCERS ////
#define TRANSDUCER_SENSIBILITY 0.0416
#define TRANSDUCER_OFFSET 2.52
#define CORRECTION_OFFSET_BAT 0.0
#define CORRECTION_OFFSET_MOT 0.2
#define LOWPASS_CONSTANT 0.1

// board specific offsets : [BAT, MOT]
//1 : [-0.2, 0]
//2 : [0.0,0.2]
//3 : [0.2,0.05]
//4 : [,]
//5 : [,]

//// VOLTAGE MEASUREMENT ////
//used in DigiCom.c
// *5/4096 (12bit ADC with Vref = 5V) *0.1 (divider bridge 50V -> 5V) *coeff - offset(trimming)
#define VOLT_CONVERSION_COEFF 68.5
#define VOLT_CONVERSION_OFFSET 0.3
// board specific coefficients and offsets :
//1 : 68.5
//2 : 68.5 +0.3
//3 : 68.5 +0.3
//4 : 
//5 : 


void handle_current_sensor(volatile float *f32_current, uint16_t u16_ADC_reg, uint8_t u8_sensor_num);
void handle_temp_sensor(volatile uint8_t *u8_temp, uint16_t u16_ADC_reg);
void handle_joulemeter(volatile float *f32_energy,volatile float f32_bat_current,volatile float f32_bat_voltage,volatile uint8_t u8_time_period);

#endif /* SENSORS_H_ */