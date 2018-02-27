/*
 * sensors.c
 *
 * Created: 10/01/2018 17:28:30
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include "sensors.h"
#include <avr/io.h>

#define TRANSDUCER_SENSIBILITY 0.0416
#define TRANSDUCER_OFFSET 2.26
#define LOWPASS_CONSTANT 0.1

void handle_current_sensor(float *f32_current, uint16_t u16_ADC_reg)
{
	volatile float f_new_current = ((((float)u16_ADC_reg*5/4096) - TRANSDUCER_OFFSET)/TRANSDUCER_SENSIBILITY)/3 ;// /3 because current passes 3x in transducer for more precision.
	f_new_current = (f_new_current-1.76);// correction of offset and ramp error (conversion + hardware) measured with ampmeter of the power supply : bad
	*f32_current = (*f32_current)*(1-LOWPASS_CONSTANT) + LOWPASS_CONSTANT*f_new_current ;// low pass filter ---------------------TODO test
}

void handle_temp_sensor(uint8_t *u8_temp, uint16_t u16_ADC_reg)
{
	volatile float f_sens_volt = ((float)u16_ADC_reg*5/4096);
	// give temp by three linear approxiations : 
	// 0 -> 3.7V => T = 20*V-22
	// 3.7 -> 4.7V => T = 55.5*V-155.5
	// 4.7 -> 5V => T = 220*V-840
	// this approximation system is used because it requires less processing power and variable accuracy than the 3rd order polyfit.
	
	if (f_sens_volt <= 3.7)
	{
		*u8_temp = (uint8_t)(20*f_sens_volt-22);
	}
	
	if (f_sens_volt <= 4.7 && f_sens_volt > 3.7)
	{
		*u8_temp = (uint8_t)(55.5*f_sens_volt-155.5);
	}
	
	if (f_sens_volt > 4.7)
	{
		*u8_temp = (uint8_t)(200*f_sens_volt-840);
	}
}