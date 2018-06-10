/*
 * sensors.c
 *
 * Created: 10/01/2018 17:28:30
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include "sensors.h"
#include <avr/io.h>

void handle_current_sensor(volatile float *f32_current, uint16_t u16_ADC_reg, uint8_t u8_sensor_num)
{
	volatile float f_new_current = ((((volatile float)u16_ADC_reg*5.0/4096.0) - TRANSDUCER_OFFSET)/TRANSDUCER_SENSIBILITY) ;// /3 because current passes 3x in transducer for more precision.
	if (u8_sensor_num)
	{//batt
		f_new_current = (f_new_current+CORRECTION_OFFSET_BAT);// correction of offset
	}else{
		f_new_current = (f_new_current+CORRECTION_OFFSET_MOT);// correction of offset
	}
	
	*f32_current = (*f32_current)*(1-LOWPASS_CONSTANT) + LOWPASS_CONSTANT*f_new_current ;// low pass filter ---------------------TODO test
}

void handle_temp_sensor(volatile uint8_t *u8_temp, uint16_t u16_ADC_reg)
{
	float f_sens_volt = ((float)u16_ADC_reg*5.0/4096.0);
	//Thermistors NTC 495-75654-ND
	// give temp by three linear approxiations : 
	// 0 -> 3.7V => T = 20*V-22
	// 3.7 -> 4.7V => T = 55.5*V-155.5
	// 4.7 -> 5V => T = 220*V-840
	// this approximation system is used because it requires less processing power and variable accuracy than the 3rd order polyfit. 
	// Here we approximate the curve by three straight lines
	
	if (f_sens_volt <= 3.7)
	{
		*u8_temp = (uint8_t)(20.0*f_sens_volt-22.0);
	}
	
	if (f_sens_volt <= 4.7 && f_sens_volt > 3.7)
	{
		*u8_temp = (uint8_t)(55.5*f_sens_volt-155.5);
	}
	
	if (f_sens_volt > 4.7)
	{
		*u8_temp = (uint8_t)(200.0*f_sens_volt-840.0);
	}
}

void handle_joulemeter(volatile float *f32_energy, volatile float f32_bat_current, volatile float f32_bat_voltage, uint8_t u8_time_period) //units : A, V, ms
{
	*f32_energy += f32_bat_voltage*f32_bat_current*(float)u8_time_period/1000 ;
}