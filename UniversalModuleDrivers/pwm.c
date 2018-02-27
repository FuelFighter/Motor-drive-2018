/*
 * pwm.c
 *
 * Created: 24.03.2017 15:38:18
 *  Author: Jørgen Jackwitz
 */ 

#include "pwm.h"

#include <avr/interrupt.h>
#include <avr/io.h>

void pwm_init(void){
	
	//Set pwm_pins as output;
	PORTE &= ~((1<<PE3)|(1<<PE4));
	DDRE |= (1<<PE3)|(1<<PE4);
	
	//Timer 3 fast pwm, mode 14, TOP at ICR
	TCCR3B |= (1<<WGM33)|(1<<WGM32);
	TCCR3A |= (1<<WGM31);
	TCCR3A &= ~(1<<WGM30);

	// Non inverted PWM for A
	TCCR3A |= (1<<COM3A1);
	TCCR3A &= ~(1<<COM3A0);
	
	// Inverted PWM for B
	TCCR3A |= (1<<COM3B1);
	TCCR3A |= (1<<COM3B0);
	
	//Set prescale clk/1 for timer 3
	
	TCCR3B |= (1<<CS30);
	TCCR3B &= ~((1<<CS32)|(1<<CS31));
	
	//Set top value for timer 3
	ICR3 = 0x199; //20kHz
	
	OCR3A = (int)((0.5)*ICR3) ; //PWM_PE3 (non inverted)
	OCR3B = OCR3A ; //PWM_PE4 (inverted)
}

/*
void pwm_set_duty_cycle(pwmPin_t pin, uint16_t dutyCycle)
{
	switch (pin)
	{
		case PWM_PB4:
			OCR2A = (dutyCycle & 0xFF);
			break;
		case PWM_PE3:
			OCR3A = (dutyCycle & 0xFFFF);
			break;
		case PWM_PE4:
			OCR3B = (dutyCycle & 0xFFFF);
			break;
		case PWM_PE5:
			OCR3C = (dutyCycle & 0xFFFF);
		default:
			break;
	}
}



void pwm_set_top_t3(uint16_t top)
{
	ICR3 = (top & 0xFFFF);
}

void pwm_set_prescale(pwmPrescale_t scale, pwmTimer_t timer)
{
	switch (scale)
	{
		case DISABLE:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS22)|(1<<CS21)|(1<<CS0));
				
			} else if(timer == PWM_T3) {
				TCCR3B &= ~((1<<CS32)|(1<<CS31)|(1<<CS30));
			}
			break;
		case SCALE_1:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS22)|(1<<CS21));
				TCCR2A |= ~((1<<CS20));
			} else if(timer == PWM_T3) {
				TCCR3B &= ~((1<<CS32)|(1<<CS31));
				TCCR3B |= (1<<CS30);
			} 
			break;
		case SCALE_8:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS22)|(1<<CS20));
				TCCR2A |= ~((1<<CS21));
			} else if(timer == PWM_T3) {
				TCCR3B &= ~((1<<CS32)|(1<<CS30));
				TCCR3B |= (1<<CS31);
			}
			break;
		case SCALE_32_T2_ONLY:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS22));
				TCCR2A |= (1<<CS21)|(1<<CS20);
			} 
			break;
		case SCALE_64:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS21)|(1<<CS20));
				TCCR2A |= (1<<CS22);
			} else if(timer == PWM_T3) {
				TCCR3B &= ~((1<<CS32));
				TCCR3B |= (1<<CS30)|(1<<CS31);
			}
			break;
		case SCALE_128_T2_ONLY:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS21));
				TCCR2A |= (1<<CS22)|(1<<CS20);
			} 
			break;
		case SCALE_256:
			if (timer == PWM_T2)
			{
				TCCR2A &= ~((1<<CS20));
				TCCR2A |= (1<<CS22)|(1<<CS21);
			} else if(timer == PWM_T3) {
				TCCR3B &= ~((1<<CS31)|(1<<CS30));
				TCCR3B |= (1<<CS32);
			}
			break;
		case SCALE_1024:
			if (timer == PWM_T2)
			{
				TCCR2A |= (1<<CS22)|(1<<CS21)|(1<<CS20);
			} else if(timer == PWM_T3) {
				TCCR3B &= ~((1<<CS31));
				TCCR3B |= (1<<CS32)|(1<<CS30);
			}
			break;
		default:
			break;
	}
}

*/