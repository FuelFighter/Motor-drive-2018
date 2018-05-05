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
	
	if (SW_MODE == BIPOLAR)
	{
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
		ICR3 = 0x85; //30kHz  0x85, 20kHz : 0x100

		
		OCR3A = (int)((0.5)*ICR3) ; //PWM_PE3 (non inverted)
		OCR3B = OCR3A ; //PWM_PE4 (inverted)
		
	}else{//UNIPOLAR
		
		//Timer 3 phase correct pwm, TOP at ICR (mode 10)
		TCCR3B |= (1<<WGM33);
		TCCR3B &= ~(1<<WGM32);
		TCCR3A |= (1<<WGM31);
		TCCR3A &= ~(1<<WGM30);
		
		// Set OC3A on Compare Match when up-counting. clear OC3A on Compare Match when downcounting.
		TCCR3A |= (1<<COM3A1);
		TCCR3A &= ~(1<<COM3A0);
		
		// Set OC3B on Compare Match when up-counting. Clear OC3B on Compare Match when downcounting.
		TCCR3A |= (1<<COM3B1);
		TCCR3A &= ~(1<<COM3B0);
		
		//Set prescale clk/1 for timer 3
		TCCR3B |= (1<<CS30);
		TCCR3B &= ~((1<<CS32)|(1<<CS31));
		
		//Set top value for timer 3
		ICR3 = 0x85; //30kHz  0x85, 20kHz : 0x100
		
		//initialising compare registers at Duty cycle 50%
		OCR3A = (int)((0.5)*ICR3) ; //PWM_PE3
		OCR3B = ICR3-OCR3A ; //PWM_PE4
	}
}
