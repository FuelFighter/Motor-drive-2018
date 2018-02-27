/*
 * adc.c
 *
 * Created: 18.02.2017 10:41:35
 *  Author: olesot, free running by Tanguy Simon
 */ 

//Written for UM

#include "adc.h"
#include <avr/io.h>

void adc_init(void){
	
	/* Voltage ref AVcc with external capacitor on AREF pin */
	ADMUX |= (1<<REFS0);
	
	/* Select prescaler to 64 --> conversion f= 125kHz */
	ADCSRA |= (1<<ADPS2)|(1<<ADPS2);
	
	/* Enable the ADC */
	ADCSRA |= (1<<ADEN);
}

void adc_Free_running_init(void){ //WORKS - check precision
	
	/* Voltage ref AVcc with external capacitor on AREF pin */
	ADMUX |= (1<<REFS0);
	
	/* Select prescaler to 64 --> conversion f= 125kHz */
	ADCSRA |= (1<<ADPS2);
	
	/* Auto trigger of ADC */
	ADCSRA |= (1<<ADATE);
	
	/* Enable Interrupt of ADC */
	ADCSRA |= (1<<ADIE);
	
	/* Free running mode */
	ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); // (initially at 0 so not useful)
	
	/* Enable the ADC */
	ADCSRA |= (1<<ADEN);
	
	/* Start the conversion */
	ADCSRA |= (1<<ADSC);
}

uint16_t adc_read(adc_channel_t channel){
	
	//Setting channel and type of reading, see enum in adc.h 
	ADMUX &= 0b11100000;
	ADMUX |= (int8_t)channel;	
		
	/* Start the conversion */
	ADCSRA |= (1<<ADSC);
	
	/* Wait for the conversion to complete */
	while(ADCSRA & (1<<ADSC));
	
	return (ADCL+(ADCH<<8));
}

uint16_t adc_Free_running_read(adc_channel_t channel){ 
	//Setting channel and type of reading, see enum in adc.h
	ADMUX &= 0b11100000;
	ADMUX |= (int8_t)channel;

	return (ADCL+(ADCH<<8));
}

void Set_ADC_Channel(adc_channel_t channel)
{
	channel &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|channel; // clears the bottom 3 bits before ORing
}

void Set_ADC_Channel_ext(uint8_t u8_CHn, uint8_t * u8_ADC_tx)
{
	switch(u8_CHn)
	{
		case 0 :
			u8_ADC_tx[0] = 0b00000110 ;
			u8_ADC_tx[1] = 0b00 ;
		break;
		
		case 1 :
			u8_ADC_tx[0] = 0b00000110 ;
			u8_ADC_tx[1] = 0b01 << 6 ;
		break;
		
		case 2 :
			u8_ADC_tx[0] = 0b00000110 ;
			u8_ADC_tx[1] = 0b10 << 6 ;
		break;
		
		case 3 :
		u8_ADC_tx[0] = 0b00000110 ;
		u8_ADC_tx[1] = 0b11 << 6 ;
		break;
		
		case 4 :
		u8_ADC_tx[0] = 0b00000111 ;
		u8_ADC_tx[1] = 0b00 << 6 ;
		break;
		
		case 5 :
		u8_ADC_tx[0] = 0b00000111 ;
		u8_ADC_tx[1] = 0b01 << 6 ;
		break;
		
		case 6 :
		u8_ADC_tx[0] = 0b00000111 ;
		u8_ADC_tx[1] = 0b10 << 6 ;
		break;
		
		case 7 :
		u8_ADC_tx[0] = 0b00000111 ;
		u8_ADC_tx[1] = 0b11 << 6 ;
		break;
	}
}
/*
MCP3208 doc :
	ADC0 = 0b0000011000,
	ADC1 = 0b0000011001,
	ADC2 = 0b0000011010,
	ADC3 = 0b0000011011,
	ADC4 = 0b0000011100,
	ADC5 = 0b0000011101,
	ADC6 = 0b0000011110,
	ADC7 = 0b0000011111,
*/ 