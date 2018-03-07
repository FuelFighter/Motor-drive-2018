/*
 * PWMtest1803.c
 *
 * Created: 10.01.2018
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 
//CLKI/O 8MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "speed.h"
#include "sensors.h"
#include "controller.h"
#include "DigiCom.h"
#include "UniversalModuleDrivers/spi.h"
#include "UniversalModuleDrivers/timer.h"
#include "UniversalModuleDrivers/rgbled.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"
#include "UniversalModuleDrivers/uart.h"
#include "motor_controller_selection.h"
#include "AVR-UART-lib-master/usart.h"

#define USE_USART0

//counters for manual prescalers
static uint8_t can_sender_counter = 0;
static uint8_t speed_handler_counter = 0;

//for CAN
static uint8_t send_can = 0;

//for SPI
static uint8_t u8_SPI_count = 0; 

//for speed
static uint16_t u16_speed_count = 0;

void timer1_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11); // timer 1 prescaler set CLK/64
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0; //reset timer value
	TIMSK1 |= (1<<OCIE1A); //enable interrupt
	OCR1A = 125; //compare value //every 1ms
}

void timer0_init_ts(){ 
	TCCR0A |= (1<<CS02)|(1<<CS00); // timer 0 prescaler set CLK/1024
	TCCR0A |= (1<<WGM01); //CTC
	TCNT0 = 0; //reset timer value
	TIMSK0 |= (1<<OCIE0A); //enable interrupt
	OCR0A = 39; //compare value // 78 for 10ms, 39 for 5ms
} // => reload time timer 0 = 10ms

ModuleValues_t ComValues = {
	.f32_motor_current = 0.0,
	.f32_batt_current = 0.0,
	.f32_batt_volt = 0.0,
	.f32_energy = 0.0,
	.u8_motor_temp = 0,
	.u8_car_speed = 0,
	.u8_throttle_cmd = 0, //in amps
	.u8_duty_cycle = 50,
	.u16_watchdog = WATCHDOG_RELOAD_VALUE,
	.motor_status = IDLE,
	.clutch = NEUTRAL,
	.clutch_required = NEUTRAL
};

int main(void)	
{
	cli();
	pwm_init();
	can_init(0,0);
	timer1_init_ts();
	timer0_init_ts();
	speed_init();
	spi_init(DIV_4); // clk at clkio/4 = 2MHz init of SPI for external ADC device
	
	//uart_set_FrameFormat(USART_8BIT_DATA|USART_1STOP_BIT|USART_NO_PARITY|USART_ASYNC_MODE); // default settings
	uart_init(BAUD_CALC(500000)); // 8n1 transmission is set as default
	stdout = &uart0_io; // attach uart stream to stdout & stdin
	stdin = &uart0_io; // uart0_in and uart0_out are only available if NO_USART_RX or NO_USART_TX is defined
	
	rgbled_init();
	drivers_init();
	sei();
	
	rgbled_turn_on(LED_BLUE);

    while (1){
		
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);
	
		//sends motor current and current cmd through USB
		printf("%i",ComValues.u8_car_speed);
		printf(",");
		printf("%i",u16_speed_count);
		printf("\n");
		/*
		printf("%i",(uint16_t)(ComValues.f32_motor_current*1000));
		printf(",");
		printf("%u",ComValues.u8_throttle_cmd*1000);
		printf(",");
		printf("%u",(uint16_t)(ComValues.u8_duty_cycle*10.0));
		printf("\n");
		*/
		receive_uart(&ComValues);
	}
}


ISR(TIMER0_COMP_vect){ // every 5ms
	
	if (can_sender_counter == 1) // every 10ms
	{
		//handle_speed_sensor(&ComValues.u8_car_speed, &u16_speed_count, 10.0);
		handle_joulemeter(&ComValues.f32_energy, ComValues.f32_batt_current, ComValues.f32_batt_volt, 10) ;
		send_can = 1;
		can_sender_counter = 0;
	} else {
		can_sender_counter ++;
	}
	
	if (speed_handler_counter == 100) // every 1s
	{
		//handle_speed_sensor(&ComValues.u8_car_speed, &u16_speed_count, 1000);
		//for test
		ComValues.u8_car_speed ++ ;
		if (ComValues.u8_car_speed == 99)
		{
			ComValues.u8_car_speed = 0 ;
		}
		
		speed_handler_counter = 0;
		} else {
		speed_handler_counter ++;
	}
	
	manage_motor(&ComValues);
}


/////////////////////////////////////COMMUNICATION WITH EXTERNAL ADC////////////////////////////////
/*External ADC HW setup (on Motor Drive V2.0):
*	CH0 : Motor current
*	CH1 : Battery current
*	CH2 : Battery voltage
*	CH4 : Motor temperature
*/


ISR(TIMER1_COMPA_vect){// every 1ms

	if (u8_SPI_count == 4)
	{
		//motor temp
		SPI_handler_4(&ComValues.u8_motor_temp);
		u8_SPI_count = 0 ;
	}
	
	if (u8_SPI_count == 3)
	{
		u8_SPI_count ++ ;
	}
	
	if (u8_SPI_count == 2)
	{
		//batt volt
		SPI_handler_2(&ComValues.f32_batt_volt);
		u8_SPI_count ++ ;
	}
	
	if (u8_SPI_count == 1)
	{
		//batt current
		SPI_handler_1(&ComValues.f32_batt_current);
		u8_SPI_count ++ ;
	}	
	
	if (u8_SPI_count == 0)
	{
		//motor current
		SPI_handler_0(&ComValues.f32_motor_current);
		u8_SPI_count ++ ;
	}
}


ISR(INT5_vect)
{
	u16_speed_count ++ ;
}
