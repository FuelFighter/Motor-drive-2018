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
static uint8_t systic_counter_fast = 0;
static uint16_t systic_counter_slow = 0;

//for CAN
static uint8_t send_can = 0;

//for UART
uint8_t b_send_uart = 0;

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
	.motor_status = OFF,
	.clutch = NEUTRAL,
	.clutch_required = NEUTRAL,
	.b_driver_status = 0,
	.ctrl_type = CURRENT
};

int main(void)	
{
	cli();
	rgbled_init();
	//rgbled_turn_on(LED_BLUE);

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
	drivers_init();
	
	sei();
	
    while (1){
		
		handle_motor_status_can_msg(&send_can, &ComValues); //send CAN
		handle_can(&ComValues, &rxFrame); //receive CAN
		
		if (b_send_uart)
		{
			send_uart(ComValues);
			b_send_uart = 0;
		}
		receive_uart(&ComValues);
		
		err_check(&ComValues); //verifying current, temperature and voltage
	}
}


ISR(TIMER0_COMP_vect){ // every 5ms
	
	if (systic_counter_fast == 1) // every 10ms
	{
		b_send_uart = 1;
		if (ComValues.u16_watchdog == 0)
		{
			if (ComValues.motor_status != ERR)
			{
				ComValues.motor_status = OFF ;
			}
			}else{
			ComValues.u16_watchdog -- ;
		}
		handle_joulemeter(&ComValues.f32_energy, ComValues.f32_batt_current, ComValues.f32_batt_volt, 10) ;		
		systic_counter_fast = 0;
	} else {
		systic_counter_fast ++;
	}
	
	if (systic_counter_slow == 100) // every 0.5s 
	{
		send_can = 1;
		handle_speed_sensor(&ComValues.u8_car_speed, &u16_speed_count, 500.0);
		manage_LEDs(ComValues); //UM LED according to motor state
		systic_counter_slow = 0;
		} else {
		systic_counter_slow ++;
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


ISR(INT5_vect) //interrupt on rising front of the speed sensor (each time a magnet passes in frot of the reed switch)
{
	u16_speed_count ++ ;
}
