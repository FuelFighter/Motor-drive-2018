/*
 * PWMtest1803.c
 *
 * Created: 10.01.2018
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 
////////////////  TODO  ////////////
/*
* Gear2 ? /!\ reverse motion
*
*
*
*
*/
////////////////  DESCRIPTION  ////////////
/* The motorcontroller has CAN interface with the dashboard and the electrical clutch
* It has a UART interface with a computer (serialPlot, arduino IDE, Atmel studio serial interface and Simulink)
* There are two modules in the car (1 & 2) with their corresponding clutch. (choose this in motor_controller_selectrion.h)
* It can be controlled in PWM (only through UART) or Current
* It can control the belt powertrain (default) or the Gear powertrain (upon reception of clutch CAN message)
* The main only has timer definition and systic handlers.
* The SPI, UART, CAN communication and state LEDs are managed in DigiCom.c
* controller.c manages the modulator and current loop
* the sensors.c manages conversions from the current, voltage and temperature sensors.
* state_machine.c manages the different states of the motorcontroller, the inter-state transitions and actions during each state.
* speed.c is dedicated to the speed counter (reed switch with magnets on the wheel) and Synchronous speed duty cycle to engage the gears.

//////////////////////// WHEN PROGRAMMING A UM///////////////
* Look into motor_controller_selection.h and choose the correct defines.
*/

//CLKI/O = 8MHz

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
#include "state_machine.h"
#include "AVR-UART-lib-master/usart.h"

#define USE_USART0

//counters for manual prescalers
uint8_t systic_counter_fast = 0;
uint16_t systic_counter_slow = 0;

//for CAN
uint8_t b_send_can = 0;
uint8_t b_select_can_msg = 0;

//for UART
uint8_t b_send_uart = 0;

//for SPI
uint8_t u8_SPI_count = 0; 

//for speed
volatile uint16_t u16_speed_count = 0;


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
	OCR0A = 39; //compare value // 78 for 10ms, 39 for 5ms, 19 for 2.56ms
} // => reload time timer 0 = 10ms

volatile ModuleValues_t ComValues = {
	.f32_motor_current = 0.0,
	.f32_batt_current = 0.0,
	.f32_batt_volt = 0.0,
	.f32_energy = 0.0,
	.u8_motor_temp = 0,
	.u16_car_speed = 0,
	.u16_motor_speed = 0,
	.u8_accel_cmd = 0, //in amps
	.u8_brake_cmd = 0, //in amps
	.u8_duty_cycle = 50,
	.u16_watchdog_can = 0,
	.u16_watchdog_throttle = 0,
	.motor_status = OFF,
	.message_mode = CAN,
	.gear_status = NEUTRAL,
	.gear_required = NEUTRAL,
	.b_driver_status = 0,
	.ctrl_type = CURRENT,
	.pwtrain_type = BELT
};

int main(void)	
{
	cli();
	rgbled_init();
	//rgbled_turn_on(LED_BLUE);
	DWC_init();
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
	drivers(0);
	sei();
	
    while (1){
		
		handle_can(&ComValues, &rxFrame); //receive CAN
		//receive_uart(&ComValues);
		
		if (b_send_can)
		{
			if (b_select_can_msg)// sending one or the other
			{
				handle_motor_status_can_msg(ComValues); //send motor status on CAN
				b_select_can_msg = 0;
			}else{
				handle_clutch_cmd_can_msg(ComValues); // send clutch command on CAN
				b_select_can_msg = 1;
			}
			b_send_can = 0;
		}
		
		if (b_send_uart)
		{
			send_uart(ComValues);
			b_send_uart = 0;
		}
	}
}


ISR(TIMER0_COMP_vect){ // every 5ms
	handle_DWC(&ComValues); // sets accel and brake cmds to 0
	state_handler(&ComValues);
	if (systic_counter_fast == 7) // every 41ms
	{
		b_send_can = 1;
		b_send_uart = 1;
		if (ComValues.u16_watchdog_can != 0 && ComValues.message_mode == CAN) //if in uart ctrl mode (see Digicom.h), the watchdog is not used
		{
			ComValues.u16_watchdog_can -- ;
		}
		
		if (ComValues.u16_watchdog_throttle != 0 && ComValues.message_mode == CAN) //if in uart ctrl mode (see Digicom.h), the watchdog is not used
		{
			ComValues.u16_watchdog_throttle -- ;
		}else if (ComValues.message_mode == UART)
		{
			ComValues.u16_watchdog_throttle = 0;
		}
		
		handle_joulemeter(&ComValues.f32_energy, ComValues.f32_batt_current, ComValues.f32_batt_volt, 41) ;		
		systic_counter_fast = 0;
	} else {
		systic_counter_fast ++;
	}
	
	if (systic_counter_slow == 100) // every 0.5s 
	{
		manage_LEDs(ComValues); //UM LED according to motor state
		systic_counter_slow = 0;
		} else {
		systic_counter_slow ++;
	}
}


/////////////////////////////////////COMMUNICATION WITH EXTERNAL ADC////////////////////////////////
/*External ADC HW setup (on Motor Drive V2.0):
*	CH0 : Motor current
*	CH1 : Battery current
*	CH2 : Battery voltage
*	CH4 : Motor temperature
*/


ISR(TIMER1_COMPA_vect){// every 1ms
	
	if (u16_speed_count < 2000 ) //after 3s with no magnet, speed = 0
	{
		u16_speed_count ++ ;
	} else
	{
		ComValues.u16_car_speed = 0;
		u16_speed_count = 0;
	}
	
	if (u8_SPI_count == 3)
	{
		//motor temp
		SPI_handler_4(&ComValues.u8_motor_temp);
		u8_SPI_count = 0 ;
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


ISR(INT5_vect) //interrupt on rising front of the speed sensor (each time a magnet passes in front of the reed switch)
{
	//rgbled_toggle(LED_GREEN);
	handle_speed_sensor(&ComValues.u16_car_speed, &u16_speed_count);
}
