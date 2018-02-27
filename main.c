/*
 * PWMtest1803.c
 *
 * Created: 10.01.2018
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 
//CLKI/O 8MHz
#define USE_USART0
#define WATCHDOG_RELOAD_VALUE 500

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "speed.h"
#include "sensors.h"
#include "pid.h"
#include "controller.h"
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


// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Current;

//ADC buffers
static uint16_t u16_ADC0_reg = 0;
static uint16_t u16_ADC1_reg = 0;
static uint16_t u16_ADC2_reg = 0;
static uint16_t u16_ADC4_reg = 0;


static uint8_t send_can = 0;

//for SPI
static uint8_t u8_SPI_count = 0; 
static uint8_t u8_txBuffer[2]; 
static uint8_t u8_rxBuffer[3];

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

typedef struct{
	float f32_motor_current;
	float f32_batt_current;
	float f32_batt_volt;
	uint8_t u8_motor_temp;
	uint8_t u8_car_speed;
	uint8_t u8_throttle_cmd;
	uint8_t u8_duty_cycle ;
	uint16_t u16_watchdog ;
	MotorControllerState_t motor_status; // [||||||statebit2|statebit1]
	CarDirection_t Direction;
}ModuleValues_t;


ModuleValues_t ComValues = {
	.f32_motor_current = 0,
	.f32_batt_current = 0,
	.f32_batt_volt = 0,
	.u8_motor_temp = 0,
	.u8_car_speed = 0,
	.u8_throttle_cmd = 0, //in amps
	.u8_duty_cycle = 50,
	.u16_watchdog = WATCHDOG_RELOAD_VALUE,
	.motor_status = IDLE,
	.Direction = FORWARD,
};


void handle_can(ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx)){
		switch (rx->id){
			case FORWARD_CAN_ID:
			
				if (rx->data.u8[3] > 10)
				{
					ComValues.motor_status = FW_ACCEL ;
					vals->u8_throttle_cmd = rx->data.u8[3]/10.0 ;
				} else {
					ComValues.motor_status = IDLE ;
					vals->u8_throttle_cmd = 0;
				}
				
				if (rx->data.u8[2] > 25 && ComValues.motor_status == IDLE)
				{
					ComValues.motor_status = FW_BRAKE ;
					vals->u8_throttle_cmd = rx->data.u8[2]/10.0 ;
				}
				
				
				break;
			
			case BRAKE_CAN_ID:
				if (vals->Direction == FORWARD)
				{
					vals->motor_status = FW_BRAKE;
				} else {
					vals->motor_status = BW_BRAKE;
				}
				break;
		}
	}
}

void handle_motor_status_can_msg(uint8_t *send, ModuleValues_t *vals){
	if(*send){
		txFrame.data.u8[0] = vals->motor_status;
		txFrame.data.u8[1] = vals->u8_throttle_cmd;
		txFrame.data.u16[1] = (uint16_t)(vals->f32_motor_current);
		txFrame.data.u16[2] = OCR3B ;
		txFrame.data.u16[3] = vals->u8_car_speed;
		
		can_send_message(&txFrame);
		*send = 0;
	}
}

int main(void)	
{
	cli();
	pid_init(&Current, 0.1, 0.05, 0, 0);
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
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	
	sei();
	
	rgbled_turn_on(LED_BLUE);
	
    while (1){
		
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);
	
		//sends motor current and current cmd through USB
		/*uart_putint((uint16_t)(ComValues.f32_motor_current*1000));
		uart_puts(",");
		uart_putint(ComValues.u8_throttle_cmd*1000);
		uart_puts(",");
		uart_putint((uint16_t)((float)OCR3A/ICR3*1000));
		uart_puts("\r\n");*/
		
		printf("%i",(uint16_t)(ComValues.f32_motor_current*1000));
		printf(",");
		printf("%u",ComValues.u8_throttle_cmd*1000);
		printf(",");
		printf("%u",(uint16_t)(ComValues.u8_duty_cycle*10.0));
		printf("\n");
		
		/////////////////////receiving throttle cmd through USB
		if(uart_AvailableBytes()!=0){
			volatile uint16_t u16_data_received=uart_getint(); //in Amps. if >10, braking, else accelerating. eg : 12 -> brake 2 amps; 2 -> accel 2 amps
			uart_flush();
			if (u16_data_received >10 && u16_data_received <= 20)
			{
				ComValues.u8_throttle_cmd = u16_data_received-10 ;
				ComValues.motor_status = FW_BRAKE ;
			}
			if (u16_data_received>0 && u16_data_received <= 10)
			{
				ComValues.u8_throttle_cmd = u16_data_received ;
				ComValues.motor_status = FW_ACCEL;
			}
			if (u16_data_received == 0)
			{
				ComValues.u8_throttle_cmd = u16_data_received ;
				ComValues.motor_status = IDLE;
			}
		}	
	}
}


ISR(TIMER0_COMP_vect){ // every 5ms

	//send_can = 1;
	if (ComValues.f32_batt_volt > 15.0) //if motor controller card powered
	{
		if (ComValues.motor_status == FW_BRAKE || ComValues.motor_status == BW_ACCEL)
		{
			ComValues.u16_watchdog = WATCHDOG_RELOAD_VALUE ;
			drivers(1); //drivers turn on
			controller(-ComValues.u8_throttle_cmd, ComValues.f32_motor_current,&ComValues.u8_duty_cycle);
		}
	
		if (/*ComValues.motor_status == BW_BRAKE || */ComValues.motor_status == FW_ACCEL)
		{
			ComValues.u16_watchdog = WATCHDOG_RELOAD_VALUE ;
			drivers(1); //drivers turn on
			controller(ComValues.u8_throttle_cmd, ComValues.f32_motor_current, &ComValues.u8_duty_cycle);
		}
		if (ComValues.motor_status == IDLE)
		{
			/*if (ComValues.u16_watchdog == 0)
			{
				drivers(0);//drivers shutdown
				reset_I(); //reset integrator
			}else{
				ComValues.u16_watchdog -- ;
			}*/
			controller(0.0, ComValues.f32_motor_current,&ComValues.u8_duty_cycle);		
		}
	}else{
		drivers(0);//drivers shutdown
		reset_I(); //reset integrator
	}
	
	//handle_speed_sensor(&ComValues.u8_car_speed, &u16_speed_count, 100);
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
		Set_ADC_Channel_ext(4, u8_txBuffer);
		spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
		u8_rxBuffer[1]&= ~(0b111<<5);
		u16_ADC4_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
		u8_SPI_count = 0 ;
	}
	
	if (u8_SPI_count == 3)
	{
		u8_SPI_count ++ ;
	}
	
	if (u8_SPI_count == 2)
	{
		//batt volt
		Set_ADC_Channel_ext(2, u8_txBuffer);
		spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
		u8_rxBuffer[1]&= ~(0b111<<5);
		u16_ADC2_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
		u8_SPI_count ++ ;
	}
	
	if (u8_SPI_count == 1)
	{
		//batt current
		Set_ADC_Channel_ext(1, u8_txBuffer);
		spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
		u8_rxBuffer[1]&= ~(0b111<<5);
		u16_ADC1_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
		u8_SPI_count ++ ;
	}	
	
	if (u8_SPI_count == 0)
	{
		//motor current
		Set_ADC_Channel_ext(0, u8_txBuffer);
		spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
		u8_rxBuffer[1]&= ~(0b111<<5);
		u16_ADC0_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
		u8_SPI_count ++ ;
	}
	
	////////////////////INTERPRETATION OF RECEIVED ADC VALUES//////////////
	handle_current_sensor(&ComValues.f32_motor_current, u16_ADC0_reg);
	
	handle_current_sensor(&ComValues.f32_batt_current, u16_ADC1_reg);
	
	ComValues.f32_batt_volt = (float)u16_ADC2_reg/66.1 -0.37; // *5/4096 (12bit ADC with Vref = 5V) *0.1 (divider bridge 50V -> 5V) *coeff - offset(trimming)
	
	handle_temp_sensor(&ComValues.u8_motor_temp, u16_ADC4_reg);
}


ISR(INT5_vect)
{
	u16_speed_count ++ ;
}
