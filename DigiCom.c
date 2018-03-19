/*
 * DigiCom.c
 *
 * Created: 04/03/2018 14:12:59
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include <stdlib.h>
#include <avr/io.h>
#include "DigiCom.h"
#include "sensors.h"
#include "controller.h"
#include "UniversalModuleDrivers/adc.h"
#include "UniversalModuleDrivers/spi.h"
#include "UniversalModuleDrivers/rgbled.h"
#include "AVR-UART-lib-master/usart.h"

//ADC buffers
static uint16_t u16_ADC0_reg = 0;
static uint16_t u16_ADC1_reg = 0;
static uint16_t u16_ADC2_reg = 0;
static uint16_t u16_ADC4_reg = 0;

//for SPI
static uint8_t u8_txBuffer[2];
static uint8_t u8_rxBuffer[3];

/////////////////////////  SPI  /////////////////////////

void SPI_handler_0(float * f32_motcurrent) // motor current
{
	Set_ADC_Channel_ext(0, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC0_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_current_sensor(f32_motcurrent, u16_ADC0_reg);
}

void SPI_handler_1(float * f32_batcurrent) // battery current
{
	Set_ADC_Channel_ext(1, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC1_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_current_sensor(f32_batcurrent, u16_ADC1_reg);
}

void SPI_handler_2(float * f32_batvolt) //battery voltage
{
	Set_ADC_Channel_ext(2, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC2_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	*f32_batvolt = (float)u16_ADC2_reg/66.1 -0.37; // *5/4096 (12bit ADC with Vref = 5V) *0.1 (divider bridge 50V -> 5V) *coeff - offset(trimming)
}

void SPI_handler_4(uint8_t * u8_mottemp) //motor temperature
{
	Set_ADC_Channel_ext(4, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC4_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_temp_sensor(u8_mottemp, u16_ADC4_reg);
}


///////////////////////  CAN  /////////////////////////


//receiving
void handle_can(ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx) && vals->motor_status != ERR){
		switch (rx->id){
			case STEERING_WHEEL_CAN_ID	: //receiving can messages from the steering wheel
				
				vals->u16_watchdog = WATCHDOG_RELOAD_VALUE ; // resetting to max value each time a message is received.

				if (rx->data.u8[3] > 10)
				{
					vals->motor_status = ACCEL ;
					vals->u8_throttle_cmd = rx->data.u8[3]/10.0 ;
				} else {
					vals->motor_status = IDLE ;
					vals->u8_throttle_cmd = 0;
				}
				
				if (rx->data.u8[2] > 25 && vals->motor_status == IDLE)
				{
					vals->motor_status = BRAKE ;
					vals->u8_throttle_cmd = rx->data.u8[2]/10.0 ;
				}
			break;
		}
	}
}

//sending
void handle_motor_status_can_msg(uint8_t *send, ModuleValues_t *vals){
	
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	
	if(*send){
		txFrame.data.u8[0] = vals->motor_status;
		txFrame.data.u8[1] = 0;
		txFrame.data.u16[1] = (uint16_t)(vals->f32_motor_current);
		txFrame.data.u16[2] = (uint16_t)(vals->f32_energy*1000) ;
		txFrame.data.u16[3] = (uint16_t)(vals->u8_car_speed) ;
		
		can_send_message(&txFrame);
		*send = 0;
	}
}

///////////////////  UART  ////////////////////

//receiving 
void receive_uart(ModuleValues_t * vals)
{
	if(uart_AvailableBytes()!=0 && vals->motor_status != ERR){
		volatile uint16_t u16_data_received=uart_getint(); //in Amps. if >10, braking, else accelerating. eg : 12 -> brake 2 amps; 2 -> accel 2 amps
		uart_flush();
		if (vals->ctrl_type == CURRENT)
		{
			if (u16_data_received >10 && u16_data_received <= 20)
			{
				vals->u8_throttle_cmd = u16_data_received-10 ;
				vals->motor_status = BRAKE ;
			}
			if (u16_data_received>0 && u16_data_received <= 10)
			{
				vals->u8_throttle_cmd = u16_data_received ;
				vals->motor_status = ACCEL;
			}
			if (u16_data_received == 0)
			{
				vals->u8_throttle_cmd = u16_data_received ;
				vals->motor_status = IDLE;
			}
		}else if (vals->ctrl_type == PWM)
		{
			vals->u8_duty_cycle = u16_data_received;
			vals->motor_status = ACCEL;
		}
	}
}

//sending
//sends motor current and current cmd through USB
void send_uart(ModuleValues_t vals)
{
	//printf("%i,%i,%u,%u,%u,%u,%i",(int16_t)(vals.f32_motor_current*1000),(int16_t)(vals.f32_batt_current*1000),(uint16_t)(vals.f32_batt_volt*1000),vals.u8_car_speed,vals.u8_duty_cycle,vals.u8_motor_temp,vals.u8_throttle_cmd);

// 	printf("%i",(int16_t)(vals.f32_batt_current*1000));
// 	printf(",");
// 	printf("%u",(uint16_t)(vals.f32_batt_volt*1000));
// 	printf(",");
// 	printf("%u",vals.u8_car_speed);
// 	printf(",");
	printf("%u",(uint16_t)(vals.f32_batt_volt)*100);
 	printf(",");
 	printf("%u",vals.u8_duty_cycle*10);
 	printf(",");
// 	printf("%u",vals.u8_motor_temp);
// 	printf(",");
 	if (vals.motor_status == BRAKE)
 	{
 		printf("%i",-vals.u8_throttle_cmd*1000);
 	}else
 	{
 		printf("%i",vals.u8_throttle_cmd*1000);
 	}
	printf(",");
	printf("%i",(int16_t)(vals.f32_motor_current*1000));
	printf("\n");
}

///////////////// LED /////////////////////
void manage_LEDs(ModuleValues_t vals)
{	
	switch (vals.motor_status)
	{
		case OFF :
			rgbled_turn_off(LED_GREEN);
			rgbled_turn_off(LED_RED);
			rgbled_turn_on(LED_BLUE);
		break ;
		
		case ACCEL :
			rgbled_turn_off(LED_RED);
			rgbled_turn_off(LED_BLUE);
			rgbled_toggle(LED_GREEN);
		break;
		
		case BRAKE :
			rgbled_turn_off(LED_RED);
			rgbled_turn_off(LED_BLUE);
			rgbled_toggle(LED_GREEN);
		break;
		
		case IDLE :
			rgbled_turn_off(LED_RED);
			rgbled_turn_off(LED_BLUE);
			rgbled_turn_on(LED_GREEN);
		break;
		
		case ERR :
			rgbled_turn_off(LED_GREEN);
			rgbled_turn_off(LED_BLUE);
			rgbled_turn_on(LED_RED);
		break;
	}
}