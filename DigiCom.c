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
	
	handle_current_sensor(f32_motcurrent, u16_ADC0_reg,0);
}

void SPI_handler_1(float * f32_batcurrent) // battery current
{
	Set_ADC_Channel_ext(1, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC1_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_current_sensor(f32_batcurrent, u16_ADC1_reg,1);
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
			case DASHBOARD_CAN_ID	: //receiving can messages from the steering wheel
				
				vals->message_mode = CAN ;
				vals->ctrl_type = CURRENT ;
				vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE ; // resetting to max value each time a message is received.

				if (rx->data.u8[3] > 10 && (vals->motor_status == IDLE || vals->motor_status == ACCEL))
				{
					vals->i8_throttle_cmd = rx->data.u8[3]/10 ; 
					vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE ;
				}
				
				if (rx->data.u8[2] > 25 && (vals->motor_status == IDLE || vals->motor_status == BRAKE))
				{
					vals->i8_throttle_cmd = -rx->data.u8[2]/10 ;
					vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE ;
				}
				
				if (rx->data.u8[2] <= 25 && rx->data.u8[3] <= 10)
				{
					vals->i8_throttle_cmd = 0;
				}
				
			break;
			
			case E_CLUTCH_CAN_ID :
				vals->pwtrain_type = GEAR ;
				vals->gear_status = rx->data.u8[0] ; //receiving gear status from the clutch
			break;
		}
	}
}

//sending
void handle_motor_status_can_msg(ModuleValues_t vals){
	
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	
	txFrame.data.u8[0] = vals.motor_status;
	txFrame.data.u8[1] = vals.u8_duty_cycle;
	txFrame.data.u16[1] = (uint16_t)(vals.f32_motor_current);
	txFrame.data.u16[2] = (uint16_t)(vals.f32_energy*1000) ;
	txFrame.data.u16[3] = vals.u8_car_speed*36 ; //100 = 10.0km/h
	//txFrame.data.u16[4] = vals.u8_car_speed-(vals.u8_car_speed/10)*10 ; //decimal part
	//add motor temp
		
	can_send_message(&txFrame);
}

void handle_clutch_cmd_can_msg(ModuleValues_t vals){
	
	txFrame.id = MOTOR_CL_CMD_CAN_ID;
	txFrame.length = 1;

	txFrame.data.u8[0] = vals.gear_required;
		
	can_send_message(&txFrame);
}

///////////////////  UART  ////////////////////

//receiving 
uint8_t u8_uart_welcome = 0;
void receive_uart(ModuleValues_t * vals)
{
	if(uart_AvailableBytes()!=0){
		
		if (u8_uart_welcome == 2 && vals->message_mode == UART)
		{
			volatile int16_t i16_data_received=uart_getint();
			uart_flush();
			
			if (i16_data_received == 11)
			{
				vals->ctrl_type = CURRENT;
				printf("\nCURRENT control mode activated.\n");
				vals->b_send_uart_data = 1;
				vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE ;
			}
			
			if (i16_data_received == 12)
			{
				vals->ctrl_type = PWM;
				printf("\nPWM control mode activated.\n");
				vals->b_send_uart_data = 1;
				vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE ;
			}
			
			if (i16_data_received == 13)
			{
				vals->u16_watchdog_can = 0;
				printf("\nTurning off.\n");
				printf("Drivers deactivated.\n");
				vals->b_send_uart_data = 0;
			}
			
			if (vals->ctrl_type == CURRENT)
			{
				if (i16_data_received > -10 && i16_data_received < 10) //limited braking and acceleration at 10A
				{
					vals->i8_throttle_cmd = i16_data_received ;
				}
				
			}else if (vals->ctrl_type == PWM)
			{
				vals->u8_duty_cycle = i16_data_received;
			}
		}
		
		if (u8_uart_welcome == 1)
		{
			volatile int16_t i16_choice_received=uart_getint();
			uart_flush();
			if (i16_choice_received == 1)
			{//can ctrl
				printf("\n CAN control mode. sending data.\n");
				vals->b_send_uart_data = 1;
				u8_uart_welcome = 2;
			}
			if (i16_choice_received == 2)
			{//uart ctrl
				vals->message_mode = UART ;
				u8_uart_welcome = 2;
				printf("UART message mode activated.\n\n");
				
				printf("for CURRENT control mode, write: 11, followed by a space\n");
				printf("for PWM control mode, write:     12, followed by a space\n");
				printf("to turn OFF, write:              13, followed by a space\n\n");
				
				printf("In CURRENT, for acceleration or braking, write the number of Amps (integers, [-10 10]), followed by a space\n\n");
				
				printf("In PWM, write the duty cycle (integers, [5 95]), followed by a space\n");
			}
		}
		
		if (u8_uart_welcome == 0)
		{
			//first uart cmd received
			printf("Welcome to the UART interface of motor drive v2.1.\n");
			printf("To use the CAN bus as control medium, type: 1 followed by a space.\n");
			printf("To use the UART as control medium, type:    2 followed by a space.\n");
			volatile int16_t useless=uart_getint();
			uart_flush();
			u8_uart_welcome = 1;
		}
	}
}

//sending
//sends motor current and current cmd through USB
void send_uart(ModuleValues_t vals)
{
	if (vals.b_send_uart_data)
	{
		//printf("%i,%i,%u,%u,%u,%u,%i",(int16_t)(vals.f32_motor_current*1000),(int16_t)(vals.f32_batt_current*1000),(uint16_t)(vals.f32_batt_volt*1000),vals.u8_car_speed,vals.u8_duty_cycle,vals.u8_motor_temp,vals.i8_throttle_cmd);
		printf("%i",(int16_t)(vals.f32_batt_current*1000));
		printf(",");
		printf("%u",(uint16_t)(vals.f32_batt_volt*1000));
		printf(",");
		printf("%i",(int16_t)(vals.f32_motor_current*1000));
		printf(",");
		printf("%u",vals.u8_duty_cycle);
		printf(",");
		printf("%i",vals.i8_throttle_cmd);
		printf("\n");
	}
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
		
		case ENGAGE :
			rgbled_turn_off(LED_RED);
			rgbled_turn_on(LED_GREEN);
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