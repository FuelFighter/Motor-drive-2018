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

void SPI_handler_0(volatile float * p_f32_motcurrent) // motor current
{
	Set_ADC_Channel_ext(0, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC0_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_current_sensor(p_f32_motcurrent, u16_ADC0_reg,0);
}

void SPI_handler_1(volatile float * f32_batcurrent) // battery current
{
	Set_ADC_Channel_ext(1, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC1_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_current_sensor(f32_batcurrent, u16_ADC1_reg,1);
}

void SPI_handler_2(volatile float * f32_batvolt) //battery voltage
{
	Set_ADC_Channel_ext(2, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC2_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	*f32_batvolt = VOLT_CONVERSION_OFFSET+(float)u16_ADC2_reg/VOLT_CONVERSION_COEFF;
}

void SPI_handler_4(volatile uint8_t * u8_mottemp) //motor temperature
{
	Set_ADC_Channel_ext(4, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC4_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_temp_sensor(u8_mottemp, u16_ADC4_reg);
}


///////////////////////  CAN  /////////////////////////


//receiving
void handle_can(volatile ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx) && vals->motor_status != ERR){
		switch (rx->id){
			case DASHBOARD_CAN_ID	: //receiving can messages from the steering wheel
				
				vals->message_mode = CAN ;
				vals->ctrl_type = CURRENT ;
				vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE ; // resetting to max value each time a message is received.
				if (rx->data.u8[3] > 10)
				{
					vals->u8_accel_cmd = rx->data.u8[3]/10 ; 
					vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE ;
				}
				
				if (rx->data.u8[2] > 27)
				{
					vals->u8_brake_cmd = rx->data.u8[2]/10 ;
					vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE ;
				}
				
				if (rx->data.u8[2] <= 27)
				{
					vals->u8_brake_cmd = 0;
				}
				if (rx->data.u8[3] <= 10)
				{
					vals->u8_accel_cmd = 0;
				}
				
			break;
			
			case E_CLUTCH_CAN_ID :
				vals->pwtrain_type = GEAR ;
				vals->u16_motor_speed = rx->data.u16[0] ; //receiving motor speed from encoder from clutch board
				vals->gear_status = rx->data.u8[2] ; //receiving gear status from the clutch board
			break;
		}
	}
}

//sending
void handle_motor_status_can_msg(volatile ModuleValues_t vals){
	
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	
	txFrame.data.u8[0] = vals.motor_status;
	txFrame.data.i8[1] = (int8_t)(vals.f32_motor_current*10);
	txFrame.data.u16[1] = (uint16_t)(vals.f32_batt_volt*10);
	txFrame.data.u16[2] = (uint16_t)abs((int16_t)vals.f32_energy/100.0) ;
	txFrame.data.u8[6] = (uint8_t)(vals.u16_car_speed*3.6*0.5) ; //sent in km/h
	txFrame.data.u8[7] = vals.u8_motor_temp;
		
	can_send_message(&txFrame);
}

void handle_clutch_cmd_can_msg(volatile ModuleValues_t vals){
	
	txFrame1.id = MOTOR_CL_CMD_CAN_ID;
	txFrame1.length = 1;

	txFrame1.data.u8[0] = vals.gear_required;
		
	can_send_message(&txFrame1);
}

///////////////////  UART  ////////////////////

//receiving 
void receive_uart(volatile ModuleValues_t * vals)
{
	if(uart_AvailableBytes()!=0){
		vals->message_mode = UART ;
		int16_t i16_data_received=uart_getint();
		uart_flush();
		vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE ;
		if (vals->ctrl_type == CURRENT)
		{
			if (i16_data_received > -10 && i16_data_received < 0) //limited braking and acceleration at 10A
			{
				vals->u8_brake_cmd = -i16_data_received ;
			}
			if (i16_data_received > 0 && i16_data_received < 10) //limited braking and acceleration at 10A
			{
				vals->u8_accel_cmd = i16_data_received ;
			}
				
		}else if (vals->ctrl_type == PWM)
		{
			vals->u8_duty_cycle = i16_data_received;
		}
	}
}

//sending
//sends motor current and current cmd through USB
void send_uart(volatile ModuleValues_t vals)
{
	//printf("%i,%i,%u,%u,%u,%u,%i",(int16_t)(vals.f32_motor_current*1000),(int16_t)(vals.f32_batt_current*1000),(uint16_t)(vals.f32_batt_volt*1000),vals.u8_car_speed,vals.u8_duty_cycle,vals.u8_motor_temp,vals.i8_throttle_cmd);
	printf("\r\n");
	
	/*printf("batt curr : %i",(int16_t)(vals.f32_batt_current*10));
	printf(",");
	printf(" batt volt : %u",(uint16_t)(vals.f32_batt_volt*10));
	printf(",");
	printf(" mot curr : %i",(int16_t)(vals.f32_motor_current*10));
	printf(",");
	printf(" duty : %u",vals.u8_duty_cycle);
	printf(",");
	if (vals.u8_accel_cmd > 0)
	{
		printf(" cmd : %u",vals.u8_accel_cmd);
	}else
	{
		printf(" cmd : %i",-(int8_t)vals.u8_brake_cmd);
	}
	printf(",");
	printf("speed km/h : %u",(uint16_t)(vals.u16_car_speed*3.6));
	printf(",");
	printf(" gear req : %u",vals.gear_required);
	printf(",");
	printf(" gear status : %u",vals.gear_status);
	printf(",");
	printf(" ctrl mode (0:Cur, 1:PWM) : %u",vals.ctrl_type);
	printf(",");
	printf(" motor mode : %u",vals.motor_status);
	*/
	
	printf("%i",(int16_t)(vals.f32_motor_current*1000));
	printf(",");
	printf("%u",vals.u8_duty_cycle);
	printf(",");
	printf("%u",vals.u8_accel_cmd*1000);
	printf(",");
	printf("%u",(uint16_t)(vals.u16_car_speed*360.0));	// vehicle speed in m/h
	printf(",");
	//printf("%u",(uint16_t)(vals.u16_motor_speed/4.29));
	printf("%u",(uint16_t)(vals.u16_motor_speed*5.03));	// vehicle speed in m/h = rpm*18/375/60*2*pi*0.556/2*3.6
	printf(",");
	printf("%u",(uint16_t)(vals.f32_batt_volt*10));
	printf(",");
	printf("%i",(int16_t)(vals.f32_batt_current*1000));
	
}

///////////////// LED /////////////////////
void manage_LEDs(volatile ModuleValues_t vals)
{	
	switch (vals.motor_status)
	{
		case OFF :
			rgbled_turn_off(LED_GREEN);
			rgbled_turn_on(LED_BLUE);
			if (vals.u16_watchdog_can == 0) //no can messages
			{
				rgbled_turn_on(LED_RED);
			}else{
				rgbled_turn_off(LED_RED);
			}
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