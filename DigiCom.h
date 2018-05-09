/*
 * DigiCom.h
 *
 * Created: 04/03/2018 14:13:20
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 


#ifndef DIGICOM_H_
#define DIGICOM_H_

#include "motor_controller_selection.h"
#include "state_machine.h"
#include "UniversalModuleDrivers/can.h"


// CAN Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
CanMessage_t rxFrame1;
CanMessage_t txFrame1;

///////////////// PROTOTYPES //////////////s

//SPI
void SPI_handler_0(volatile float * f32_motcurrent); // motor current
void SPI_handler_1(volatile float * f32_batcurrent); // battery current
void SPI_handler_2(volatile float * f32_batvolt); //battery voltage
void SPI_handler_4(volatile uint8_t * u8_mottemp); //motor temperature

//CAN
void handle_motor_status_can_msg(volatile ModuleValues_t vals); //sending status
void handle_clutch_cmd_can_msg(volatile ModuleValues_t vals); //sending required gear to clutch
void handle_can(volatile ModuleValues_t *vals, CanMessage_t *rx); //receiving

//UART
void receive_uart(volatile ModuleValues_t * vals);
void send_uart(volatile ModuleValues_t vals);

//LEDs
void manage_LEDs(volatile ModuleValues_t vals);


#endif /* DIGICOM_H_ */