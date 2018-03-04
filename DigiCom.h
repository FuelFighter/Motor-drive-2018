/*
 * DigiCom.h
 *
 * Created: 04/03/2018 14:13:20
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 


#ifndef DIGICOM_H_
#define DIGICOM_H_

void SPI_handler_0(float * f32_motcurrent); // motor current
void SPI_handler_1(float * f32_batcurrent); // battery current
void SPI_handler_2(float * f32_batvolt); //battery voltage
void SPI_handler_4(uint8_t * u8_mottemp); //motor temperature
#endif /* DIGICOM_H_ */