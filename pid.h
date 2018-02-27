/*
 * pid.h
 *
 * Created: 5/1/2017 11:37:40 PM
 *  Author: Ole
 */ 


#ifndef PID_H_
#define PID_H_

#include <avr/io.h>
#include <stdint.h>
#include "UniversalModuleDrivers/usbdb.h"

typedef struct{

	int lastError;
	int totError;
	int intError;
	int derError;

	float timeStep;
	float Kp;
	float Kd;
	float Ki;
} Pid_t;

void pid_init(Pid_t *PID, float t, float p, float i, float d);

int32_t pid(Pid_t *PID, uint16_t currentValue, uint16_t setpoint);

#endif /* PID_H_ */