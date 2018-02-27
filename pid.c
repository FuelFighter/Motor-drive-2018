/*
 * pid.c
 *
 * Created: 24.03.2017 13:52:07
 *  Author: Jorgen Jackwitz
 */ 

#include "pid.h"

#define TIMECONSTANT 1000000
#define GAINCONSTANT 1000

int32_t pid(Pid_t *PID, uint16_t currentValue, uint16_t setpoint){
	int output = 0;

	PID->totError = setpoint - currentValue;
	// Proportional gain
	int propGain = (PID->Kp)*(PID->totError);
	// Integration gain
	PID->intError = PID->intError + (PID->timeStep)*(PID->totError);
	int intGain = PID->intError*(PID->Ki);
	// Derivation gain
	PID->derError = (PID->totError - PID->lastError)/(PID->timeStep);
	int derGain = (PID->derError)*(PID->Kd);
	PID->lastError = PID->totError;
	/*
	printf("totError: %d \t", PID->totError);
	printf("IntError: %d \t", PID->intError);
	printf("DerError: %d \t",PID->derError);

	printf("PropGain: %d \t", propGain);
	printf("IntGain: %d \t", intGain);
	printf("DerGain: %d \t", derGain);
	*/
	// Pid output
	output = propGain + intGain + derGain;
	//printf("Out: %d \n", output);
	return output;
}

void pid_init(Pid_t *PID, float t, float p, float i, float d){
	PID->Kp = p;
	PID->Ki = i;
	PID->Kd = d;
	PID->timeStep = t;
}