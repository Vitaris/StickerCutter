#include "PID.h"
#include <stdlib.h>

pidc_t pid_create(float* in, float* out, float* set, float kp, float ki, float kd)
{
	// Create PID data structure
	pidc_t pid = (pidc_t)malloc(sizeof(struct pid_controller));

	// Initialize PID data structure by given parameters
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;

	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	pid->iterm = 0;

	// Set default limits
	pid_limits(pid, -1024, 1024);
		
	return pid;
}


void pid_compute(pidc_t pid)
{
	float in = *(pid->input);
	// Compute error
	float error = (*(pid->setpoint)) - in;

	// Compute integral
	pid->iterm += (pid->Ki * error);
	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
	// Compute differential on input
	float dinput = in - pid->lastin;
	// Compute PID output
	float out = pid->Kp * error + pid->iterm - pid->Kd * dinput;
	// Apply limit to output value
	if (out > pid->omax)
		out = pid->omax;
	else if (out < pid->omin)
		out = pid->omin;
	// Output to pointed variable
	(*pid->output) = out;
	// Keep track of some variables for next execution
	pid->lastin = in;
}

void pid_limits(pidc_t pid, float min, float max)
{
	if (min >= max) return;
	pid->omin = min;
	pid->omax = max;

	//Adjust output to new limits
	if (*(pid->output) > pid->omax)
		*(pid->output) = pid->omax;
	else if (*(pid->output) < pid->omin)
		*(pid->output) = pid->omin;

	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
}

void pid_reset_all(pidc_t pid) {
	pid->iterm = 0.0;
	pid->lastin = 0.0; 
}