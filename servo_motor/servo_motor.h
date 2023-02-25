/*	Positional controller for Microcontrollers
	Copyright (C) 2022

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "hardware/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Quadrature encoder
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"

#include "servo_pwm.h"
// #include "pid/PID.h"
#include "../pid/PID.h"
#include "pos_controller.h"


struct servo_motor {

	// Servo motor consist of:
	// pwm, encoder, pid, pos_controller

	// Encoder
	float enc_position;
	float enc_velocity;
	int32_t enc_old;
	uint sm;


	// PWM
	uint pwm_slice;
	
	// PID
	// Position
	struct pid_controller ctrldata_pos;
	pidc_t pid_pos;
	float in_pos;
	float out_pos;
	float set_pos;

	// PID
	// Velocity
	struct pid_controller ctrldata_vel;
	pidc_t pid_vel;
	float in_vel;
	float out_vel;
	float set_vel;

	// Positional controller
	struct pos_controller pos_ctrData;
	posc_t pos;
	float generated_pos;

	// Controller state
	bool positioning_request;
	bool in_positioning;
	bool positioning_done;

	// Time vars
	float current_cycle_time;
	uint64_t last_start_time;

	// Path generator
	float requested_pos;
	float current_pos;
	uint64_t start_time;
	float ramp_time;
	float t_ramp; 
	float s_ramp; 
	float s_conts;
	float t_const;

	// Default movement
	float speed; 	// Desired motor speed
	float acc;		// Motor acceleration



};

typedef struct servo_motor * servo_t;

/*-------------------------------------------------------------*/
/*		Function prototypes				*/
/*-------------------------------------------------------------*/
#ifdef	__cplusplus
extern "C" {
#endif

	static int add_encoder_pio();

	/**
	 * @brief Creates a new PID controller
	 *
	 * Creates a new pid controller and initializes it�s input, output and internal
	 * variables. Also we set the tuning parameters
	 *
	 * @param pid A pointer to a pid_controller structure
	 * @param in Pointer to float value for the process input
	 * @param out Poiter to put the controller output value
	 * @param set Pointer float with the process setpoint value
	 * @param kp Proportional gain
	 * @param ki Integral gain
	 * @param kd Diferential gain
	 *
	 * @return returns a pidc_t controller handle
	 */
	servo_t servo_motor_create(servo_t motor, uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin);

	/**
	 * @brief Creates a new PID controller
	 *
	 * Creates a new pid controller and initializes it�s input, output and internal
	 * variables. Also we set the tuning parameters
	 *
	 * @param pid A pointer to a pid_controller structure
	 * @param in Pointer to float value for the process input
	 * @param out Poiter to put the controller output value
	 * @param set Pointer float with the process setpoint value
	 * @param kp Proportional gain
	 * @param ki Integral gain
	 * @param kd Diferential gain
	 *
	 * @return returns a pidc_t controller handle
	 */
	void motor_compute(servo_t motor);

	void motor_goto(servo_t motor, float position, float speed);

	float enc2speed(int32_t enc);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file
