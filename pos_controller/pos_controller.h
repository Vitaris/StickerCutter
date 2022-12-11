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
#ifndef POS_CONTROLLER_H
#define POS_CONTROLLER_H

#include "hardware/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/**
 * Structure that holds Positional controller data, multiple instances are
 * posible using different structures for each controller
 */
struct pos_controller {

	// IO
	float * in_pos;
	float * out_pos;

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


	// Input, output and setpoint
	float * input; //!< Current Process Value
	float * output; //!< Corrective Output from PID Controller
	float * setpoint; //!< Controller Setpoint
	// Tuning parameters
	float Kp; //!< Stores the gain for the Proportional term
	float Ki; //!< Stores the gain for the Integral term
	float Kd; //!< Stores the gain for the Derivative term
	// Output minimum and maximum values
	float omin; //!< Maximum value allowed at the output
	float omax; //!< Minimum value allowed at the output
	// Variables for PID algorithm
	float iterm; //!< Accumulator for integral term
	float lastin; //!< Last input value for differential term

};

typedef struct pos_controller * posc_t;

/*-------------------------------------------------------------*/
/*		Function prototypes				*/
/*-------------------------------------------------------------*/
#ifdef	__cplusplus
extern "C" {
#endif
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
	posc_t pos_control_create(posc_t pos, float* out, float acc, float speed);

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
	float pos_compute(posc_t pos, float in_pos);

	void pos_goto(posc_t pid, float position);
	void compute_path(posc_t pos, float input_pos);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file
