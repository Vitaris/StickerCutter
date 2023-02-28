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
};

typedef struct pos_controller * posc_t;


#ifdef	__cplusplus
extern "C" {
#endif

	/**
	 * @brief Positional controler initialization
	 * 
	 * @param pos Pointer to the controller struct
	 * @param out Pointer to the output variable
	 * @param acc Motor acceleration
	 * @param speed Motor speed
	 *
	 */
	posc_t pos_control_create(posc_t pos, float* out, float acc, float speed);

	/**
	 * @brief Compute the output position
	 * 
	 * @param pos Pointer to the controller struct
	 * @param in_pos Current position 
	 *
	 */
	float pos_compute(posc_t pos, float in_pos);

	/**
	 * @brief Compute the output position
	 * 
	 * @param pos Pointer to the controller struct
	 * @param in_speed Current speed
	 *
	 */
	float speed_compute(posc_t pos, bool run, float in_speed);

	/**
	 * @brief Function to request a positioning
	 * 
	 * @param pos Pointer to the controller struct
	 * @param position Requested position
	 * @param speed Requested speed
	 *
	 */
	void pos_goto(posc_t pos, float position, float speed);

	/**
	 * @brief Compute the path to follow
	 * 
	 * @param pos Pointer to the controller struct
	 * @param input_pos Current position
	 * 
	 */
	void compute_path(posc_t pos, float input_pos);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file
