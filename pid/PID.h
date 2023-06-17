/*	Floating point PID control loop for Microcontrollers
	Copyright (C) 2014 Jesus Ruben Santa Anna Zamudio.

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

	Author website: http://www.geekfactory.mx
	Author e-mail: ruben at geekfactory dot mx
 */
#ifndef PID_H
#define PID_H
/*-------------------------------------------------------------*/
/*		Includes and dependencies			*/
/*-------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/*-------------------------------------------------------------*/
/*		Macros and definitions				*/
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
/*		Typedefs enums & structs			*/
/*-------------------------------------------------------------*/

/**
 * Structure that holds PID all the PID controller data, multiple instances are
 * posible using different structures for each controller
 */
struct pid_controller {
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
	// Time related
	uint32_t lasttime; //!< Stores the time when the control loop ran last time
	uint32_t sampletime; //!< Defines the PID sample time
};

typedef struct pid_controller * pidc_t;

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
	pidc_t pid_create(pidc_t pid, float* in, float* out, float* set, float kp, float ki, float kd);

	
	/**
	 * @brief Computes the output of the PID control
	 *
	 * This function computes the PID output based on the parameters, setpoint and
	 * current system input.
	 *
	 * @param pid The PID controller instance which will be used for computation
	 */
	void pid_compute(pidc_t pid);

	/**
	 * @brief Sets the limits for the PID controller output
	 *
	 * @param pid The PID controller instance to modify
	 * @param min The minimum output value for the PID controller
	 * @param max The maximum output value for the PID controller
	 */
	void pid_limits(pidc_t pid, float min, float max);


#ifdef	__cplusplus
}
#endif

#endif
// End of Header file
