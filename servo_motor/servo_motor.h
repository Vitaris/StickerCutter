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
// #include "pos_controller.h"
#include "button.h"
#define MAN false
#define AUTO true
#define CUT_OFFSET 10.0
#define MAN_SPEED 5.0

enum state{IN_POSITIONING,POSITIONING_DONE};
enum mode{MANUAL,POSITIONER,FEEDER};

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
	float current_pos;
	float out_pos;
	float set_pos;

	// PID
	// Velocity
	struct pid_controller ctrldata_vel;
	pidc_t pid_vel;
	float current_vel;
	float out_vel;
	float set_vel;

	// Positional controller
	float generated_pos;

	// Feeder controller

	float stops[10];
	uint8_t no_of_stops;

	// Controller state
	bool positioning_request;

	
	// Time vars
	float current_cycle_time;
	uint64_t current_time;

	// Path generator
	float requested_pos;
	uint64_t start_time;
	float ramp_time;
	float last_speed;
	bool speed_reached;

	
	float t_ramp; 	// duration of acceleration(decceleration) 
	float s_ramp;	// distance travelled during acc(dec)
	float s_conts; 	// distance travelled during constant speed movement
	float t_const;	// duration of constant speed movement

	// Servo controler 
	enum state state;
	enum mode mode;
	float cycle_time;

	// Feeder
	float movement_start_time;
	float breaking_start_time;
	float acc_time;
	float acc_dist;
	float acc_progress_time;
	float breaking_progress_time;
	float begin_pos;

	bool movement_request;
	bool movement_in_progress;
	bool movement_finished;

	bool breaking_request;
	bool breaking_in_progress;
	bool breaking_finished;


	// Default movement
	float nominal_speed; 	// Desired motor speed
	float nominal_acc;		// Motor acceleration
	float scale;			// Scale factor of the servo motor

	// Mode, 0 = Manual, 1 = Automatic
	// bool mode;
	
	// Manual control
	bool *man_plus;
	bool *man_minus;


	// debug
	int a;
	int b;
	bool edge_1;
	bool edge_2;
	bool edge_3;
	float mem_speed;
	float mem_pos;
	// 

};

typedef struct servo_motor * servo_t;


#ifdef	__cplusplus
extern "C" {
#endif

	static int add_encoder_pio();

	/**
	 * @brief Creates a new PID controller
	 * 
	 * @param servo A pointer to a servo_motor structure
	 * @param pio_ofset Offset of the pio to use
	 * @param sm Offset of the state machine to use
	 * @param encoder_pin First pin to use for the encoder, second pin is encoder_pin + 1
	 * @param pwm_pin First PWM pin to use for the pwm, second pin is pwm_pin + 1
	 * @param mode 0 = Manual, 1 = Automatic
	 * @param man_plus 
	 * @param man_minus
	 *
	 * @return returns a pidc_t controller handle
	 */
	servo_t servo_create(servo_t servo, uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin, float scale, enum mode mode, 
							bool *man_plus, bool *man_minus);

	/**
	 * @brief Computation function for teh servo motor, have to be called in a servo loop (1ms)
	 * 
	 * @param servo A pointer to a servo_motor structure
	 *
	 */
	void servo_compute(servo_t servo, float cycle_time);

	void servo_goto(servo_t servo, float position, float speed);

	float enc2speed(int32_t enc, float current_cycle);


	/**
	 * @brief Compute the path to follow
	 * 
	 * @param servo Pointer to the controller struct
	 * 
	 */
	void compute_path_params(servo_t servo);

	/**
	 * @brief Compute the output position
	 * 
	 * @param servo Pointer to the controller struct
	 * @param current_pos Current position 
	 *
	 */
	float pos_compute(servo_t servo, float current_pos);

	float pos_compute_2(servo_t servo, float delta_time, float current_pos);

	/**
	 * @brief Compute the output position
	 * 
	 * @param servo Pointer to the controller struct
	 * @param run Button state
	 *
	 */
	float speed_compute(servo_t servo, bool plus, bool minus);

	/**
	 * @brief Adds a stop to the array of stops
	 * @param servo Pointer to the controller struct
	 *
	 */
	void add_stop(servo_t servo);

	/**
	 * @brief Removes a stop from the array of stops
	 * @param servo Pointer to the controller struct
	 *
	 */
	void remove_stop(servo_t servo);

	/**
	 * @brief Feeder desc
	 * @param servo Pointer to the controller struct
	 *
	 */
	float feeder(servo_t servo);
	
	
	bool stop_ahead(servo_t servo);
	float get_breaking_distance(servo_t servo);
	float get_dist_to_stop(servo_t servo);
	float accelerate(servo_t servo);
	float continuous_feeding(servo_t servo);
	float breaking_to(servo_t servo);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file

