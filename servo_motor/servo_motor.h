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
#define FOLLOWING_ERROR 2.1 // Maximum permisible position deviation

enum op_state{SERVO_OK, ERR};
enum state{IN_POSITIONING,POSITIONING_DONE};
enum mode{MANUAL,POSITIONER,FEEDER};

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
#define ENC_0 6
#define ENC_1 8

// First pin of PWM couple.
#define PWM_0 18
#define PWM_1 20

typedef struct servo_motor {

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
	pidc_t pid_pos;
	float current_pos;
	float out_pos;
	float set_pos;
	bool *posError; // Pointer to bool
	char (*error_message)[16]; // Error message

	// PID
	// Velocity
	pidc_t pid_vel;
	float current_vel;
	float out_vel;
	float set_vel;

	// Positional controller
	float generated_pos;
	bool pos_error_internal;

	// Feeder controller

	float stops[10];
	uint8_t no_of_stops;

	// Controller state
	bool positioning_request;
	char (*servo_name)[10];

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
	
	// Limits & Errors
	bool pos_limit_enabled;
	float max_diff; // Max following error
	float max_pos;
	float min_pos;

	/**
	 * @brief Error code variable
	 * 
	 * @note 01 - following error
	 * @note 02 - max position overrun
	 * @note 03 - min position overrun
	 */
	uint8_t error_code;

	// Feeder
	float movement_start_time;
	float breaking_start_time;
	float acc_time;
	float acc_dist;
	float acc_progress_time;
	float movement_progress_time;
	float breaking_progress_time;
	float begin_pos;

	float computed_speed;

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
	button_t *man_plus;
	button_t *man_minus;


	// debug
	int a;
	int b;
	bool edge_1;
	bool edge_2;
	bool edge_3;
	float mem_speed;
	float mem_pos;
	// 

} * servo_t;

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
	 * @param man_plus Pointer to the manual plus button
	 * @param man_minus Pointer to the manual minus button
	 * @return returns a pidc_t controller handle
	 */
	servo_t servo_create(char (*servo_name)[10], uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin, float scale, enum mode mode, 
							button_t *man_plus, button_t *man_minus, bool *error, char (*message)[16]);

	/**
	 * @brief Computation function for teh servo motor, have to be called in a servo loop (1ms)
	 * @param servo A pointer to a servo_motor structure
	 */
	void servo_compute(servo_t servo, float cycle_time);

	/**
	 * @brief Send the servo to a position
	 * @param servo A pointer to a servo_motor structure
	 */
	void servo_goto(servo_t servo, float position, float speed);

	/**
	 * @brief Compute the speed from the encoder value
	 * @param servo A pointer to a servo_motor structure
	 * @return returns the speed in rev/s
	 */
	float enc2speed(int32_t enc, float current_cycle);

	/**
	 * @brief Compute the path to follow
	 * @param servo Pointer to the controller struct
	 */
	void compute_path_params(servo_t servo);

	/**
	 * @brief Compute the output position
	 * @param servo Pointer to the controller struct
	 * @param current_pos Current position 
	 */
	float pos_compute(servo_t servo, float current_pos);

	float pos_compute_2(servo_t servo, float delta_time, float current_pos);

	void robust_pos_compute(servo_t servo);

	/**
	 * @brief Compute the output position
	 * @param servo Pointer to the controller struct
	 * @param run Button state
	 */
	float speed_compute(servo_t servo, bool plus, bool minus);

	/**
	 * @brief Adds a stop to the array of stops
	 * @param servo Pointer to the controller struct
	 */
	void add_stop(servo_t servo);

	/**
	 * @brief Removes a stop from the array of stops
	 * @param servo Pointer to the controller struct
	 */
	void remove_stop(servo_t servo);

	/**
	 * @brief Feeder desc
	 * @param servo Pointer to the controller struct
	 */
	float feeder(servo_t servo);
	
	
	/**
	 * @brief Detects if there is a stop in breaking distance
	 * @param servo Pointer to the controller struct
	 * @return true if there is a stop in breaking distance
	 */
	bool stop_ahead(servo_t servo);

	/**
	 * @brief Compute the breaking distance at the current speed
	 * @param servo Pointer to the controller struct
	 * @return breaking distance
	 */
	float get_breaking_distance(servo_t servo);

	/**
	 * @brief Compute the distance to the next stop
	 * @param servo Pointer to the controller struct
	 * @return distance to the next stop
	 */
	float get_dist_to_stop(servo_t servo);


	float accelerate(servo_t servo);

	/**
	 * @brief Compute next position according to the continuous feeding
	 * @param servo Pointer to the controller struct
	 * @return next position
	 */
	float continuous_feeding(servo_t servo);

	/**
	 * @brief Compute next position according to the ramp slowdow
	 * @param servo Pointer to the controller struct
	 * @return next position
	 */
	float breaking_to(servo_t servo);

	/**
	 * @brief Check real servo position agains computed one and raise following error if needed
	 * @param servo Pointer to the controller struct
	 * @return void	 */
	void check_for_following_error(servo_t* servo);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file

