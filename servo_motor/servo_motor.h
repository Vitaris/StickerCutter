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
#include "../pid/PID.h"
#include "button.h"
#define MAN false
#define AUTO true
#define CUT_OFFSET 10.0
#define MAN_SPEED 5.0
#define FOLLOWING_ERROR 0.5 // Maximum permisible position deviation

enum op_state{SERVO_OK, ERR};
enum state{IN_POSITIONING, POSITIONING_DONE};

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
#define ENC_0 6
#define ENC_1 8

// First pin of PWM couple.
#define PWM_0 18
#define PWM_1 20

// Servo motor consist of: pwm, encoder, pid
typedef struct servo_motor {

	// Encoder
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
	char (*error_message)[21]; // Error message

	// PID
	// Velocity
	pidc_t pid_vel;
	float current_vel;
	float out_vel;
	float set_vel;

	// Servo controller
	bool pos_error_internal;

	// Feeder controller
	float stops[10];
	float next_stop;
	uint8_t no_of_stops;

	// Controller state
	char servo_name[10];

	// Servo controler 
	bool *enable;
	bool enable_previous;
	enum state state;
	float cycle_time;
	float computed_speed;
	bool positive_direction;
	bool movement_request;
	bool movement_in_progress;
	bool movement_done;
	bool braking;

	// Default movement
	float nominal_speed; 	// Desired motor speed
	float nominal_acc;		// Motor acceleration
	float current_speed; 	// Desired motor speed
	float current_acc;		// Motor acceleration
	float scale;			// Scale factor of the servo motor

	
	// Limits & Errors
	bool pos_limit_enabled;
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

	// Manual control
	button_t *man_plus;
	button_t *man_minus;

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
	 * @param man_plus Pointer to the manual plus button
	 * @param man_minus Pointer to the manual minus button
	 * @return returns a pidc_t controller handle
	 */
	servo_t servo_create(char servo_name[10], uint pio_ofset, uint sm, uint encoder_pin,
							uint pwm_pin, float scale, button_t *man_plus, button_t *man_minus, 
							bool *enable, bool *error, char (*message)[21]);

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

	void robust_pos_compute(servo_t servo);

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

	void servo_reset_all(servo_t servo);

	void servo_manual_handling(servo_t  servo);


#ifdef	__cplusplus
}
#endif

#endif
// End of Header file

