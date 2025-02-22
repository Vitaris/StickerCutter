#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "servo_pwm.h"
#include "../pid/PID.h"
#include "button.h"

#define HALF_SECOND_DELAY 500
#define MANUAL_SPEED 100.0
#define AUTOMAT_SPEED_SLOW 15.0
#define AUTOMAT_SPEED_MID 100.0
#define AUTOMAT_SPEED_NORMAL 150.0
#define AUTOMAT_SPEED_FAST 200.0
#define MANUAL_NORMAL 150.0
#define MANUAL_FAST 200.0

typedef struct servo_motor {
	// Encoder
	uint sm;
	int32_t enc_old;

	// PWM
	uint pwm_slice;
	
	// PID
	// Position
	pidc_t pid_pos;
	float enc_position;
	float out_pos;
	float set_pos;

	// PID
	// Velocity
	pidc_t pid_vel;
	float enc_speed;
	float out_vel;
	float set_vel;

	// Error handling
	bool *error; // Pointer to bool
	char (*error_message)[21]; // Error message
	bool pos_error_internal;

	// Feeder controller
	float stops[10];
	float next_stop;
	uint8_t no_of_stops;

	// Controller state
	char servo_name[10];

	// Servo controler
	enum positioning{
		IDLE,
		REQUESTED,
		ACCELERATING,
		BRAKING,
		POSITION_REACHED
	} positioning;

	float servo_position;
	float servo_speed;
	uint32_t delay_start;
	uint32_t delay_finish;
	bool *enable;
	bool enable_previous;
	float computed_speed;
	bool positive_direction;
	bool set_zero;
	bool nominal_speed_reached;
	float enc_offset;

	// Default movement
	float nominal_speed; 	// Desired motor speed
	float nominal_acc;		// Motor acceleration
	float current_speed; 	// Desired motor speed
	float current_acc;		// Motor acceleration
	float scale;			// Scale factor of the servo motor

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

/**
 * @brief Creates and initializes a new servo motor controller
 * 
 * @param servo_name Name identifier for the servo (max 9 chars)
 * @param pio_ofset PIO program offset
 * @param sm State machine number
 * @param encoder_pin First encoder pin (A phase)
 * @param pwm_pin First PWM pin
 * @param scale Position scaling factor
 * @param man_plus Manual forward button
 * @param man_minus Manual reverse button
 * @param enable Global enable signal
 * @param error Global error signal
 * @param message Error message buffer
 * @return Initialized servo controller handle
 */
servo_t servo_create(const char servo_name[10], const uint pio_ofset, const uint sm, 
					const uint encoder_pin, const uint pwm_pin, const float scale,
					button_t *const man_plus, button_t *const man_minus, 
					bool *const enable, bool *const error, char (*const message)[21]);

/**
 * @brief Main servo computation function, called in 1ms loop
 * @param servo Servo controller handle
 */
void servo_compute(servo_t servo);

/**
 * @brief Commands servo movement with start delay
 * @param servo Servo controller handle
 * @param position Target position
 * @param speed Movement speed
 * @param delay Start delay in milliseconds
 */
void servo_goto_delayed(servo_t const servo, const float position, const float speed, const uint32_t delay);

/**
 * @brief Commands immediate servo movement
 * @param servo Servo controller handle
 * @param position Target position
 * @param speed Movement speed
 */
void servo_goto(servo_t const servo, const float position, const float speed);

/**
 * @brief Handles manual jog control
 * @param servo Servo controller handle
 * @param min Minimum position limit
 * @param max Maximum position limit
 * @param homed True if home position is set
 */
void servo_manual_handling(servo_t const servo, const float min, const float max, const bool homed);

/**
 * @brief Commands servo to stop with controlled deceleration
 * @param servo Servo controller handle
 */
void stop_positioning(servo_t const servo);

#endif
