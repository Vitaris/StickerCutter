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

typedef struct servo_motor servo_t;

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
servo_t* servo_create(const char servo_name[7], const uint pio_ofset, const uint sm, 
					const uint encoder_pin, const uint pwm_pin, const float scale,
					button_t *const man_plus, button_t *const man_minus, 
					bool *const enable, bool *const error, char (*const message)[21]);

/**
 * @brief Main servo computation function, called in 1ms loop
 * @param servo Servo controller handle
 */
void servo_compute(servo_t* const servo);

/**
 * @brief Commands servo movement with start delay
 * @param servo Servo controller handle
 * @param position Target position
 * @param speed Movement speed
 * @param delay Start delay in milliseconds
 */
void servo_goto_delayed(servo_t* const servo, const float position, const float speed, const uint32_t delay);

/**
 * @brief Commands immediate servo movement
 * @param servo Servo controller handle
 * @param position Target position
 * @param speed Movement speed
 */
void servo_goto(servo_t* const servo, const float position, const float speed);

/**
 * @brief Handles manual jog control
 * @param servo Servo controller handle
 * @param min Minimum position limit
 * @param max Maximum position limit
 * @param homed True if home position is set
 */
void servo_manual_handling(servo_t* const servo, const float min, const float max, const float speed, const bool homed);

/**
 * @brief Commands servo to stop with controlled deceleration
 * @param servo Servo controller handle
 */
void servo_stop_positioning(servo_t* const servo);

/**
 * @brief Gets the current position of the servo
 * @param servo Servo controller handle
 * @return Current position in user units
 */
float servo_get_position(const servo_t* const servo);

/**
 * @brief Gets a pointer to the servo's position variable
 * @param servo Servo controller handle
 * @return Pointer to position value in user units
 */
float* servo_get_position_pointer(servo_t* const servo);

/**
 * @brief Checks if servo is in idle state
 * @param servo Servo controller handle
 * @return true if servo is idle, false otherwise
 */
bool servo_is_idle(const servo_t* const servo);

/**
 * @brief Checks if servo is in acceleration phase
 * @param servo Servo controller handle
 * @return true if servo is accelerating, false otherwise
 */
bool servo_is_accelerating(const servo_t* const servo);

/**
 * @brief Checks if target position has been reached
 * @param servo Servo controller handle
 * @return true if position reached, false otherwise
 */
bool servo_is_position_reached(const servo_t* const servo);

bool servo_is_speed_reached(const servo_t* const servo);

/**
 * @brief Sets the target stop position without starting movement
 * @param servo Servo controller handle
 * @param position Target position in user units
 */
void servo_set_stop_position(servo_t* const servo, const float position);

/**
 * @brief Sets current position as zero reference
 * @param servo Servo controller handle
 * 
 * Resets internal position counters and sets current position as new zero point
 */
void servo_set_zero_position(servo_t* const servo);

#endif
 