#ifndef MACHINE_H
#define MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

#define PRECUT_POSITION 10.0
#define CUT_LENGTH 25.0
#define KNIFE_OUTPUT_PIN 17
#define SCALE_CUTTER 20.0
#define SCALE_FEEDER 6.4
#define SENSOR_KNIFE_OFFSET_X 25.0
#define SENSOR_KNIFE_OFFSET_Y 14.0
#define FAR_AWAY_DISTANCE 1000.0
#define CUTTING_OVERLAP 10.0
#define CUTTING_OVERLAP 10.0
#define POSITION_EDGE_RIGHT -45.0
#define POSITION_EDGE_LEFT -1480.0

typedef enum {
	MANUAL,
	HOMING,
	PARAMS,
	AUTOMAT, 
	FAILURE
} machine_state_t;

typedef struct {
	// Servo motors
	servo_t servo_0;
	servo_t servo_1;

	// Buttons
	button_t F1;
	button_t F2;
	button_t Right;
	button_t Left;
	button_t In;
	button_t Out;

	// Machine status
	machine_state_t state;

	bool enable;
	bool homed;
	bool machine_error;
	char error_message[21];

	// Cutter
	bool params_ready;
	float paper_mark_position;
} machine_t; 

/**
 * @brief Global machine controller instance
 * Accessible from other modules that include this header
 */
extern machine_t machine;

/**
 * @brief Initializes the machine controller
 */
void machine_init(void);

/**
 * @brief Main state machine computation function
 */
void machine_compute(void);

/**
 * @brief Activates the failure state of the machine
 * 
 * Transitions the machine into a failure state, which indicates
 * that an error or malfunction has occurred. This state typically
 * requires operator intervention to resolve.
 * 
 * @note Once activated, the machine will remain in failure state
 *       until explicitly reset by authorized personnel.
 */
void activate_failure_state(void);

/**
 * @brief Handles failure state
 */
void handle_failure_state(void);

/**
 * @brief Raises cutting knife
 */
void knife_up(void);

/**
 * @brief Lowers cutting knife
 */
void knife_down(void);

/**
 * @brief Sets error state with message
 * @param text Error message text
 */
void raise_error(char text[]);

#endif
