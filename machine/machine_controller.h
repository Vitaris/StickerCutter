#ifndef MACHINE_H
#define MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"
#include "../rotary_encoder/rotary_encoder.h"

// Physical constants
#define SENSOR_KNIFE_OFFSET_X 25.0f
#define SENSOR_KNIFE_OFFSET_Y 14.0f
#define FAR_AWAY_DISTANCE 1000.0f
#define POSITION_EDGE_RIGHT -45.0f
#define POSITION_EDGE_LEFT -1480.0f

// Speed constants
#define MANUAL_SPEED_SLOW 20.0f
#define MANUAL_SPEED_NORMAL 100.0f
#define MANUAL_SPEED_FAST 200.0f
#define AUTOMAT_SPEED_SCAN 15.0f
#define AUTOMAT_SPEED_SLOW 50.0f
#define AUTOMAT_SPEED_MID 100.0f
#define AUTOMAT_SPEED_NORMAL 200.0f
#define AUTOMAT_SPEED_FAST 250.0f
#define AUTOMAT_SPEED_CUT 180.0f

#define HALF_SECOND_DELAY 500

typedef enum {
	MANUAL,
	HOMING,
	PARAMS,
	AUTOMAT, 
	FAILURE
} machine_state_t;

typedef enum {
	MANUAL_IDLE,
	MANUAL_READY
} manual_substate_t;

typedef enum {
	HOMING_START,
	HOMING_SCANNING,
	HOMING_FOUND,
	HOMING_RETURN_TO_ZERO,
	HOMING_FINISHED
} homing_substate_t;

typedef enum {
	PARAM_START,
	PARAM_STICKER_HEIGHT,
	PARAM_ROWS_TO_CUT,
	PARAM_PAPER_WIDTH,
	PARAM_FINSIHED
} param_substate_t;

typedef enum {
	IDLE,
	NAVIGATE_TO_LEAD_IN,
	MOVING_TO_LEAD_IN,
	LEAD_IN_READY,
	START_ENTRY_CUT,
	ENTRY_CUT_IN_PROGRESS,
	ENTRY_CUT_DONE,
	ENTRY_CUT_RETURN,
	RETURN_TO_LEAD_IN,
	PERFORM_REST_OF_CUT,
	REST_OF_CUT_IN_PROGRESS,
	REST_OF_CUT_DONE,
	HOME_RETURN,
	COMPLETE
} automatic_substate_t;

extern machine_state_t machine_state;
extern manual_substate_t manual_substate;
extern homing_substate_t homing_substate;
extern param_substate_t param_substate;
extern automatic_substate_t automatic_substate;

typedef struct {
	servo_t* servo_cutter;
	servo_t* servo_feeder;

	button_t* F1;
	button_t* F2;
	button_t* Right;
	button_t* Left;
	button_t* In;
	button_t* Out;
	button_t* home_switch;
	rotary_encoder_t* encoder;
} devices_t;

extern devices_t devices;

typedef struct {
	bool enable;
	bool homed;
	bool machine_error;
	char error_message[21];
	float feeder_offset;

	// Cutter
	bool params_ready;
	float paper_right_mark_position;
	float sticker_height;
	float paper_width;
	size_t rows_to_cut;
	size_t rows_cut_performed;
	float cut_overlap; 

	bool test;
} machine_t;

extern machine_t machine;

typedef struct {
	machine_state_t state;
	manual_substate_t manual_substate;
	homing_substate_t homing_substate;
	param_substate_t param_substate;
	automatic_substate_t automatic_substate;
	bool homed;
	bool enabled;
	bool paper_positions_set;
	bool feeder_zeroed;
	float pos_cutter;
	float pos_feeder;
	float feeder_offset;
	float encoder_value;
	int encoder_value_int;
	size_t rows_to_cut;
	size_t rows_cut_performed;
	char error_message[21];
} machine_display_snapshot_t;

/**
 * @brief Initializes the machine controller
 */
void machine_init(void);

void machine_get_display_snapshot(machine_display_snapshot_t *snapshot);

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
