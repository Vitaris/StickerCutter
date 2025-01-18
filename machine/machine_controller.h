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

typedef enum {
    AUTO_IDLE,
    AUTO_PAPER_DETECT,
    AUTO_MARK_DETECT,
    AUTO_CUTTING
} auto_substate_t;

enum cutter_state{
	CUTTER_IDLE,
	CUTTER_REQUESTED,
	TO_HOME,
	AT_HOME,
	TO_PRECUT,
	BACK_HOME,
	CUT_TO_END,
	FINAL_RETURN,
	CUT_DONE,
	ROLL_OUT_PAPER,
	STOP_CUTTING};

typedef enum {
	MANUAL, 
	AUTOMAT, 
	FAILURE
} machine_state_t;

enum machine_condition{
	OK,
	ERROR};

typedef struct machine {

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

	auto_substate_t auto_substate;
	bool enable;
	bool machine_error;
	bool homed;
	enum machine_condition machine_condition;
	char error_message[21];
	float paper_edge_position;
	float mark_position;

	// Mark probe
	detector_t detector;

	enum cutter_state cutter_state;
} machine_t; 

/**
 * @brief Global machine controller instance
 * Accessible from other modules that include this header
 */
extern machine_t machine;

#ifdef	__cplusplus
extern "C" {
#endif

	/**
     * @brief Initializes the machine controller
     */
	void machine_init(void);

	/**
     * @brief Main state machine computation function
     */
	void machine_compute(void);

	/**
     * @brief Handles automatic operation state
     */
	void handle_automatic_state(void);

	/**
     * @brief Handles failure state
     */
	void handle_failure_state(void);

	/**
     * @brief Handles the homing sequence
     */
	void handle_homing_sequence(void);

	/**
     * @brief Handles the cutter state transitions
     */
	void handle_cutter_state(void);

	/**
     * @brief Computes sticker cutting sequence
     */
	void sticker_cut_compute(void);

	/**
     * @brief Computes paper feeding sequence
     */
	void feeder_compute(void);

	/**
     * @brief Executes complete sticker cutting sequence
     */
	void perform_sticker_cut(void);

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

	/**
     * @brief Resets machine parameters to default values
     */
	void reset_params(void);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file