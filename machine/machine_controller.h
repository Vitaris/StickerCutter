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

enum cutter_state{
	CUTTER_IDLE,
	CUTTER_REQUESTED,
	TO_HOME,
	TO_PRECUT,
	BACK_HOME,
	CUT_TO_END,
	FINAL_RETURN,
	CUT_DONE,
	STOP_CUTTING};

enum machine_state{
	MANUAL_DISABLED_MOTORS, 
	MANUAL, 
	AUTOMAT, 
	FAILURE
	};

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
	bool enable;
	bool machine_error;
	bool homed;
	enum machine_state state;
	enum machine_condition machine_condition;
	char error_message[21];

	// LCD Texts
	char state_text[21];
	char condition_text[10];
	char position_cutter[8];
	char position_feeder[8];
	char F1_text[11];
	char F2_text[11];

	// Mark probe
	detector_t detector;

	enum cutter_state cutter_state;
} * machine_t; 


#ifdef	__cplusplus
extern "C" {
#endif

	/**
	 * @brief      Creates a machine controller.
	 * 
	 * @param      machine         Machine controller data structure
	 * @param      F1              F1 button state
	 * @param      F2              F2 button state
	 * @param      servo_state_01  Servo 01 state
	 * @param      servo_state_02  Servo 02 state
	 * 
	 * @return     Machine controller data structure
	 */
	machine_t create_machine();

	/**
	 * @brief      Cyclically computes the machine controller.
	 * 
	 * @param      machine  Machine controller data structure
	 */
	void machine_compute(machine_t machine);

	void sticker_cut_compute(machine_t machine);

	void set_text(char LCD_text[], char text[], uint8_t len);

	void set_text_10(char LCD_text[], char text[]);

	void set_text_20(char LCD_text[], char text[]);

	void set_pause();

	bool is_time(float cycle_time);

	void perform_sticker_cut(machine_t machine);

	

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file