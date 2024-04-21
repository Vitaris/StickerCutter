#ifndef MACHINE_H
#define MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

enum machine_state{MANUAL_M, AUTOMAT};
enum machine_condition{OK, ERROR};

typedef struct machine {
	// Servo motors
	servo_t test_servo_0;
	servo_t test_servo_1;


	// Buttons
	button_t F1;
	button_t F2;
	button_t Right;
	button_t Left;
	button_t In;
	button_t Out;


	enum machine_state machine_state;
	enum machine_condition machine_condition;

	// Texts
	char state_text[10];
	char condition_text[10];
	char F1_text[10];
	char F2_text[10];

	// Mark probe
	struct detector ctrldata_detector;
	detector_t detector;
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
	machine_t create_machine(bool *F1, bool *F2, bool *servo_state_01, bool *servo_state_02);

	/**
	 * @brief      Cyclically computes the machine controller.
	 * 
	 * @param      machine  Machine controller data structure
	 */
	void machine_compute(machine_t machine, const float current_cycle_time);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file