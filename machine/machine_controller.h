#ifndef MACHINE_H
#define MACHINE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

enum machine_state{MANUAL_M, AUTOMAT};
enum machine_condition{OK, ERROR};

struct machine {
	enum machine_state machine_state;
	enum machine_condition machine_condition;

	// Control Buttons
	bool *F1;
	bool *F2;

	// Servos states
	bool *servo_state_01;
	bool *servo_state_02;

	char F1_text[10];
	char F2_text[10];
};

typedef struct machine * machine_t;

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
	machine_t create_machine(machine_t machine, bool *F1, bool *F2, bool *servo_state_01, bool *servo_state_02);

	/**
	 * @brief      Cyclically computes the machine controller.
	 * 
	 * @param      machine  Machine controller data structure
	 */
	void machine_compute(machine_t machine);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file