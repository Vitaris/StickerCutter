#ifndef MARKDETECTOR_H
#define MARKDETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MEM_SIZE 200

enum machine_state{MANUAL_M, AUTOMAT};
enum machine_condition{OK, ERROR};

struct detector {
	enum machine_state machine_state;
	enum machine_condition machine_condition;

	uint8_t sensor_pin;				// GPIO ADC Pin, possible choice: 26, 27, 28
	uint16_t result;				// Result of ADC conversion

	uint16_t memory[MEM_SIZE];		// Memory for results
	uint16_t occupancy;				// Occupancy of a results memory
	size_t shift_size;				// Size of 500 - 1 uint16_ts

	float positions[MEM_SIZE];		// Positions of measured values

	float *feeder_position;			// Current position of feeder


	// Control Buttons
	bool *F1;
	bool *F2;

	// Servos states
	bool *servo_state_01;
	bool *servo_state_02;

	char F1_text[10];
	char F2_text[10];
};

typedef struct detector * detector_t;

#ifdef	__cplusplus
extern "C" {
#endif

	/**
	 * @brief Creates a detector.
	 *
	 * @param detector        The detector
	 * @param sensor_pin      The sensor pin
	 * @param feeder_position  The feeder position
	 *
	 * @return     { description_of_the_return_value }
	 */
	detector_t create_detector(detector_t detector, uint8_t sensor_pin, float *feeder_position);

	void machine_compute(machine_t machine);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file