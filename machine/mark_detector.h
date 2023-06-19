#ifndef MARKDETECTOR_H
#define MARKDETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>



#define MEM_SIZE 200

struct detector {
	uint8_t sensor_pin;				// GPIO ADC Pin, possible choice: 26, 27, 28
	uint16_t result;				// Result of ADC conversion

	uint16_t memory[MEM_SIZE];		// Memory for results
	uint16_t occupancy;				// Occupancy of a results memory
	size_t shift_size;				// Size of 500 - 1 uint16_ts

	float positions[MEM_SIZE];		// Positions of measured values

	float *feeder_position;			// Current position of feeder
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

	void detector_compute(detector_t machine);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file