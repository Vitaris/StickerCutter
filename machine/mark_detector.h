#ifndef MARKDETECTOR_H
#define MARKDETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>



#define MEM_SIZE 1000
#define AVG_SIZE 10

typedef struct detector {
	uint8_t sensor_pin;				// GPIO ADC Pin, possible choice: 26, 27, 28
	uint16_t result;				// Result of ADC conversion
	uint16_t average;				// Average of result values based on 100 samples
	uint16_t samples;				// Current number of samples
	uint16_t average_samples;		// Current number of average samples
	bool sampling_done;				// Sampling done flag

	uint16_t memory[MEM_SIZE];		// Memory for results
	uint16_t average_memory[MEM_SIZE];		// Memory for average result
	uint16_t occupancy;				// Occupancy of a results memory
	int16_t diff;					// Difference between current and previous result
	int16_t diff_old;				// Last difference between current and previous result
	size_t shift_size;				// Size of 500 - 1 uint16_ts

	float positions[MEM_SIZE];		// Positions of measured values

	float *feeder_position;			// Current position of feeder
} * detector_t;

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
	detector_t create_detector(uint8_t sensor_pin);

	void detector_compute(detector_t machine);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file