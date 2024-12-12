#ifndef MARKDETECTOR_H
#define MARKDETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "../servo_motor/servo_motor.h"

#define MEM_SIZE 200
#define AVG_SIZE 10
#define STOP_MEMORY_LENGHT 10
#define BELLOW_AVG_MIN 100
#define VOID_REFLECTIVITY_THRESHOLD 120

enum detector_state {
	DETECTOR_GET_ACTIVATED,
	DETECTOR_IDLE,
	DETECTOR_SCANNING,
	DETECTOR_LINE_FOUND,
	DETECTOR_WAITING,
	DETECTOR_APPEND_STOP,
	DETECTOR_MARK_NOT_FOUND,
	DETECTOR_ERROR,
};

enum edge_detection {
	EDGE_IDLE,
	EDGE_ACTIVATED,
	EDGE_SCANNING,
	EDGE_FOUND,
	EDGE_RETURN_TO_ZERO,
	EDGE_ERROR
};

enum detector_error{
	ACTIVATION_ERROR
};

typedef struct detector {
	enum detector_state detector_state;
	enum edge_detection edge_detection;
	enum detector_error detector_error;
	bool detecting_request;
	bool line_found;
	bool edge_found;
	uint8_t sensor_pin;						// GPIO ADC Pin, possible choice: 26, 27, 28
	uint16_t current_reflectivity;			// Current result from reflectivity sensor
	uint16_t average;						// Average of result values based on 100 samples
	uint16_t initial_average;				// Average of result values based on 100 samples
	uint16_t samples;						// Current number of samples
	uint16_t average_samples;				// Current number of average samples
	bool sampling_done;						// Sampling done flag
	
	// Calibration
	bool calibrated;
	uint32_t calibration_sum;
	uint16_t calibration_samples;
	uint16_t calibration_min;
	uint16_t calibration_max;


	uint16_t memory[MEM_SIZE];				// Memory for results
	uint16_t average_memory[MEM_SIZE];		// Memory for average result
	uint16_t occupancy;						// Occupancy of a results memory
	int16_t diff;							// Difference between current and previous result
	int16_t diff_old;						// Last difference between current and previous result
	size_t shift_size;						// Size of 499 - 1 uint16_ts
	size_t float_shift_size;				// Size of 499 - 1 floats

	float positions[MEM_SIZE];				// Positions of measured values
	float stops[STOP_MEMORY_LENGHT];		// Stops memory
	float edge_position;

	float *feeder_position;					// Current position of feeder
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
	detector_t create_detector(uint8_t sensor_pin, float *feeder_position);

	void detector_compute(detector_t machine);

	uint16_t calculate_average(uint16_t data_array[], uint16_t array_length, uint16_t initial_average);

	uint16_t adc_read_simulation(uint16_t data[], uint16_t *sample, uint8_t size);

	float get_next_stop(detector_t detector, float current_pos);
	bool find_range(uint16_t data_array[], uint16_t array_length, uint16_t base_value,
					int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code);

	bool find_minimum_at_range(uint16_t data_array[], uint16_t array_length, uint16_t *index_of_minimum,
                int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file