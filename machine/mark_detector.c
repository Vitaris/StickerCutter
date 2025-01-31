#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "mark_detector.h"

detector_t create_detector(uint8_t sensor_pin, float *feeder_position) {
    // Create detector structure directly
    detector_t detector;

    // Set gpio pin as ADC
    // Available pins:    26, 27, 28, 29 (29 is cpu temperature)
    // Inputs:           0,  1,  2,  3
    adc_init();
    adc_gpio_init(sensor_pin + 26);
    adc_select_input(sensor_pin);

    // Initialize array
    memset(detector.stops, 0, sizeof(detector.stops));
    detector.samples = 0;
    detector.sampling_done = false;

    detector.sensor_pin = sensor_pin;
    detector.shift_size = sizeof(uint16_t) * (MEM_SIZE - 1);
    detector.float_shift_size = sizeof(float) * (MEM_SIZE - 1);
    detector.feeder_position = feeder_position;

    detector.diff_old = 0;

    detector.state = DETECTOR_IDLE;
    detector.edge_detection = EDGE_IDLE;
    detector.detecting_request = false;

    // Calibration
    detector.calibrated = false;
    detector.calibration_sum = 0;
    detector.calibration_samples = 0;
    detector.calibration_min = 0;
    detector.calibration_max = 0;
    
    return detector;
}

void detector_compute(detector_t detector)
{
    // Get new value of reflectivity
    detector.current_reflectivity = adc_read();

    // Shift sensor readings using memmove
    memmove(&detector.memory[1], &detector.memory[0], (MEM_SIZE - 1) * sizeof(uint16_t));
    detector.memory[0] = detector.current_reflectivity;

    // Shift position readings using memmove
    memmove(&detector.positions[1], &detector.positions[0], (MEM_SIZE - 1) * sizeof(float));
    detector.positions[0] = *detector.feeder_position;

    // Handle main state machine
    switch(detector.state) {
        case DETECTOR_IDLE:             detector_idle_state(detector); break;
        case DETECTOR_LINE_DETECTION:   detector_line_detection(detector); break;
        case DETECTOR_EDGE_DETECTION:   detector_edge_detection(detector); break;
        case DETECTOR_ERROR:            detector_failure_state(detector); break;
    }
}

void detector_idle_state(detector_t detector) {
    // Substates default values
    detector.line_detecting_state = LINE_IDLE;
    detector.edge_detection = EDGE_IDLE;

    // Detector default values
    detector.sampling_done = false;
    detector.line_found = false;
    detector.samples = 0;

}

void detector_line_detection(detector_t detector) {
    // State machine
    switch(detector.line_detecting_state) {
        case LINE_IDLE:
            break;
            
        case LINE_ACTIVATED:
            // Initial fill of samples and positions arrays
            if (!detector.sampling_done) {
                detector.samples++;
                if (detector.samples >= MEM_SIZE) {
                    detector.initial_average = calculate_average(detector.memory, MEM_SIZE, 0);

                    // Detect no paper or something similar
                    if (abs(detector.initial_average - 2000) > 500 ) {
                        detector.state = DETECTOR_ERROR;
                        detector.detector_error = ACTIVATION_ERROR;
                    }
                    detector.sampling_done = true;
                    detector.state = LINE_SCANNING;
                }
            }
            break;

        case LINE_SCANNING:
            // Calculate "base" value for every evaluation
            detector.average = calculate_average(detector.memory, MEM_SIZE, detector.initial_average);

            // Find range of valid data to search for the local minimum
            int16_t a, b = 0;
            bool error;
            int8_t error_code;
            int16_t index_of_minimum = 0;
            bool range_found = find_range(detector.memory, MEM_SIZE, detector.average, &a, &b, &error, &error_code);
            if (range_found) {
                bool minimum_found = find_minimum_at_range(detector.memory, MEM_SIZE, &index_of_minimum, &a, &b, &error, &error_code);
                if (minimum_found) {
                    detector.stops[0] = detector.positions[index_of_minimum];
                    detector.state = LINE_FOUND;
                }
            }
            break;

        case LINE_FOUND:
            detector.line_found = false;
            detector.state = LINE_WAITING;
            break;

        case LINE_WAITING:
            break;

        case LINE_ERROR:
            break;
    }
}

void detector_edge_detection(detector_t detector) {
    // Edge detector state machine
    switch(detector.edge_detection) {
        case EDGE_IDLE:
            detector.edge_found = false;
            detector.edge_position = 0.0;
            break;
        case EDGE_ACTIVATED:
            if (detector.current_reflectivity < VOID_REFLECTIVITY_THRESHOLD) {
                detector.edge_detection = EDGE_ERROR;
            }
            break;
        case EDGE_SCANNING:
            if (detector.current_reflectivity < VOID_REFLECTIVITY_THRESHOLD) {
                detector.edge_position = *detector.feeder_position;
                detector.edge_detection = EDGE_FOUND;
            }
            break;
        case EDGE_FOUND:
            break;
        case EDGE_RETURN_TO_ZERO:
            break;
        case EDGE_ERROR:
            detector.edge_detection = EDGE_IDLE;
            break;
    }
}

void detector_failure_state(detector_t detector) {}

bool get_paper_presence(detector_t detector) {
    return detector.current_reflectivity > VOID_REFLECTIVITY_THRESHOLD;
}

bool get_paper_absence(detector_t detector) {
    return detector.current_reflectivity < VOID_REFLECTIVITY_THRESHOLD;
}

uint16_t calculate_average(uint16_t data_array[], uint16_t array_length, uint16_t initial_average) {
    int i;
    uint16_t no_of_elements = 0;
    uint32_t sum = 0;
    for (i = 0; i < array_length; i++) {

        // If inital average is 0 means that take any value to the average (usefull for the first computation)
        if (initial_average > 0) {
            if (data_array[i] < initial_average - BELLOW_AVG_MIN) {
                continue;
            }
        }
        sum += data_array[i];
        no_of_elements++;
    }

    return sum / no_of_elements;
}

bool find_range(uint16_t data_array[], uint16_t array_length, uint16_t base_value,
                int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code) {
    int16_t tolerance_line = base_value - BELLOW_AVG_MIN;
    bool a_point_found = false;
    bool b_point_found = false;
    *error = false;
    *error_code = 0;

    // check if range is not at the begging or end of array, it could produce a false mininum
    if ((data_array[0] < tolerance_line) || (data_array[array_length - 1] < tolerance_line)){
        return false;
    }

    // find a point A when value went out from tolerance area
    // and point B when value returns back to tolerance area 
    for (int i = 1; i < array_length; i++) {
        if (data_array[i - 1] > tolerance_line &&  data_array[i] <= tolerance_line) {
            *point_a = i;
            // there should be only one point A and point B
            if (a_point_found) { 
                *error = true;
                *error_code = 1;
                return false;
            }
            a_point_found = true;
        }
        if (data_array[i - 1] < tolerance_line &&  data_array[i] >= tolerance_line) {
            *point_b = i;
            // there should be only one point A and point B
            if (b_point_found) { 
                *error = true;
                *error_code = 2;
                return false;
            }
            b_point_found = true;
        }

        // Point A and B should always be found, if not, raise the error.
        if (a_point_found && b_point_found) {
            return true;
        } else if (a_point_found && !b_point_found) {
            *error = true;
            *error_code = 3;
        } else if (!a_point_found && b_point_found)
        {
            *error = true;
            *error_code = 4;
        }
    }
}

bool find_minimum_at_range(uint16_t data_array[], uint16_t array_length, uint16_t *index_of_minimum,
                int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code) {
    *error = false;
    *error_code = 0;
    int16_t minimum = 10000;
    *index_of_minimum = 0;

    // Check that point A and B are within the data range.
    if (*point_a < 0) {
        *error = true;
        *error_code = 1;
        return false;
    }

    if (*point_b > array_length) {
        *error = true;
        *error_code = 2;
        return false;
    }

    if (*point_a > *point_b) {
        *error = true;
        *error_code = 3;
        return false;
    }

    // Find the minimum value in a given range of data
    for (int16_t i = *point_a; i < *point_b; i++) {
        if (data_array[i] < minimum) {
            minimum = data_array[i];
            *index_of_minimum = i; 
        }
    }
    return true;
}

uint16_t adc_read_simulation(uint16_t data[], uint16_t *sample, uint8_t size) {
    if (*sample >= size) {
        *sample = 0;
    }
    return data[(*sample)++];
}


float get_next_stop(detector_t detector, float current_pos) {
    // If there are no stops in memory, send feeder somewhere far away to find some
    if (detector.stops[0] == 0.0) {
        return current_pos + 1000.0;
    }
    else
    {
        return detector.stops[0];
    }
}