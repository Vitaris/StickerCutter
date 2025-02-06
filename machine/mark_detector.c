#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "mark_detector.h"

detector_t detector;

void init_detector(uint8_t sensor_pin, float *feeder_position, bool *detector_error, char (*error_message)[21]) {
    // Set gpio pin as ADC
    // Available pins:    26, 27, 28, 29 (29 is cpu temperature)
    // Inputs:           0,  1,  2,  3
    adc_init();
    adc_gpio_init(sensor_pin + 26);
    adc_select_input(sensor_pin);

    // Initialize array
    detector.samples = 0;
    detector.sampling_done = false;

    detector.sensor_pin = sensor_pin;
    detector.feeder_position = feeder_position;

    // Error handling
    detector.error = detector_error;
	detector.error_message = error_message;
}

void detector_compute()
{
    // Get new value of reflectivity
    detector.current_reflectivity = adc_read();

    // Shift sensor readings using memmove
    memmove(&detector.memory[1], &detector.memory[0], (MEM_SIZE - 1) * sizeof(uint16_t));
    detector.memory[0] = detector.current_reflectivity;

    // Shift position readings using memmove
    memmove(&detector.positions[1], &detector.positions[0], (MEM_SIZE - 1) * sizeof(float));
    detector.positions[0] = *detector.feeder_position;

    if (detector.sampling_done) {
        detector.average = calculate_average(detector.initial_average);
    } 
    else {
        detector.samples++;
        if (detector.samples >= MEM_SIZE) {
            detector.initial_average = calculate_average(0);
            detector.sampling_done = true;
        }
    }
}

void detector_restart() {
    detector.samples = 0;
    detector.sampling_done = false;
}

bool mark_detection() {
    // Find range of valid data to search for the local minimum
    detector.start_of_spike = 0;
    detector.end_of_spike = 0; 
    int16_t index_of_minimum = 0;
    bool range_found = find_range(detector.average);
    if (range_found) {
        bool minimum_found = find_minimum_at_range(&index_of_minimum);
        if (minimum_found) {
            detector.mark_position = detector.positions[index_of_minimum];
            return true;
        }
    }
    return false;
}

bool get_void_presence(detector_t detector) {
    return detector.current_reflectivity < VOID_REFLECTIVITY_THRESHOLD;
}

bool get_void_absence(detector_t detector) {
    return detector.current_reflectivity > VOID_REFLECTIVITY_THRESHOLD;
}

uint16_t calculate_average(uint16_t initial_average) {
    uint32_t sum = 0;
    uint16_t no_of_elements = 0;
    for (int i = 0; i < MEM_SIZE; i++) {

        // If inital average is 0 means that take any value to the average (useful for the first computation)
        if (initial_average > 0) {
            if (detector.memory[i] < initial_average - BELLOW_AVG_MIN) {
                continue;
            }
        }
        sum += detector.memory[i];
        no_of_elements++; // It's needed to know how many elements were used for the average calculation
    }
    return sum / no_of_elements;
}

bool find_range(uint16_t base_value) {
    int16_t tolerance_line = base_value - BELLOW_AVG_MIN;
    // check if range is not at the beginning or end of array, it could produce a false mininum
    if ((detector.memory[0] < tolerance_line) || (detector.memory[MEM_SIZE - 1] < tolerance_line)){
        return false;
    }

    // find a point A when value went out from tolerance area
    // and point B when value returns back to tolerance area 
    for (int i = 1; i < MEM_SIZE; i++) {
        if (detector.memory[i - 1] > tolerance_line && detector.memory[i] <= tolerance_line) {
            if (detector.start_of_spike == 0) {
                detector.start_of_spike = i;
            }
            else {
                *detector.error = true;
                strcpy(*detector.error_message, "DC1");
                return false;
            }
        }
        if (detector.memory[i - 1] < tolerance_line && detector.memory[i] >= tolerance_line) {
            if (detector.end_of_spike == 0) {
                detector.end_of_spike = i;
            }
            else {
                *detector.error = true;
                strcpy(*detector.error_message, "DC2");
                return false;
            }
        }
    }

    // Point A and B should always be found, if not, raise the error.
    if (detector.start_of_spike != 0 && detector.end_of_spike != 0) {
        return true;
    } 
    else if (detector.start_of_spike == 0 && detector.end_of_spike != 0) {
        *detector.error = true;
        strcpy(*detector.error_message, "DC3");
    } 
    else if (detector.start_of_spike != 0 && detector.end_of_spike == 0) {
        *detector.error = true;
        strcpy(*detector.error_message, "DC4");
    }
    return false;
}

bool find_minimum_at_range(uint16_t *index_of_minimum) {
    int16_t minimum = 10000;
    *index_of_minimum = 0;

    // Check that point A and B are within the data range.
    if (detector.start_of_spike < 0) {
        strcpy(*detector.error_message, "DC5");
        return false;
    }

    if (detector.end_of_spike > MEM_SIZE) {
        strcpy(*detector.error_message, "DC6");
        return false;
    }

    if (detector.start_of_spike > detector.end_of_spike) {
        strcpy(*detector.error_message, "DC7");
        return false;
    }

    // Find the minimum value in a given range of data
    for (int16_t i = detector.start_of_spike; i < detector.end_of_spike; i++) {
        if (detector.memory[i] < minimum) {
            minimum = detector.memory[i];
            *index_of_minimum = i;
        }
    }
    return true;
}
