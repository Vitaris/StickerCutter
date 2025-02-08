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

    // Initialize moving average filter
    init_moving_average_filter(&detector.reflectivity_filter);
}

void detector_compute()
{
    // Get new value of reflectivity
    // detector.current_reflectivity = adc_read();
    detector.current_reflectivity = moving_average_compute(&detector.reflectivity_filter, adc_read());

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

void detector_restart() {
    detector.samples = 0;
    detector.sampling_done = false;
}


static bool find_minimum(uint16_t *index_of_minimum) {
    int16_t minimum = 10000;    // init minimum far above the possible value 

    // Find the minimum value in a given range of data
    for (int16_t i = 0; i < MEM_SIZE; i++) {
        if (detector.memory[i] < minimum) {
            minimum = detector.memory[i];
            *index_of_minimum = i;
        }
    }
}

bool detect_mark() {
    // Find range of spike in the data to search for the local minimum
    int16_t index_of_minimum = 0;
    find_minimum(&index_of_minimum);
    // Took valid only if minimum is in the middle of the range
    if (index_of_minimum <= 95 && index_of_minimum > 105) {
        return false;
    }
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

// Check if spike is at array boundaries
static bool is_spike_at_boundaries(uint16_t tolerance_line) {
    return (detector.memory[0] < tolerance_line) || 
           (detector.memory[MEM_SIZE - 1] < tolerance_line);
}

// Find starting point of spike
static bool find_spike_start(uint16_t tolerance_line) {
    for (int i = 1; i < MEM_SIZE; i++) {
        if ((detector.memory[i - 1] > tolerance_line) &&
            (detector.memory[i] <= tolerance_line)) {
            if (detector.start_of_spike == 0) {
                detector.start_of_spike = i;
                return true;
            }
            *detector.error = true;
            strcpy(*detector.error_message, "DC1");
            return false;
        }
    }
    return true;
}

// Find ending point of spike
static bool find_spike_end(uint16_t tolerance_line) {
    for (int i = 1; i < MEM_SIZE; i++) {
        if ((detector.memory[i - 1] < tolerance_line) && 
            (detector.memory[i] >= tolerance_line)) {
            if (detector.end_of_spike == 0) {
                detector.end_of_spike = i;
                return true;
            }
            *detector.error = true;
            strcpy(*detector.error_message, "DC2");
            return false;
        }
    }
    return true;
}

// Validate found spike points
static bool validate_spike_points(void) {
    if (detector.start_of_spike == 0 || detector.end_of_spike == 0) {
        return false;
    }

    // Check that point A and B are within the data range.
    if (detector.start_of_spike < 0) {
        // strcpy(*detector.error_message, "DC5");
        return false;
    }

    if (detector.end_of_spike > MEM_SIZE) {
        // strcpy(*detector.error_message, "DC6");
        return false;
    }

    if (detector.start_of_spike > detector.end_of_spike) {
        // strcpy(*detector.error_message, "DC7");
        return false;
    }
    return true;
}

// Main find_range function
bool find_range(uint16_t base_value) {
    detector.start_of_spike = 0;
    detector.end_of_spike = 0;
    uint16_t tolerance_line = base_value - BELLOW_AVG_MIN;

    // Early return if spike is at boundaries
    if (is_spike_at_boundaries(tolerance_line)) {
        return false;
    }

    // Find spike start and end points
    if (!find_spike_start(tolerance_line)) {
        return false;
    }

    if (!find_spike_end(tolerance_line)) {
        return false;
    }
    // Validate the found points
    return validate_spike_points();
}

bool find_minimum_at_range(uint16_t *index_of_minimum) {
    int16_t minimum = 10000;    // init minimum far above the possible value 
    *index_of_minimum = 0;

    // Find the minimum value in a given range of data
    for (int16_t i = detector.start_of_spike; i < detector.end_of_spike; i++) {
        if (detector.memory[i] < minimum) {
            minimum = detector.memory[i];
            *index_of_minimum = i;
        }
    }

    // Took valid only if minimum is in the middle of the range
    if (*index_of_minimum != MEM_SIZE/2) {
        return false;
    }

    return true;
}

void init_moving_average_filter(moving_average_filter_t* filter) {
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->index = 0;
    filter->sum = 0;
    filter->buffer_full = false;
}

uint16_t moving_average_compute(moving_average_filter_t* filter, uint16_t new_value) {
    // Subtract oldest value from sum
    filter->sum -= filter->buffer[filter->index];
    
    // Add new value
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    
    // Update index
    filter->index = (filter->index + 1) % WINDOW_SIZE;
    if (filter->index == 0) {
        filter->buffer_full = true;
    }
    
    // Calculate and return average
    uint8_t divisor = filter->buffer_full ? WINDOW_SIZE : filter->index;
    if (divisor == 0) divisor = 1;  // Prevent division by zero
    
    return (uint16_t)(filter->sum / divisor);
}
