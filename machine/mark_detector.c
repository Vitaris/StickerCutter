#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "mark_detector.h"

#define MEM_SIZE 200
#define WINDOW_SIZE 10
#define AVG_SIZE 10
#define STOP_MEMORY_LENGHT 10
#define BELLOW_AVG_MIN 40
#define VOID_REFLECTIVITY_THRESHOLD 120

/**
 * @brief Represents a detector configuration structure
 * 
 * This structure holds configuration parameters and state information
 * for mark detection functionality.
 */
typedef struct {
    uint16_t buffer[WINDOW_SIZE];
    uint8_t index;
    uint32_t sum;
    bool buffer_full;
} moving_average_filter_t;

// 1. Use a detector structure to encapsulate state
typedef struct {
    uint16_t reflectivity_history[MEM_SIZE];
    uint16_t average;
    uint16_t initial_average;
    uint16_t samples;
    int16_t start_of_spike, end_of_spike;
    bool sampling_done;
    float position_history[MEM_SIZE];
    float mark_position;
    float edge_position;
    float *feeder_position;
    bool *error;
    char (*error_message)[21];
    moving_average_filter_t reflectivity_filter;
} detector_t;

// 2. Create a single static instance if needed
static detector_t detector;

/**
 * @brief Calculates the average value from sensor readings
 * @param initial_average Reference average for filtering outliers (0 for no filtering)
 * @return Calculated average value of valid readings
 */
uint16_t calculate_average(uint16_t initial_average);

/**
 * @brief Finds the range where sensor values drop below average
 * @param base_value Base value for comparison
 * @return true if valid range found, false otherwise
 */
bool find_range();

/**
 * @brief Finds the minimum value within a specified range
 * @param index_of_minimum Output parameter for index of minimum value
 * @return true if minimum found, false if error occurred
 */
bool find_minimum_at_range(uint16_t *index_of_minimum);

/**
 * @brief Initializes a moving average filter.
 * 
 * @param filter Pointer to the moving average filter structure to be initialized.
 * 
 * This function initializes all the necessary components of a moving average filter,
 * preparing it for use in signal processing. It should be called before the filter
 * is used for the first time.
 */
void init_moving_average_filter(moving_average_filter_t* filter);

/**
 * @brief Computes a moving average by adding a new value to the filter
 * 
 * @param filter Pointer to the moving average filter structure
 * @param new_value New value to be added to the moving average calculation
 * @return uint16_t The computed moving average value
 */
uint16_t moving_average_compute(moving_average_filter_t* filter, uint16_t new_value);


void init_detector(const uint8_t sensor_pin, 
                  float* const feeder_pos,
                  bool* const detector_error, 
                  char (* const error_mes)[21]) {
    // Set gpio pin as ADC
    // Available pins:    26, 27, 28, 29 (29 is cpu temperature)
    // Inputs:           0,  1,  2,  3
    adc_init();
    adc_gpio_init(sensor_pin + 26);
    adc_select_input(sensor_pin);

    // Initialize array
    detector.samples = 0;
    detector.sampling_done = false;

    detector.feeder_position = feeder_pos;

    // Error handling
    detector.error = detector_error;
	detector.error_message = error_mes;

    // Initialize moving average filter
    init_moving_average_filter(&detector.reflectivity_filter);
}

void detector_compute() {
    // Shift sensor readings using memmove
    memmove(&detector.reflectivity_history[1], &detector.reflectivity_history[0], (MEM_SIZE - 1) * sizeof(uint16_t));
    detector.reflectivity_history[0] = moving_average_compute(&detector.reflectivity_filter, adc_read());

    // Shift position readings using memmove
    memmove(&detector.position_history[1], &detector.position_history[0], (MEM_SIZE - 1) * sizeof(float));
    detector.position_history[0] = *detector.feeder_position;

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
    for (int i = 0; i < MEM_SIZE; i++) {
        sum += detector.reflectivity_history[i];
    }
    return sum / MEM_SIZE;
}

void detector_restart() {
    detector.samples = 0;
    detector.sampling_done = false;
}

bool is_sampling_done(void) {
    return detector.sampling_done;
}

bool find_minimum(uint16_t *index_of_minimum) {
    int16_t minimum = 10000;    // init minimum far above the possible value 

    // Find the minimum value in a given range of data
    for (int16_t i = 0; i < MEM_SIZE; i++) {
        if (detector.reflectivity_history[i] < minimum) {
            minimum = detector.reflectivity_history[i];
            *index_of_minimum = i;
        }
    }
}

bool detect_mark() {
    // Find range of spike in the data to search for the local minimum
    int16_t index_of_minimum = 0;
    find_minimum(&index_of_minimum);
    // Took valid only if minimum is in the middle of the range
    if (index_of_minimum != MEM_SIZE/2) {
        return false;
    }

    if (detector.reflectivity_history[index_of_minimum] > detector.initial_average - BELLOW_AVG_MIN) {
        return false;
    }

    bool range_found = find_range();
    if (range_found) {
        bool minimum_found = find_minimum_at_range(&index_of_minimum);
        if (minimum_found) {
            detector.mark_position = detector.position_history[index_of_minimum];
            return true;
        }
    }
    return false;
}

bool get_void_presence() {
    return detector.reflectivity_history[0] < VOID_REFLECTIVITY_THRESHOLD;
}

bool get_void_absence() {
    return detector.reflectivity_history[0] > VOID_REFLECTIVITY_THRESHOLD;
}

// Check if spike is at array boundaries
bool is_spike_at_boundaries(uint16_t tolerance_line) {
    return (detector.reflectivity_history[0] < tolerance_line) || 
           (detector.reflectivity_history[MEM_SIZE - 1] < tolerance_line);
}

// Find starting point of spike
bool find_spike_start(uint16_t tolerance_line) {
    for (int i = 1; i < MEM_SIZE; i++) {
        if ((detector.reflectivity_history[i - 1] > tolerance_line) &&
            (detector.reflectivity_history[i] <= tolerance_line)) {
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
bool find_spike_end(uint16_t tolerance_line) {
    for (int i = 1; i < MEM_SIZE; i++) {
        if ((detector.reflectivity_history[i - 1] < tolerance_line) && 
            (detector.reflectivity_history[i] >= tolerance_line)) {
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
bool validate_spike_points(void) {
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
bool find_range() {
    detector.start_of_spike = 0;
    detector.end_of_spike = 0;
    uint16_t tolerance_line = detector.average - BELLOW_AVG_MIN;

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
        if (detector.reflectivity_history[i] < minimum) {
            minimum = detector.reflectivity_history[i];
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

float get_mark_position(void) {
    return detector.mark_position;
}
