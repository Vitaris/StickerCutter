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
#define INITIAL_MINIMUM_VALUE 0x1000  // 4096 in hex
#define MIN_SPIKE_AREA 1000    // Minimum valid area
#define MAX_SPIKE_AREA 5000    // Maximum valid area

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

typedef struct {
    uint16_t reflectivity_history[MEM_SIZE];
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
    float long_term_alpha;        // Smoothing factor (0.0 - 1.0)
    uint16_t long_term_average;   // Long term average value
} detector_t;

 detector_t detector;

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

bool is_spike_at_boundaries(uint16_t tolerance_line);

/**
 * @brief Updates the long-term average reflectivity value
 * 
 * @param new_value New reflectivity value to incorporate
 * 
 * Uses exponential moving average with alpha factor to smooth out readings
 * and maintain a baseline reference for mark detection.
 */
static void update_long_term_average(uint16_t new_value) {
    if (detector.long_term_average == 0) {
        detector.long_term_average = new_value;
    } else {
        detector.long_term_average = (uint16_t)(
            detector.long_term_alpha * new_value + 
            (1.0f - detector.long_term_alpha) * detector.long_term_average
        );
    }
}

/**
 * @brief Calculates the area of a reflectivity spike
 * 
 * @param tolerance_line The threshold value for spike detection
 * @return uint32_t The calculated area below the tolerance line
 * 
 * Area calculation helps validate if detected mark has expected characteristics
 */
static uint32_t calculate_spike_area(uint16_t tolerance_line) {
    uint32_t area = 0;

    // Sum the differences from tolerance line
    for (int i = 0; i < MEM_SIZE; i++) {
        if (detector.reflectivity_history[i] < tolerance_line) {
            area += (tolerance_line - detector.reflectivity_history[i]);
        }
    }

    return area;
}

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

    // Initialize long term average
    detector.long_term_alpha = 0.2;
    detector.long_term_average = 0;
}

void detector_compute() {
    uint16_t new_value = adc_read();

    // Shift sensor readings using memmove
    memmove(&detector.reflectivity_history[1], &detector.reflectivity_history[0], (MEM_SIZE - 1) * sizeof(uint16_t));
    detector.reflectivity_history[0] = moving_average_compute(&detector.reflectivity_filter, new_value);

    // Shift position readings using memmove
    memmove(&detector.position_history[1], &detector.position_history[0], (MEM_SIZE - 1) * sizeof(float));
    detector.position_history[0] = *detector.feeder_position;

    update_long_term_average(new_value);

    if (!detector.sampling_done) {
        detector.samples++;
        if (detector.samples >= MEM_SIZE) {
            detector.sampling_done = true;
        }
    }
}

void detector_restart() {
    detector.samples = 0;
    detector.sampling_done = false;
    detector.long_term_average = 0;
}

bool is_sampling_done(void) {
    return detector.sampling_done;
}

void find_min(uint16_t *index_of_minimum) {
    uint16_t minimum = INITIAL_MINIMUM_VALUE;    // init minimum far above the possible value 

    // Find the minimum and maximum value in a given range of data
    for (uint16_t i = 0; i < MEM_SIZE; i++) {
        if (detector.reflectivity_history[i] < minimum) {
            minimum = detector.reflectivity_history[i];
            *index_of_minimum = i;
        }
    }
}

bool detect_mark() {
    // Evaluate only the samples which have the spike in the middle of the range
    uint16_t index_of_minimum = 0;
    find_min(&index_of_minimum);

    // Took valid only if minimum is in the middle of the range
    if (index_of_minimum != MEM_SIZE/2) {
        return false;
    }

    // Verify that the spike has the minimum depth
    if (detector.reflectivity_history[index_of_minimum] > detector.long_term_average - BELLOW_AVG_MIN) {
        return false;
    }

    uint16_t tolerance_line = detector.long_term_average - BELLOW_AVG_MIN;
    // Early return if spike is at boundaries
    if (is_spike_at_boundaries(tolerance_line)) {
        return false;
    }

    // Validate spike area
    uint32_t area = calculate_spike_area(tolerance_line);
    if (area < MIN_SPIKE_AREA || area > MAX_SPIKE_AREA) {
        return false;
    }

    // Everything is valid, mark the position
    detector.mark_position = detector.position_history[index_of_minimum];
    return true;
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
