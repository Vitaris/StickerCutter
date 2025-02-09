#ifndef MARKDETECTOR_H
#define MARKDETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "../servo_motor/servo_motor.h"

/**
 * @brief Buffer size for sensor readings and position history
 */
#define MEM_SIZE 200

/**
 * @brief Size of the moving average window
 */
#define AVG_SIZE 10

/**
 * @brief Length of memory used for stop condition detection
 */
#define STOP_MEMORY_LENGHT 10

/**
 * @brief Minimum difference from average to detect mark
 */
#define BELLOW_AVG_MIN 40

/**
 * @brief Threshold value to detect void/gap in material
 */
#define VOID_REFLECTIVITY_THRESHOLD 120

#define WINDOW_SIZE 10

typedef struct {
    uint16_t buffer[WINDOW_SIZE];
    uint8_t index;
    uint32_t sum;
    bool buffer_full;
} moving_average_filter_t;

/**
 * @brief Main detector structure containing all operational data and state information
 * Manages the complete state of the mark detection system including hardware config,
 * sensor readings, calibration data, and detection results
 */
typedef struct {
    // Hardware configuration
    uint8_t sensor_pin;                  // GPIO ADC Pin (26, 27, 28)
    
    // Sensor readings and processing
    uint16_t memory[MEM_SIZE];           // Raw sensor reading history
    uint16_t current_reflectivity;       // Latest sensor reading
    uint16_t average;                    // Moving average of readings
    uint16_t initial_average;            // Initial baseline average for calibration
    uint16_t samples;                    // Number of samples collected during initialization
    int16_t start_of_spike, end_of_spike; // Range of spike in sensor readings
    bool sampling_done;                  // Flag indicating if initial sampling is complete

    // Position tracking
    float positions[MEM_SIZE];           // History of feeder positions
    float mark_position;                 // Position where mark was detected
    float edge_position;                 // Position where edge was detected
    float *feeder_position;              // Pointer to current feeder position

    bool *error;                         // Pointer to global error bool
	char (*error_message)[21];           // Error message
    moving_average_filter_t reflectivity_filter;
} detector_t;

extern detector_t detector;

#ifdef  __cplusplus
extern "C" {
#endif

    /**
     * @brief Creates and initializes a detector instance
     * @param sensor_pin ADC pin number (26-28) for the reflectivity sensor
     * @param feeder_position Pointer to the current feeder position value
     * @param detector_error Pointer to error flag
     * @param error_message Pointer to error message array
     */
    void init_detector(uint8_t sensor_pin, float *feeder_position, bool *detector_error, char (*error_message)[21]);

    /**
     * @brief Main processing function for the detector
     * Handles sensor reading and data processing
     */
    void detector_compute();

    /**
     * @brief Restarts the detector sampling process
     * Resets sample counter and sampling completion flag to begin fresh sampling
     */
    void detector_restart();

    /**
     * @brief Processes current readings to detect registration marks
     * @return true if mark is detected, false otherwise
     */
    bool detect_mark();

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
     * @brief Checks if void is present under the sensor
     * @param detector The detector instance
     * @return true if void is detected, false otherwise
     */
    bool get_void_presence(detector_t detector);

    /**
     * @brief Checks if void is absent under the sensor
     * @param detector The detector instance
     * @return true if no void is detected, false otherwise
     */
    bool get_void_absence(detector_t detector);

    void init_moving_average_filter(moving_average_filter_t* filter);

    uint16_t moving_average_compute(moving_average_filter_t* filter, uint16_t new_value);
    
#ifdef  __cplusplus
}
#endif

#endif