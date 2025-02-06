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
#define BELLOW_AVG_MIN 100

/**
 * @brief Threshold value to detect void/gap in material
 */
#define VOID_REFLECTIVITY_THRESHOLD 120

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
} detector_t;

extern detector_t detector;

#ifdef  __cplusplus
extern "C" {
#endif

    /**
     * @brief Creates and initializes a detector instance
     * @param sensor_pin ADC pin number (26-28) for the reflectivity sensor
     * @param feeder_position Pointer to the current feeder position value
     */
    void init_detector(uint8_t sensor_pin, float *feeder_position, bool *detector_error, char (*error_message)[21]);

    /**
     * @brief Main processing function for the detector
     * Handles state machine logic, sensor reading, and detection algorithms
     * @param detector The detector instance to process
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
    bool mark_detection();

    /**
     * @brief Handles the idle state of the detector
     * @param detector The detector instance
     * Resets all detector states and prepares for new detection cycle
     */
    void detector_idle_state(detector_t detector);

    /**
     * @brief Processes sensor readings for mark detection
     * @param detector The detector instance
     * Analyzes sensor data to identify registration marks and updates position data
     */
    void detector_line_detection(detector_t detector);

    /**
     * @brief Processes sensor readings for edge detection
     * @param detector The detector instance
     * Analyzes sensor data to identify material edges and updates position data
     */
    void detector_edge_detection(detector_t detector);

    /**
     * @brief Handles error conditions in the detector
     * @param detector The detector instance
     * Implements error recovery procedures and state management
     */
    void detector_failure_state(detector_t detector);

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
    bool find_range(uint16_t base_value);

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
    
#ifdef  __cplusplus
}
#endif

#endif
// End of Header file