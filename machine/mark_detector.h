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

/**
 * @brief States for the main detector state machine
 * Controls the process of finding and tracking registration marks
 */
typedef enum {
    LINE_IDLE,          // Edge detection inactive
    LINE_ACTIVATED,     // Initial state when detector is activated
    LINE_SCANNING,      // Actively scanning for marks
    LINE_FOUND,    // Successfully found a registration mark
    LINE_WAITING,       // Waiting for next operation
    LINE_APPEND_STOP,   // Adding a stop position to memory
    LINE_MARK_NOT_FOUND,// Failed to find registration mark
    LINE_ERROR,         // Error state
} line_detecting_state_t;

/**
 * @brief States for the edge detection state machine
 * Controls the process of finding material edges
 */
typedef enum  {
    EDGE_IDLE,             // Edge detection inactive
    EDGE_ACTIVATED,        // Edge detection activated
    EDGE_SCANNING,         // Scanning for edge
    EDGE_FOUND,           // Edge successfully detected
    EDGE_RETURN_TO_ZERO,  // Returning to initial position
    EDGE_ERROR            // Error in edge detection
} edge_detection_t;

typedef enum {
    DETECTOR_IDLE,
    DETECTOR_LINE_DETECTION,
    DETECTOR_EDGE_DETECTION,
    DETECTOR_ERROR
} detector_state_t;

/**
 * @brief Error codes for detector operation
 */
enum detector_error{
    ACTIVATION_ERROR      // Error during detector activation
};

/**
 * @brief Main detector structure containing all operational data
 */
typedef struct {
    // State management
    detector_state_t state;              // Current state of the detector
    line_detecting_state_t line_detecting_state;     // Current state of mark detector
    edge_detection_t edge_detection;     // Current state of edge detection
    enum detector_error detector_error;  // Current error status
    bool detecting_request;              // Flag for detection request
    bool line_found;                     // Flag indicating mark detection
    bool edge_found;                     // Flag indicating edge detection
    
    // Hardware configuration
    uint8_t sensor_pin;                  // GPIO ADC Pin (26, 27, 28)
    
    // Sensor readings and processing
    uint16_t current_reflectivity;       // Latest sensor reading
    uint16_t average;                    // Moving average of readings
    uint16_t initial_average;            // Initial baseline average
    uint16_t samples;                    // Current sample count
    uint16_t average_samples;            // Samples used for averaging
    bool sampling_done;                  // Sampling completion flag
    
    // Calibration data
    bool calibrated;                     // Calibration status
    uint32_t calibration_sum;           // Sum for calibration calculations
    uint16_t calibration_samples;       // Number of calibration samples
    uint16_t calibration_min;           // Minimum calibration value
    uint16_t calibration_max;           // Maximum calibration value

    // Data storage
    uint16_t memory[MEM_SIZE];          // Raw sensor reading history
    uint16_t average_memory[MEM_SIZE];   // Averaged reading history
    uint16_t occupancy;                  // Memory utilization
    int16_t diff;                        // Current reading difference
    int16_t diff_old;                    // Previous reading difference
    size_t shift_size;                   // Memory shift size for uint16
    size_t float_shift_size;             // Memory shift size for float

    // Position tracking
    float positions[MEM_SIZE];           // Position history
    float stops[STOP_MEMORY_LENGHT];     // Stop position memory
    float edge_position;                 // Detected edge position
    float *feeder_position;              // Current feeder position pointer
} detector_t;

#ifdef  __cplusplus
extern "C" {
#endif

    /**
     * @brief Creates a detector instance and initializes its parameters
     * @param detector The detector to create
     * @param sensor_pin The ADC pin number (26-28) for the reflectivity sensor
     * @param feeder_position Pointer to the current feeder position
     * @return Initialized detector instance
     */
    detector_t create_detector(uint8_t sensor_pin, float *feeder_position);

    /**
     * @brief Main processing function for the detector
     * Handles state machine logic, sensor reading, and detection algorithms
     * @param detector The detector instance to process
     */
    void detector_compute(detector_t detector);

    void detector_idle_state(detector_t detector);

    void detector_line_detection(detector_t detector);

    void detector_edge_detection(detector_t detector);

    void detector_failure_state(detector_t detector);

    /**
     * @brief Calculates the average value from an array of sensor readings
     * @param data_array Array of sensor readings
     * @param array_length Length of the data array
     * @param initial_average Initial average value for threshold comparison
     * @return Calculated average value
     */
    uint16_t calculate_average(uint16_t data_array[], uint16_t array_length, uint16_t initial_average);

    /**
     * @brief Simulates ADC readings for testing purposes
     * @param data Array of simulated sensor values
     * @param sample Current sample index
     * @param size Size of the data array
     * @return Simulated sensor reading
     */
    uint16_t adc_read_simulation(uint16_t data[], uint16_t *sample, uint8_t size);

    /**
     * @brief Determines the next stop position for the feeder
     * @param detector The detector instance
     * @param current_pos Current position of the feeder
     * @return Next target position for the feeder
     */
    float get_next_stop(detector_t detector, float current_pos);

    /**
     * @brief Finds the range where sensor values drop below average
     * @param data_array Array of sensor readings
     * @param array_length Length of the data array
     * @param base_value Base value for comparison
     * @param point_a Output parameter for start of range
     * @param point_b Output parameter for end of range
     * @param error Output parameter indicating error state
     * @param error_code Output parameter with specific error code
     * @return true if valid range found, false otherwise
     */
    bool find_range(uint16_t data_array[], uint16_t array_length, uint16_t base_value,
                    int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code);

    /**
     * @brief Finds the minimum value within a specified range
     * @param data_array Array of sensor readings
     * @param array_length Length of the data array
     * @param index_of_minimum Output parameter for index of minimum value
     * @param point_a Start of range to search
     * @param point_b End of range to search
     * @param error Output parameter indicating error state
     * @param error_code Output parameter with specific error code
     * @return true if minimum found, false if error occurred
     */
    bool find_minimum_at_range(uint16_t data_array[], uint16_t array_length, uint16_t *index_of_minimum,
                int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code);

#ifdef  __cplusplus
}
#endif

#endif
// End of Header file