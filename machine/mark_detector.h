#ifndef MARKDETECTOR_H
#define MARKDETECTOR_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Creates and initializes a detector instance
 * 
 * @param sensor_pin ADC pin number (26-28) for the reflectivity sensor
 * @param feeder_position Pointer to the current feeder position value
 * @param detector_error Pointer to error flag for error state indication
 * @param error_message Pointer to error message array for detailed error reporting
 * 
 * Initializes ADC for the sensor pin and sets up internal state for mark detection.
 * Must be called before any other detector functions.
 */
void init_detector(const uint8_t sensor_pin, 
                  float* const feeder_position,
                  bool* const detector_error, 
                  char (* const error_message)[21]);

/**
 * @brief Main processing function for the detector
 * 
 * Reads sensor data, updates moving averages and history buffers.
 * Should be called periodically at a consistent rate for optimal detection.
 */
void detector_compute(void);

/**
 * @brief Restarts the detector sampling process
 * Resets sample counter and sampling completion flag to begin fresh sampling
 */
void detector_restart(void);

/**
 * @brief Processes current readings to detect registration marks
 * @return true if mark is detected, false otherwise
 */
bool detect_mark(void);

/**
 * @brief Checks if void is present under the sensor
 * @param detector The detector instance
 * @return true if void is detected, false otherwise
 */
bool get_void_presence(void);

/**
 * @brief Checks if void is absent under the sensor
 * @param detector The detector instance
 * @return true if no void is detected, false otherwise
 */
bool get_void_absence(void);

/**
 * @brief Checks if initial sampling is complete
 * @return true if sampling is done, false otherwise
 */
bool is_sampling_done(void);

/**
 * @brief Gets the position where the last mark was detected
 * @return float Position of the last detected mark
 */
float get_mark_position(void);

#endif