#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "machine_controller.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

/**
 * @brief Structure for monitoring and storing mark positions and sticker dimensions
 * Used during the initialization phase to learn sticker dimensions and later
 * for validating proper mark detection during operation
 */
typedef struct {
    bool sticker_dimensions_set;          // Indicates if sticker dimensions are learned
    
    float sticker_height;                 // Height of a single sticker
    float mark_distance;                  // Distance between stickers in a row
    
    float first_mark_position;            // Position of first detected mark
    float second_mark_position;           // Position of second detected mark
    float third_mark_position;            // Position of third detected mark
} marks_monitor_t;

/**
 * @brief States for the automatic cutting mode state machine
 * Controls the sequence of operations from mark detection to cutting
 */
typedef enum {
    AUTOMATIC_IDLE,                       // Waiting for start command
    AUTOMATIC_MARK_GOTO,                 // Moving knife to initial mark position
    AUTOMATIC_MARK_GOTO_IN_PROGRESS,     // Waiting for knife to reach mark position
    AUTOMATIC_MARK_POS_REACHED,          // Knife has reached mark position
    AUTOMATIC_START_ROLLING_OUT_PAPER,   // Begin feeding paper
    AUTOMATIC_WAIT_FOR_CONSTANT_SPEED_OF_PAPER, // Waiting for stable paper feed
    AUTOMATIC_WAITING_FOR_SAMPLES,       // Collecting sensor readings
    AUTOMATIC_SCANNING_FOR_MARK,         // Actively looking for marks
    AUTOMATIC_MARK_FOUND,               // Mark detected
    AUTOMATIC_SAVE_FIRST_MARK,          // Recording position of first mark
    AUTOMATIC_SAVE_SECOND_MARK,         // Recording position of second mark
    AUTOMATIC_SAVE_THIRD_MARK,          // Recording position of third mark
    AUTOMATIC_REGULAR_CUT_STOP,         // Stop centered between two marks
    AUTOMATIC_GOTO_CUT_POSITION,        // Moving to cutting position
    AUTOMATIC_WAIT_FOR_CUT_POSITION,    // Waiting to reach cutting position
    AUTOMATIC_CUT_OPENING_SECTION,      // Starting the cut sequence
    AUTOMATIC_CUT_OPENING_SECTION2,     // Continuing cut sequence
    AUTOMATIC_CUT_RETURN_TO_MARK,       // Moving back to mark after first cut
    AUTOMATIC_CUT_REST_SECTION,         // Cutting remaining section
    AUTOMATIC_CUT_RETURN_TO_MARK_2,     // Final return to mark position
    AUTOMATIC_RETURN_NEW_DETECTION,     // Preparing for next mark detection
    AUTOMATIC_RETURN_TO_ZERO,           // Returning to home position
    AUTOMATIC_FINISHED                  // Cycle complete
} automatic_substate_t;

/**
 * @brief Initializes the automatic mode state machine
 * Resets all monitoring data and sets initial state
 */
void activate_automatic_state();

/**
 * @brief Main processing function for automatic mode
 * Handles state transitions and executes appropriate actions for each state
 */
void handle_automatic_state();
