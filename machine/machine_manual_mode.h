#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "machine_controller.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

/**
 * @brief Defines the substates for manual operation mode
 */
typedef enum {
    MANUAL_IDLE,      // Motors disabled, waiting for enable command
    MANUAL_READY,     // Motors enabled, ready for operations
} manual_substate_t;

/**
 * @brief Defines the substates for homing operation sequence
 */
typedef enum {
    HOMING_IDLE,              // Initial state, waiting for start command
    HOMING_START,             // Preparing to start homing sequence
    HOMING_SCANNING,          // Moving servo while scanning for home position
    HOMING_FOUND,            // Home position detected, stopping motion
    HOMING_RETURN_TO_ZERO,   // Moving back to define zero position
    HOMING_FINISHED          // Homing sequence completed
} homing_substate_t;

/**
 * @brief Activates manual operation mode
 * @details Sets the machine state to MANUAL, enables motors and initializes manual substate
 */
void activate_manual_state();

/**
 * @brief Activates homing operation mode
 * @details Sets the machine state to HOMING and initializes homing substate
 */
void activate_homing_state();

/**
 * @brief Handles manual operation state of the machine
 * @details Manages motor control, display updates, and state transitions in manual mode
 * Responds to function buttons and sensor inputs
 */
void handle_manual_state();

/**
 * @brief Handles the homing sequence of the machine
 * @details Executes the homing sequence step by step:
 * 1. Waits for start command
 * 2. Moves servo to find home position
 * 3. Detects home position
 * 4. Sets zero position
 * 5. Returns to manual mode when complete
 */
void handle_homing_state();
