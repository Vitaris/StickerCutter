#ifndef MACHINE_AUTOMATIC_MODE_H
#define MACHINE_AUTOMATIC_MODE_H

#include <stdbool.h>
#include "machine_controller.h"
#include "mark_detector.h"

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

#endif /* MACHINE_AUTOMATIC_MODE_H */
