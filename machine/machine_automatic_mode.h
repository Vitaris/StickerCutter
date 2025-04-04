#ifndef MACHINE_AUTOMATIC_MODE_H
#define MACHINE_AUTOMATIC_MODE_H

#include <stdbool.h>
#include "mark_detector.h"

/**
 * @brief Initializes the automatic mode state machine
 * Resets all monitoring data and sets initial state
 */
void activate_automatic_state(void);

/**
 * @brief Main processing function for automatic mode
 * Handles state transitions and executes appropriate actions for each state
 */
void handle_automatic_state(void);

/**
 * @brief Resets all stored paper mark positions to their default values.
 * 
 * This function clears any previously detected or stored paper mark positions,
 * effectively resetting the system's knowledge of mark locations on the paper.
 * This is typically called when starting a new job or when needing to
 * re-calibrate the mark detection system.
 */
void reset_paper_mark_positions(void);

/**
 * @brief Checks if the paper positions have been set in the machine
 * 
 * @return true if paper positions are set
 * @return false if paper positions are not set
 */
bool is_paper_positions_set(void);

#endif /* MACHINE_AUTOMATIC_MODE_H */
