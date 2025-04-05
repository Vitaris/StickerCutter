#ifndef MACHINE_MANUAL_MODE_H
#define MACHINE_MANUAL_MODE_H

#include <stdbool.h>
#include "mark_detector.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

/**
 * @brief Activates manual operation mode
 * @details Sets the machine state to MANUAL, enables motors and initializes manual substate
 */
void activate_manual_state(void);

/**
 * @brief Handles manual operation state of the machine
 * @details Manages motor control, display updates, and state transitions in manual mode
 * Responds to function buttons and sensor inputs
 */
void handle_manual_state(void);

/**
 * @brief Handles the homing sequence of the machine
 * @details Executes the homing sequence step by step:
 * 1. Waits for start command
 * 2. Moves servo to find home position
 * 3. Detects home position
 * 4. Sets zero position
 * 5. Returns to manual mode when complete
 */
void handle_homing_state(void);

/**
 * @brief Enables manual control of the servo motor movement.
 * 
 * This function handles the manual operation mode for servo motor control,
 * allowing direct user input to control servo position and movement.
 * It should be called periodically in manual operation mode.
 */
void servo_manual_movement(void);

void servo_manual_movement_slow(void);

#endif // MACHINE_MANUAL_MODE_H