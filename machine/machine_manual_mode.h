#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mark_detector.h"
#include "machine_controller.h"
#include "../servo_motor/button.h"
#include "../servo_motor/servo_motor.h"

typedef enum {
    MANUAL_IDLE,      // Motors disabled, waiting for enable command
    MANUAL_READY,     // Motors enabled, ready for operations
    MANUAL_HOMING     // Executing homing sequence
} manual_substate_t;

/**
 * @brief Handles manual operation state of the machine
 * @param machine Pointer to machine controller
 */
void handle_manual_state();

/**
 * @brief Handles the homing sequence
 */
void handle_homing_sequence(void);