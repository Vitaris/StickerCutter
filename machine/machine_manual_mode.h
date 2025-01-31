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
} manual_substate_t;

typedef enum {
    HOMEING_IDLE,      
    HOMEING_READY,     
    HOMEING_FINISHED
} homeing_substate_t;

void activate_manual_state();

void activate_homing_state();

/**
 * @brief Handles manual operation state of the machine
 * @param machine Pointer to machine controller
 */
void handle_manual_state();

void handle_homeing_state();

/**
 * @brief Handles the homing sequence
 */
void handle_homing_sequence(void);