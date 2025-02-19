#include <stdio.h>
#include "pico/stdlib.h"

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

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
 * @brief Defines the substates for parameter setting sequence
 */
typedef enum {
    PARAMS_IDLE,          // Initial state, waiting for parameter setting to begin
    PARAMS_PAPER_BEGIN,   // Setting the starting position of the paper
    PARAMS_PAPER_MARK,    // Setting the position of the registration mark
    PARAMS_PAPER_END,     // Setting the end position of the paper
} params_substate_t;

/**
 * @brief Current substate in manual operation mode.
 * 
 * Stores the current substate of the machine when operating in manual mode.
 * Static variable to maintain state within the file scope.
 */
static manual_substate_t manual_substate;


/**
 * @brief Current substate of the homing procedure in manual mode
 * 
 * This static variable keeps track of the current substate during
 * the machine's homing sequence when operating in manual mode.
 * The substates are defined by the homing_substate_t enumeration.
 */
static homing_substate_t homing_substate;

/**
 * @brief Current substate of the parameters handling in manual mode
 * 
 * Static variable tracking the current state within parameters processing
 * for manual machine operation mode.
 */
static params_substate_t params_substate;

/**
 * @brief Activates homing operation mode
 * @details Sets the machine state to HOMING and initializes homing substate
 */
static void activate_homing_state(void);

/**
 * @brief Activates parameter setting mode
 * @details Sets the machine state to PARAMS and initializes the parameter setting substate
 */
static void activate_params_state(void);



void activate_manual_state(void) {
    manual_substate = MANUAL_READY;
    machine.state = MANUAL;
    machine.enable = true;
}

void handle_manual_state(void) {
    // Update display
    set_text_20(display.state_text_1, machine.homed ? "Manual" : "Manual - NO Home");
    set_text_10(display.F1_text, machine.enable ? "Mot->OFF" : "Mot->ON");

    // Handle state transitions
    switch(manual_substate) {
        case MANUAL_IDLE:
            set_text_10(display.F2_text, "");
            if (machine.F1->state_raised) {
                machine.enable = true;
                manual_substate = MANUAL_READY;
            }
            break;

        case MANUAL_READY:
            if (!machine.homed) {
                if (get_void_absence(detector)) {
                    set_text_10(display.F2_text, "    Home");
                    if (machine.F2->state_raised) {
                        activate_homing_state();
                    }
                } 
                else {
                    // Do not allow homing if cutter head is out of cutting table
                    set_text_10(display.F2_text, "");
                }
            }
            else if (!machine.params_ready) {
                set_text_10(display.F2_text, " Parametre");
                if (machine.F2->state_raised) {
                    activate_params_state();
                }
            }
            else if (machine.homed && machine.params_ready) {
                set_text_10(display.F2_text, "   Automat");
                if (machine.F2->state_raised) {
                    activate_automatic_state();
                }
            }

            // Always able to switch to idle
            if (machine.F1->state_raised) {
                machine.enable = false;
                manual_substate = MANUAL_IDLE;
                break;
            }
           
            servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
            servo_manual_handling(machine.servo_1, 0, 0, false);

            break;
    }
}

void handle_homing_state(void) {
    // Update display
    set_text_20(display.state_text_1, "HOMING");

    set_text_10(display.F1_text, "Stop");
    if (machine.F1->state_raised) {
        activate_manual_state();
    }

    // Handle state transitions
    switch(homing_substate) {
        case HOMING_IDLE:
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                homing_substate = HOMING_START;
            }
            break;

        case HOMING_START:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 2000.0, 100.0, HALF_SECOND_DELAY);
                homing_substate = HOMING_SCANNING;
            }
            break;
        
        case HOMING_SCANNING:
            set_text_10(display.F2_text, "Hlada sa->");
            if (get_void_presence(detector)) {
                homing_substate = HOMING_FOUND;
            }
            break;

        case HOMING_FOUND:
            if (machine.servo_0->positioning == ACCELERATING) {
                stop_positioning(machine.servo_0);
            }
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                homing_substate = HOMING_RETURN_TO_ZERO;
            }
            break;

        case HOMING_RETURN_TO_ZERO:
            if (machine.servo_0->positioning == IDLE) {
                machine.servo_0->set_zero = true;
                servo_goto_delayed(machine.servo_0, -50.0, 100.0, HALF_SECOND_DELAY);
            }
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                homing_substate = HOMING_FINISHED;
            }
            break;

        case HOMING_FINISHED:
            machine.homed = true;
            activate_manual_state();
            break;
    }
}

void handle_params_state(void) {
    set_text_10(display.F1_text, "Stop");

    if (machine.F1->state_raised) {
        machine.state = MANUAL;
        return;
    }

    switch(params_substate) {
        case PARAMS_IDLE:
            set_text_20(display.state_text_1, "Nastavenie papiera");
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                params_substate = PARAMS_PAPER_BEGIN;
            }
            break;

        case PARAMS_PAPER_BEGIN:
            // Force value for debugging
            machine.paper_begin_position = -40.0;
            machine.paper_mark_position = -71.0;
            machine.paper_end_position = -1370.0 + machine.paper_begin_position; // Paper width 1370
            machine.params_ready = true;
            activate_manual_state();
            // delete when finished

            set_text_20(display.state_text_1, "Nastav zaciatok pap.");
            set_text_10(display.F2_text, "Zaciatok");
            if (machine.F2->state_raised) {
                machine.paper_begin_position = machine.servo_0->servo_position;
                params_substate = PARAMS_PAPER_MARK;
            }
            break;

        case PARAMS_PAPER_MARK:
            set_text_20(display.state_text_1, "Nastav znacku");
            set_text_10(display.F2_text, "Znacka");
            if (machine.F2->state_raised) {
                machine.paper_mark_position = machine.servo_0->servo_position;
                params_substate = PARAMS_PAPER_END;
            }
            break;

        case PARAMS_PAPER_END:
            set_text_20(display.state_text_1, "Nastav koniec pap.");
            set_text_10(display.F2_text, "Koniec");

            if (machine.F2->state_raised) {
                machine.paper_end_position = machine.servo_0->servo_position;
                machine.params_ready = true;
                activate_manual_state();
            }
            break;
    }

    // Handle servo control to set paper parameters
    servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
}

static void activate_homing_state(void) {
    manual_substate = HOMING_IDLE;
    machine.state = HOMING;
}

static void activate_params_state(void) {
    params_substate = PARAMS_IDLE;
    machine.state = PARAMS;
}
