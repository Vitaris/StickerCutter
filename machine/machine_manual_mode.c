#include <stdio.h>
#include "pico/stdlib.h"

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

typedef enum {
    MANUAL_IDLE,      // Motors disabled, waiting for enable command
    MANUAL_READY,     // Motors enabled, ready for operations
} manual_substate_t;

typedef enum {
    HOMING_START,             // Preparing to start homing sequence
    HOMING_SCANNING,          // Moving servo while scanning for home position
    HOMING_FOUND,            // Home position detected, stopping motion
    HOMING_RETURN_TO_ZERO,   // Moving back to define zero position
    HOMING_FINISHED          // Homing sequence completed
} homing_substate_t;

manual_substate_t manual_substate;
homing_substate_t homing_substate;

void activate_homing_state(void) {
    manual_substate = HOMING_START;
    machine.state = HOMING;
}

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
                if (get_void_absence()) {
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
            else {
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
        case HOMING_START:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 2000.0, 100.0, HALF_SECOND_DELAY);
                homing_substate = HOMING_SCANNING;
            }
            break;
        
        case HOMING_SCANNING:
            set_text_10(display.F2_text, "Hlada sa->");
            if (get_void_presence()) {
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
