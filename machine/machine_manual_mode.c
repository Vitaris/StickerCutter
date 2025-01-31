#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

// Define and initialize the global variable
manual_substate_t manual_substate = MANUAL_IDLE;
homing_substate_t homing_substate = HOMING_IDLE;

void activate_manual_state() {
    manual_substate = MANUAL_READY;
    machine.state = MANUAL;
    machine.enable = true;
}

void activate_homing_state() {
    manual_substate = HOMING_IDLE;
    machine.state = HOMING;
}

void handle_manual_state() {
    // Update display
    set_text_20(display.state_text, machine.homed ? "Manual" : "Manual - NO Home");
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
            if (get_void_absence(machine.detector)) {
                set_text_10(display.F2_text, "    Home");
                if (machine.F1->state_raised) {
                    machine.enable = false;
                    manual_substate = MANUAL_IDLE;
                    break;
                }
            } else {
                set_text_10(display.F2_text, "");
            }

            if (machine.F2->state_raised) {
                activate_homing_state();
            }

            servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
            servo_manual_handling(machine.servo_1, 0, 0, false);

            break;
    }
}

void handle_homing_state() {
    // Update display
    set_text_20(display.state_text, "HOMING");

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
                servo_goto_delayed(machine.servo_0, 2000.0, 100.0, 500);
                homing_substate = HOMING_SCANNING;
            }
            break;
        
        case HOMING_SCANNING:
            set_text_10(display.F2_text, "Hlada sa->");
            if (get_void_presence(machine.detector)) {
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
                servo_goto_delayed(machine.servo_0, -50.0, 100.0, 500);
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
