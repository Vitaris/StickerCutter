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
homeing_substate_t homeing_substate = HOMEING_IDLE;

void activate_manual_state() {
    manual_substate = MANUAL_READY;
    machine.state = MANUAL;
    machine.enable = true;
}

void activate_homing_state() {
    manual_substate = HOMEING_IDLE;
    machine.state = HOMEING;
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
            set_text_10(display.F2_text, "    Home");
            
            if (machine.F1->state_raised) {
                machine.enable = false;
                manual_substate = MANUAL_IDLE;
                break;
            }

            if (machine.F2->state_raised) {
                activate_homing_state();
            }

            servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
            servo_manual_handling(machine.servo_1, 0, 0, false);

            break;
    }
}

void handle_homeing_state() {
    // Update display
    set_text_20(display.state_text, "Homeing");

    set_text_10(display.F1_text, "Stop");
    if (machine.F1->state_raised) {
        activate_manual_state();
    }

    // Handle state transitions
    switch(homeing_substate) {
        case HOMEING_IDLE:
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                homeing_substate = HOMEING_READY;
            }
            break;

        case HOMEING_READY:
            handle_homing_sequence();
    
            if (machine.homed) {
                homeing_substate = HOMEING_FINISHED;
            }
            break;

        case HOMEING_FINISHED:
            activate_manual_state();
            break;
    }
}

void handle_homing_sequence(void) {
    switch(machine.detector.edge_detection) {
        case EDGE_IDLE:
            machine.detector.edge_detection = EDGE_ACTIVATED;
            break;

        case EDGE_ACTIVATED:
            if (get_void_presence(machine.detector)) {
                machine.detector.edge_detection = EDGE_ERROR;
                

            }

            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 2000.0, 100.0, 500);
                machine.detector.edge_detection = EDGE_SCANNING;
            }

            break;

        case EDGE_SCANNING:
            set_text_10(display.F2_text, "Hlada sa->");
            if (get_void_presence(machine.detector)) {
                machine.detector.edge_detection = EDGE_FOUND;
            }

            break;

        case EDGE_FOUND:
            if (machine.servo_0->positioning == ACCELERATING) {
                stop_positioning(machine.servo_0);
            }
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                machine.detector.edge_detection = EDGE_RETURN_TO_ZERO;
            }
            break;

        case EDGE_RETURN_TO_ZERO:
            if (machine.servo_0->positioning == IDLE) {
                machine.servo_0->set_zero = true;
                servo_goto_delayed(machine.servo_0, -50.0, 100.0, 500);
            }
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                machine.homed = true;
                machine.detector.edge_detection = EDGE_IDLE;
                activate_manual_state();
            }
            break;

        case EDGE_ERROR:
            set_text_10(display.F2_text, "Error!");
            break;
    }
}
