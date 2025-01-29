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

void handle_manual_state() {
    // Update display
    set_text_20(display.state_text, machine.homed ? "Manual" : "Manual - NO Home");

    // Handle state transitions
    switch(manual_substate) {
        case MANUAL_IDLE:
            set_text_10(display.F1_text, "Mot->ON");
            set_text_10(display.F2_text, "Home");
            
            if (machine.F1->state_raised) {
                machine.enable = true;
                manual_substate = MANUAL_READY;
            }
            break;

        case MANUAL_READY:
            set_text_10(display.F1_text, "Mot->OFF");
            
            if (machine.F1->state_raised) {
                machine.enable = false;
                manual_substate = MANUAL_IDLE;
                reset_params();
                break;
            }

            // Handle paper edge and mark detection
            if (machine.paper_edge_position == 0.0) {
                set_text_10(display.F2_text, "Paper");
                if (machine.F2->state_raised) {
                    machine.paper_edge_position = machine.servo_0->servo_position;
                }
            }
            else if (machine.mark_position == 0.0) {
                set_text_10(display.F2_text, "Mark");
                if (machine.F2->state_raised) {
                    machine.mark_position = machine.servo_0->servo_position;
                }
            }
            else {
                set_text_10(display.F2_text, "Start");
                if (machine.F2->state_raised) {
                    stop_positioning(machine.servo_0);
                    stop_positioning(machine.servo_1);
                    knife_up();
                    machine.state = AUTOMAT;
                }
            }

            servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
            servo_manual_handling(machine.servo_1, 0, 0, false);

            break;

        case MANUAL_HOMING:
            set_text_10(display.F1_text, "Mot->OFF");
            handle_homing_sequence();
            
            if (machine.F1->state_raised) {
                machine.enable = false;
                manual_substate = MANUAL_IDLE;
                reset_params();
            }
            else if (machine.homed) {
                manual_substate = MANUAL_READY;
            }
            break;
    }

    // machine.enable/disable motors based on current state
    if (!machine.enable) {
        set_text_20(display.state_text, "Manual, Volne motory");
        reset_params();
        manual_substate = MANUAL_IDLE;
    }
}

void handle_homing_sequence(void) {
    switch(machine.detector.edge_detection) {
        case EDGE_IDLE:
            set_text_10(display.F2_text, "     Home");
            if (machine.F2->state_raised) {
                machine.detector.edge_detection = EDGE_ACTIVATED;
            }
            break;

        case EDGE_ACTIVATED:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 2000.0, 100.0, 500);
                machine.detector.edge_detection = EDGE_SCANNING;
            }
            break;

        case EDGE_SCANNING:
            set_text_10(display.F2_text, "Hlada sa->"); 
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
            }
            break;

        case EDGE_ERROR:
            set_text_10(display.F2_text, "Error!");
            break;
    }
}
