#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

// Define and initialize the global variable

automatic_substate_t automatic_substate;

void activate_automatic_state() {
    machine.state = AUTOMAT;
    automatic_substate = AUTOMATIC_IDLE;
}

void handle_automatic_state(void) {
    set_text_20(display.state_text, "Automat");
    set_text_10(display.F1_text, "Stop");

    if (machine.F1->state_raised) {
        machine.state = MANUAL;
        machine.cutter_state = STOP_CUTTING;
        return;
    }

    // Handle mark detection states
    switch(detector.state) {
        case DETECTOR_IDLE:
            set_text_10(display.F2_text, "Hladat zn.");
            if (machine.F2->state_raised && machine.servo_1->positioning == IDLE) {
                servo_goto_delayed(machine.servo_1, machine.servo_1->enc_position + 1000.0, 15.0, 500);
                detector.state = LINE_ACTIVATED;
            }
            break;

        case LINE_SCANNING:
            set_text_10(display.F2_text, "Hlada znak");
            break;

        case LINE_FOUND:
            set_text_10(display.F2_text, "Zn Najdeny");
            machine.servo_1->next_stop = (detector.stops[0] + 14.0) / machine.servo_1->scale;
            detector.state = LINE_WAITING;
            break;

        case LINE_WAITING:
            if (machine.F2->state_raised) {
                detector.state = IDLE;
            }
            break;
    }

    
            // Handle paper edge and mark detection
            // if (machine.paper_edge_position == 0.0) {
            //     set_text_10(display.F2_text, "Paper");
            //     if (machine.F2->state_raised) {
            //         machine.paper_edge_position = machine.servo_0->servo_position;
            //     }
            // }
            // else if (machine.mark_position == 0.0) {
            //     set_text_10(display.F2_text, "Mark");
            //     if (machine.F2->state_raised) {
            //         machine.mark_position = machine.servo_0->servo_position;
            //     }
            // }
            // else {
            //     set_text_10(display.F2_text, "Start");
            //     if (machine.F2->state_raised) {
            //         stop_positioning(machine.servo_0);
            //         stop_positioning(machine.servo_1);
            //         knife_up();
            //         machine.state = AUTOMAT;
            //     }
            // }

    handle_cutter_state();

}


void handle_cutter_state(void) {
    // Existing sticker_cut_compute function renamed and content moved here
    switch(machine.cutter_state) {
        case CUTTER_IDLE:
            knife_up();
            break;

        case CUTTER_REQUESTED:
            if (machine.servo_0->enc_position == 0.0) {
                machine.cutter_state = AT_HOME;
            } else {
                machine.cutter_state = TO_HOME;
            }
            break;

        case TO_HOME:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, 0.0, 10.0, 500);
            } 
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = AT_HOME;
            }
            break;

        case AT_HOME:
            // Check if knife is above the mark
            if (get_next_stop(detector, machine.servo_1->enc_position) != machine.servo_1->enc_position) {
                raise_error("Znacka nenajdena");
            }
            else {
                machine.cutter_state = TO_PRECUT;
            }
            break;

        case TO_PRECUT:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, PRECUT_POSITION, 4.0, 500);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = BACK_HOME;
                }
            }
            break;

        case BACK_HOME:
            if (machine.servo_0->positioning == IDLE) {
                knife_down();
                servo_goto_delayed(machine.servo_0, 0.0, 10.0, 500);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = CUT_TO_END;
                }
            }
            break;

        case CUT_TO_END:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, CUT_LENGTH, 4.0, 500);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = FINAL_RETURN;
                }
            }
            break;

        case FINAL_RETURN:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 0.0, 4.0, 500);
                knife_up();
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = CUT_DONE;
                }
            }
            break;

        case CUT_DONE:
            machine.cutter_state = ROLL_OUT_PAPER;
            break; 
        
        case ROLL_OUT_PAPER:
            if (machine.servo_0->positioning == IDLE &&  machine.servo_1->positioning == IDLE) {
                servo_goto_delayed(machine.servo_1, machine.servo_1->enc_position + 5.0, 5.0, 500);
            }
            else if (machine.servo_1->positioning == POSITION_REACHED) {
                machine.cutter_state = CUTTER_IDLE;
            }
            break;

        case STOP_CUTTING:
            machine.servo_0->positioning = IDLE;
            machine.cutter_state = CUTTER_IDLE;
            break;
    }
}
