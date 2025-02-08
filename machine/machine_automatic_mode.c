#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "machine_automatic_mode.h"
#include "machine_manual_mode.h"
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
        activate_manual_state();
        return;
    }

    // Handle automatic state transitions
    switch(automatic_substate) {
        case AUTOMATIC_IDLE:
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                automatic_substate = AUTOMATIC_MARK_GOTO;
            }
            break;

        case AUTOMATIC_MARK_GOTO:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_MID, 500);
                automatic_substate = AUTOMATIC_MARK_GOTO_IN_PROGRESS;
            }
            break;

        case AUTOMATIC_MARK_GOTO_IN_PROGRESS:
            set_text_10(display.F2_text, "K znacke");
            if (machine.servo_0->positioning == IDLE) {
                automatic_substate = AUTOMATIC_MARK_POS_REACHED;
            }
            break;
        
        case AUTOMATIC_MARK_POS_REACHED:
            if (machine.servo_0->positioning == IDLE) {
                set_text_10(display.F2_text, "Na znacke");
                servo_goto_delayed(machine.servo_1, 1000.0, AUTOMAT_SPEED_SLOW, 500);
                automatic_substate = AUTOMATIC_REACH_CONSTANT_SPEED;
            }
            break;

        case AUTOMATIC_REACH_CONSTANT_SPEED:
            if (machine.servo_1->nominal_speed_reached) {
                detector_restart();
                automatic_substate = AUTOMATIC_WAITING_FOR_SAMPLES;
            }
            break;

        case AUTOMATIC_WAITING_FOR_SAMPLES:
            if (detector.sampling_done) {
                automatic_substate = AUTOMATIC_SCANNING;
            }
            break;

        case AUTOMATIC_SCANNING:
            if (detect_mark()) {
                automatic_substate = AUTOMATIC_FOUND;
                machine.servo_1->next_stop = (detector.mark_position + 14.0) / machine.servo_1->scale;
            }
            break;

        case AUTOMATIC_FOUND:
            set_text_10(display.F2_text, "Zn Najdeny");
            if (machine.servo_1->positioning == IDLE) {
                set_text_10(display.F2_text, "Zn Najdeny");
            }

            if (machine.F2->state_raised) {
                automatic_substate = AUTOMATIC_MARK_POS_REACHED;
            }
            // automatic_substate = AUTOMATIC_RETURN_TO_ZERO;
            break;

        case AUTOMATIC_RETURN_TO_ZERO:
            if (machine.servo_1->positioning == IDLE) {
                servo_goto_delayed(machine.servo_1, 0.0, 15.0, 500);
                automatic_substate = AUTOMATIC_FINISHED;
            }
            break;

        case AUTOMATIC_FINISHED:
            automatic_substate = AUTOMATIC_IDLE;
            break;
    }

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
            // if (get_next_stop(detector, machine.servo_1->enc_position) != machine.servo_1->enc_position) {
            //     raise_error("Znacka nenajdena");
            // }
            // else {
            //     machine.cutter_state = TO_PRECUT;
            // }
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
