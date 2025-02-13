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
marks_monitor_t monitor_data;

static void marks_monitor(void);
static void stop_knife_on_mark(void);

void activate_automatic_state() {
    machine.state = AUTOMAT;
    automatic_substate = AUTOMATIC_IDLE;

    // Mark monitor
    monitor_data.first_mark_position = 0.0;
    monitor_data.second_mark_position = 0.0;
    monitor_data.third_mark_position = 0.0;
    monitor_data.sticker_height = 0.0;
    monitor_data.mark_distance = 0.0;

    monitor_data.bottom_mark_detected = false;
}

void handle_automatic_state(void) {
    set_text_20(display.state_text_1, "Automat");
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

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the mark position
        case AUTOMATIC_MARK_GOTO:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
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
            set_text_10(display.F2_text, "Na znacke");
            servo_goto_delayed(machine.servo_1, FAR_AWAY_DISTANCE, AUTOMAT_SPEED_SLOW, HALF_SECOND_DELAY);
            automatic_substate = AUTOMATIC_WAIT_FOR_CONSTANT_SPEED;
            break;

// ----------------------------------------------------------------------------------------------------------
// Rolling paper at constant speed
        case AUTOMATIC_WAIT_FOR_CONSTANT_SPEED:
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
                machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET) / machine.servo_1->scale;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Mark found, save positions and move to next step
        case AUTOMATIC_FOUND:
            set_text_10(display.F2_text, "Zn Najdeny");
            if (machine.servo_1->positioning == IDLE) {
                if (monitor_data.first_mark_position == 0) {
                    automatic_substate = AUTOMATIC_SAVE_FIRST_MARK;
                } else if (monitor_data.second_mark_position == 0) {
                    automatic_substate = AUTOMATIC_SAVE_SECOND_MARK;
                } else if (monitor_data.third_mark_position == 0) {
                    automatic_substate = AUTOMATIC_SAVE_THIRD_MARK;
                } else if (monitor_data.bottom_mark_detected == false) {
                    automatic_substate = AUTOMATIC_RETURN_TO_ZERO;
                }
            }
            break;

        // Will save a first mark position and withouth stopping will continue to search the next mark
        case AUTOMATIC_SAVE_FIRST_MARK:
            monitor_data.first_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET;
            automatic_substate = AUTOMATIC_WAIT_FOR_CONSTANT_SPEED;
            break;
        
        // Will save a second mark position, stops and waits for user to confirm the sticker height
        case AUTOMATIC_SAVE_SECOND_MARK:
            monitor_data.second_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET;
            stop_knife_on_mark();
            monitor_data.sticker_height = monitor_data.second_mark_position - monitor_data.first_mark_position;
            set_text_20(display.state_text_1, "Potvrd vysku nalepky");
            set_text_10(display.F2_text, "    Potvrd");
            if (machine.F2->state_raised) {
                set_text_20(display.state_text_1, "Automat");
                automatic_substate = AUTOMATIC_MARK_GOTO;
            }
            break;

        // Will save a third mark position, stops and waits for user to confirm the mark distance
        case AUTOMATIC_SAVE_THIRD_MARK:
            monitor_data.third_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET;
            stop_knife_on_mark();
            monitor_data.mark_distance = monitor_data.third_mark_position - monitor_data.second_mark_position;
            set_text_20(display.state_text_1, "Potvrd vzdial. znac.");
            // set_text_20(display.state_text_2, "Potvrd vzdial. znac.");
            set_text_10(display.F2_text, "    Potvrd");
            if (machine.servo_1->positioning == IDLE && machine.F2->state_raised) {
                servo_goto_delayed(machine.servo_1, monitor_data.third_mark_position - monitor_data.mark_distance / 2.0, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
                set_text_20(display.state_text_1, "Automat");
                automatic_substate = AUTOMATIC_WAIT_FOR_CUT_POSITION;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the cut position and perform the cut
        case AUTOMATIC_GOTO_CUT_POSITION:
            
            break;

        case AUTOMATIC_WAIT_FOR_CUT_POSITION:
            if (machine.servo_1->positioning == IDLE) {
                set_text_10(display.F2_text, " Rezat! :)");
                if (machine.F2->state_raised) {
                    automatic_substate = AUTOMATIC_CUT_OPENING_SECTION;
                }
            }
            break;

        case AUTOMATIC_CUT_OPENING_SECTION:
            if (machine.servo_0->positioning == IDLE) {
                // knife_down();
                servo_goto_delayed(machine.servo_0, machine.paper_begin_position + CUTTING_OVERLAP, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_RETURN_TO_MARK;
            }
            break;

        case AUTOMATIC_CUT_RETURN_TO_MARK:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_REST_SECTION;
            }
            break;

        case AUTOMATIC_CUT_REST_SECTION:
            if (machine.servo_0->positioning == IDLE) {
                // knife_down();
                servo_goto_delayed(machine.servo_0, machine.paper_end_position + CUTTING_OVERLAP, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_RETURN_TO_MARK_2;
            }
            break;

        case AUTOMATIC_CUT_RETURN_TO_MARK_2:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
                // automatic_substate = AUTOMATIC_RETURN_TO_ZERO;
            }
            break;









        case AUTOMATIC_RETURN_TO_ZERO:
            if (machine.servo_1->positioning == IDLE) {
                servo_goto_delayed(machine.servo_1, 0.0, 15.0, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_FINISHED;
            }
            break;

        case AUTOMATIC_FINISHED:
            automatic_substate = AUTOMATIC_IDLE;
            break;
    }

    handle_cutter_state();
    marks_monitor();

}

// This function should monitor distances between marks
static void marks_monitor(void) {
    static float last_mark_position = 0.0;
    static bool first_mark = true;
    static bool measuring = false;
    
    // Start measuring when first mark is detected
    if (automatic_substate == AUTOMATIC_FOUND && !measuring) {
        if (first_mark) {
            last_mark_position = detector.mark_position;
            first_mark = false;
            measuring = true;
        } else {
            float current_distance = abs(detector.mark_position - last_mark_position);
            
            // If we don't have sticker height yet, store it
            if (monitor_data.sticker_height == 0) {
                monitor_data.sticker_height = current_distance;
            }
            // If we don't have mark distance yet and current distance is smaller than sticker height
            else if (monitor_data.mark_distance == 0 && current_distance < monitor_data.sticker_height) {
                monitor_data.mark_distance = current_distance;
            }
            // Validate distances
            else {
                float height_tolerance = monitor_data.sticker_height * 0.1; // 10% tolerance
                float distance_tolerance = monitor_data.mark_distance * 0.1;
                
                // Check if current distance matches either expected distance within tolerance
                if (abs(current_distance - monitor_data.sticker_height) <= height_tolerance) {
                    // Valid sticker height detected
                } else if (abs(current_distance - monitor_data.mark_distance) <= distance_tolerance) {
                    // Valid mark distance detected
                } else {
                    // Invalid distance detected - could trigger an error or warning here
                    *detector.error = true;
                    strcpy(*detector.error_message, "Invalid mark dist");
                }
            }
            
            last_mark_position = detector.mark_position;
        }
    }
    
    // Reset measurements when returning to idle
    if (automatic_substate == AUTOMATIC_IDLE) {
        first_mark = true;
        measuring = false;
    }
}

static void stop_knife_on_mark(void) {
    machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET) / machine.servo_1->scale;
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
                servo_goto_delayed(machine.servo_0, 0.0, 10.0, HALF_SECOND_DELAY);
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
                servo_goto_delayed(machine.servo_0, PRECUT_POSITION, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = BACK_HOME;
                }
            }
            break;

        case BACK_HOME:
            if (machine.servo_0->positioning == IDLE) {
                knife_down();
                servo_goto_delayed(machine.servo_0, 0.0, 10.0, HALF_SECOND_DELAY);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = CUT_TO_END;
                }
            }
            break;

        case CUT_TO_END:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, CUT_LENGTH, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = FINAL_RETURN;
                }
            }
            break;

        case FINAL_RETURN:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 0.0, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
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
                servo_goto_delayed(machine.servo_1, machine.servo_1->enc_position + 5.0, 5.0, HALF_SECOND_DELAY);
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
