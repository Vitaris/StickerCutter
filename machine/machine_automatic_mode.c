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
static void stop_knife_between_marks(void);

void activate_automatic_state() {
    machine.state = AUTOMAT;
    automatic_substate = AUTOMATIC_IDLE;

    // Mark monitor
    monitor_data.first_mark_position = 0.0;
    monitor_data.second_mark_position = 0.0;
    monitor_data.third_mark_position = 0.0;
    monitor_data.sticker_height = 0.0;
    monitor_data.mark_distance = 0.0;

    monitor_data.sticker_dimensions_set = false;
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
            automatic_substate = AUTOMATIC_START_ROLLING_OUT_PAPER;
            break;
            
// ----------------------------------------------------------------------------------------------------------
// Rolling paper at constant speed
        case AUTOMATIC_START_ROLLING_OUT_PAPER:
            monitor_data.last_stop_position = machine.servo_1->servo_position;
            servo_goto_delayed(machine.servo_1, FAR_AWAY_DISTANCE, AUTOMAT_SPEED_SLOW, HALF_SECOND_DELAY);
            automatic_substate = AUTOMATIC_WAIT_FOR_CONSTANT_SPEED_OF_PAPER;
            break;

        case AUTOMATIC_WAIT_FOR_CONSTANT_SPEED_OF_PAPER:
            if (machine.servo_1->nominal_speed_reached) {
                detector_restart();
                automatic_substate = AUTOMATIC_WAITING_FOR_SAMPLES;
            }
            break;

        case AUTOMATIC_WAITING_FOR_SAMPLES:
            if (detector.sampling_done) {
                automatic_substate = AUTOMATIC_SCANNING_FOR_MARK;
            }
            break;

        case AUTOMATIC_SCANNING_FOR_MARK:
            if (monitor_data.sticker_dimensions_set &&
                machine.servo_1->servo_position - monitor_data.last_stop_position >= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
                    // Raise error
                }
            if (detect_mark()) {
                automatic_substate = AUTOMATIC_MARK_FOUND;
                // machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y) / machine.servo_1->scale;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Mark found, save positions and move to next step
        case AUTOMATIC_MARK_FOUND:
            set_text_10(display.F2_text, "Zn Najdeny");
            if (monitor_data.sticker_dimensions_set) {
                automatic_substate = AUTOMATIC_REGULAR_CUT_STOP;
            } else {
                if (monitor_data.first_mark_position == 0) {
                    automatic_substate = AUTOMATIC_SAVE_FIRST_MARK;
                } else if (monitor_data.second_mark_position == 0) {
                    automatic_substate = AUTOMATIC_SAVE_SECOND_MARK;
                } else if (monitor_data.third_mark_position == 0) {
                    automatic_substate = AUTOMATIC_SAVE_THIRD_MARK; }
            }
            break;

        // Will save a first mark position and withouth stopping will continue to search the next mark
        case AUTOMATIC_SAVE_FIRST_MARK:
            monitor_data.first_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET_Y;
            automatic_substate = AUTOMATIC_WAIT_FOR_CONSTANT_SPEED_OF_PAPER;
            break;
        
        // Will save a second mark position, stops and waits for user to confirm the sticker height
        case AUTOMATIC_SAVE_SECOND_MARK:
            monitor_data.second_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            monitor_data.sticker_height = monitor_data.second_mark_position - monitor_data.first_mark_position;
            set_text_20(display.state_text_1, "Potvrd vysku nalepky");
            set_text_10(display.F2_text, "    Potvrd");
            if (machine.F2->state_raised) {
                set_text_20(display.state_text_1, "Automat");
                automatic_substate = AUTOMATIC_START_ROLLING_OUT_PAPER;
            }
            break;

        // Will save a third mark position, stops and waits for user to confirm the mark distance
        case AUTOMATIC_SAVE_THIRD_MARK:
            monitor_data.third_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            monitor_data.mark_distance = monitor_data.third_mark_position - monitor_data.second_mark_position;
            set_text_20(display.state_text_1, "Potvrd vzdial. znac.");
            // set_text_20(display.state_text_2, "Potvrd vzdial. znac.");
            set_text_10(display.F2_text, "    Potvrd");
            if (machine.servo_1->positioning == IDLE && machine.F2->state_raised) {
                monitor_data.sticker_dimensions_set = true;
                automatic_substate = AUTOMATIC_GOTO_CUT_POSITION;
            }
            break;

        case AUTOMATIC_REGULAR_CUT_STOP:
            stop_knife_between_marks();
            if (machine.servo_1->positioning == IDLE) {
                if (machine.servo_1->servo_position - monitor_data.last_stop_position <= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
                    // Raise error
                }
                automatic_substate = AUTOMATIC_CUT_OPENING_SECTION;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the cut position and perform the cut
        case AUTOMATIC_GOTO_CUT_POSITION:
            set_text_20(display.state_text_1, "Automat");
            servo_goto_delayed(machine.servo_1, monitor_data.third_mark_position - monitor_data.mark_distance / 2.0, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
            automatic_substate = AUTOMATIC_WAIT_FOR_CUT_POSITION;
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
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position - 50.0, AUTOMAT_SPEED_NORMAL, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_OPENING_SECTION2;

            }
            break;

        case AUTOMATIC_CUT_OPENING_SECTION2:
            if (machine.servo_0->positioning == IDLE) {
                // knife_down();
                servo_goto_delayed(machine.servo_0, machine.paper_begin_position - SENSOR_KNIFE_OFFSET_X + CUTTING_OVERLAP, AUTOMAT_SPEED_NORMAL, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_RETURN_TO_MARK;
            }
            break;

        case AUTOMATIC_CUT_RETURN_TO_MARK:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position - 50 , AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_REST_SECTION;
            }
            break;

        case AUTOMATIC_CUT_REST_SECTION:
            if (machine.servo_0->positioning == IDLE) {
                // knife_down();
                servo_goto_delayed(machine.servo_0, machine.paper_end_position - SENSOR_KNIFE_OFFSET_X - CUTTING_OVERLAP, AUTOMAT_SPEED_NORMAL, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_CUT_RETURN_TO_MARK_2;
            }
            break;

        case AUTOMATIC_CUT_RETURN_TO_MARK_2:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                servo_goto_delayed(machine.servo_1, (machine.servo_1->servo_position + (monitor_data.mark_distance / 2)), AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = AUTOMATIC_RETURN_NEW_DETECTION;
                // Tu ma byt nieco ako hladat znovu znacku
            }
            break;

        case AUTOMATIC_RETURN_NEW_DETECTION:
            if (machine.servo_0->positioning == IDLE && machine.servo_1->positioning == IDLE) {
                automatic_substate = AUTOMATIC_START_ROLLING_OUT_PAPER;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Should be used as stop case of the automatic mode
        case AUTOMATIC_FINISHED:
            automatic_substate = AUTOMATIC_IDLE;
            break;
    }
}

static void stop_knife_on_mark(void) {
    machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y) / machine.servo_1->scale;
}

static void stop_knife_between_marks(void) {
    machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y + (monitor_data.mark_distance / 2.0)) / machine.servo_1->scale;
}
