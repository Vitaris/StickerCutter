#include "pico/stdlib.h"
#include "../hmi/hmi.h"
#include "machine_controller.h"
#include "machine_automatic_mode.h"
#include "machine_manual_mode.h"
#include "../servo_motor/servo_motor.h"

static const float STICKER_HEIGHT_TOLERNACE = 10.0; // 10mm tolerance for sticker height

typedef enum {
    IDLE,                           // Waiting for start command
    
    // Paper movement states
    NAVIGATE_TO_LEAD_IN_RIGHT,    // 
    MOVING_TO_LEAD_IN_RIGHT,
    LEAD_IN_READY_RIGHT,

    START_ENTRY_CUT_RIGHT,
    ENTRY_CUT_IN_PROGRESS_RIGHT,
    ENTRY_CUT_DONE_RIGHT,
    ENTRY_CUT_RETURN_RIGHT,
    RETURN_TO_LEAD_IN_RIGHT,
    PERFORM_REST_OF_CUT_TO_RIGHT,
    REST_OF_CUT_FROM_RIGHT_IN_PROGRESS,
    REST_OF_CUT_FROM_RIGHT_DONE,

    NAVIGATE_TO_LEAD_IN_LEFT,
    MOVING_TO_LEAD_IN_LEFT,
    LEAD_IN_READY_LEFT,

    START_ENTRY_CUT_LEFT,
    ENTRY_CUT_IN_PROGRESS_LEFT,
    ENTRY_CUT_DONE_LEFT,
    RETURN_TO_LEAD_IN_LEFT,
    PERFORM_REST_OF_CUT_TO_LEFT,
    REST_OF_CUT_FROM_LEFT_IN_PROGRESS,
    REST_OF_CUT_FROM_LEFT_DONE,



    PAPER_START_FEED,              // Begin feeding paper
    PAPER_AWAIT_SPEED,             // Waiting for stable paper feed
   
    // Cutting preparation states
    CUT_STOP_AT_MARK,             // Stop centered between two marks
    CUT_MOVE_TO_START,            // Moving to cutting position
    CUT_AWAIT_POSITION,           // Waiting to reach cutting position
    
    // Cutting sequence states
    CUT_BEGIN_SEQUENCE,           // Starting the cut sequence
    CUT_REACH_EDGE,              // Continuing cut sequence
    CUT_RETURN_CENTER,           // Moving back to mark after first cut
    CUT_FINISH_SEQUENCE,         // Cutting remaining section
    
    // Next cycle preparation states
    PREP_NEXT_CYCLE,             // Moving to starting position for next cut
    PREP_NEW_DETECTION,          // Preparing for next mark detection

    // Machine movement states
    HOME_RETURN,                 // Returning to home position
    COMPLETE                     // Cycle complete
} automatic_substate_t;


automatic_substate_t automatic_substate;

int get_mark_position(void) {
    return 0; // Placeholder for actual mark position retrieval logic
}

void stop_knife_on_mark(void) {
    servo_set_stop_position(devices.servo_feeder, get_mark_position() + SENSOR_KNIFE_OFFSET_Y);
}

void stop_knife_between_marks(void) {
    // servo_set_stop_position(devices.servo_feeder, get_mark_position() + SENSOR_KNIFE_OFFSET_Y + (monitor_data.mark_distance / 2.0));
    servo_set_stop_position(devices.servo_feeder, get_mark_position() + SENSOR_KNIFE_OFFSET_Y);
}

void reset_paper_mark_positions(void) {
    machine.paper_right_mark_position = 0.0;
}

bool is_paper_positions_set(void) {
    return machine.paper_right_mark_position != 0.0;
}

void activate_automatic_state() {
    machine_state = AUTOMAT;
    automatic_substate = IDLE;
}

void handle_automatic_state(void) {
    hmi_set_status("Automat");
    hmi_set_left_button("", "Stop");
    hmi_set_right_button("", "");

    if (button_raised(devices.F1)) {
        activate_manual_state();
        return;
    }

    // Handle automatic state transitions
    switch(automatic_substate) {
        case IDLE:
            hmi_set_right_button("", "Start");
            knife_up();
            if (button_raised(devices.F2)) {
                automatic_substate = NAVIGATE_TO_LEAD_IN_RIGHT;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the mark position
        case NAVIGATE_TO_LEAD_IN_RIGHT:
        hmi_set_right_button("", "1R");
            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                servo_goto_delayed(devices.servo_cutter, -machine.cut_overlap, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                servo_goto_delayed(devices.servo_feeder, servo_get_position(devices.servo_feeder) + machine.sticker_height, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = MOVING_TO_LEAD_IN_RIGHT;
            }
            break;

        case MOVING_TO_LEAD_IN_RIGHT:
            hmi_set_right_button("", "2R");
            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                automatic_substate = LEAD_IN_READY_RIGHT;
            }
            break;
        
        case LEAD_IN_READY_RIGHT:
            hmi_set_right_button("", "3R");
            automatic_substate = START_ENTRY_CUT_RIGHT;
            break;


        case START_ENTRY_CUT_RIGHT:
            hmi_set_right_button("", "4R");
            knife_down();
            servo_goto_delayed(devices.servo_cutter, 0.0, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = ENTRY_CUT_IN_PROGRESS_RIGHT;
            break;

        case ENTRY_CUT_IN_PROGRESS_RIGHT:
            hmi_set_right_button("", "5R");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = ENTRY_CUT_DONE_RIGHT;
            }
            break;

        case ENTRY_CUT_DONE_RIGHT:
            hmi_set_right_button("", "6R");
            knife_up();
            servo_goto_delayed(devices.servo_cutter, -machine.cut_overlap, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = RETURN_TO_LEAD_IN_RIGHT;
            break;

        case RETURN_TO_LEAD_IN_RIGHT:
            hmi_set_right_button("", "7R");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = PERFORM_REST_OF_CUT_TO_RIGHT;
            }
            break;

        case PERFORM_REST_OF_CUT_TO_RIGHT:
            hmi_set_right_button("", "8R");
            knife_down();
            servo_goto_delayed(devices.servo_cutter, -machine.paper_width, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = REST_OF_CUT_FROM_RIGHT_IN_PROGRESS;
            break;

        case REST_OF_CUT_FROM_RIGHT_IN_PROGRESS:
            hmi_set_right_button("", "9R");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = REST_OF_CUT_FROM_RIGHT_DONE;
            }
            break;

        case REST_OF_CUT_FROM_RIGHT_DONE:
            hmi_set_right_button("", "10R");
            automatic_substate = NAVIGATE_TO_LEAD_IN_LEFT;
            break;


        // ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the left lead-in position
        case NAVIGATE_TO_LEAD_IN_LEFT:
            hmi_set_right_button("", "1L");
            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                // Navigate to the left side, offset inward by the cut overlap
                servo_goto_delayed(devices.servo_cutter, -machine.paper_width + machine.cut_overlap, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                // Advance the paper by sticker height
                servo_goto_delayed(devices.servo_feeder, servo_get_position(devices.servo_feeder) + machine.sticker_height, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = MOVING_TO_LEAD_IN_LEFT;
            }
            break;

        case MOVING_TO_LEAD_IN_LEFT:
            hmi_set_right_button("", "2L");
            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                automatic_substate = LEAD_IN_READY_LEFT;
            }
            break;
        
        case LEAD_IN_READY_LEFT:
            hmi_set_right_button("", "3L");
            automatic_substate = START_ENTRY_CUT_LEFT;
            break;

        case START_ENTRY_CUT_LEFT:
            hmi_set_right_button("", "4L");
            knife_down();
            // Perform lead-in cut outward to the absolute left edge
            servo_goto_delayed(devices.servo_cutter, -machine.paper_width, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = ENTRY_CUT_IN_PROGRESS_LEFT;
            break;

        case ENTRY_CUT_IN_PROGRESS_LEFT:
            hmi_set_right_button("", "5L");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = ENTRY_CUT_DONE_LEFT;
            }
            break;

        case ENTRY_CUT_DONE_LEFT:
            hmi_set_right_button("", "6L");
            knife_up();
            // Return cutter to the offset position for the main cut
            servo_goto_delayed(devices.servo_cutter, -machine.paper_width + machine.cut_overlap, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = RETURN_TO_LEAD_IN_LEFT;
            break;

        case RETURN_TO_LEAD_IN_LEFT:
            hmi_set_right_button("", "7L");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = PERFORM_REST_OF_CUT_TO_LEFT;
            }
            break;

        case PERFORM_REST_OF_CUT_TO_LEFT:
            hmi_set_right_button("", "8L");
            knife_down();
            // Perform the rest of the cut all the way to the right edge
            servo_goto_delayed(devices.servo_cutter, 0.0, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = REST_OF_CUT_FROM_LEFT_IN_PROGRESS;
            break;

        case REST_OF_CUT_FROM_LEFT_IN_PROGRESS:
            hmi_set_right_button("", "9L");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = REST_OF_CUT_FROM_LEFT_DONE;
            }
            break;

        case REST_OF_CUT_FROM_LEFT_DONE:
            hmi_set_right_button("", "10L");
            // End of left cut sequence, ready to trigger right side or next sticker
            automatic_substate = NAVIGATE_TO_LEAD_IN_RIGHT;
            break;
            /*
            
// ----------------------------------------------------------------------------------------------------------
// Rolling paper at constant speed
        case PAPER_START_FEED:
            servo_goto_delayed(devices.servo_feeder, FAR_AWAY_DISTANCE, AUTOMAT_SPEED_SCAN, HALF_SECOND_DELAY);
            automatic_substate = PAPER_AWAIT_SPEED;
            break;


        

        case PAPER_AWAIT_SPEED:
            if (servo_is_speed_reached(devices.servo_feeder)) {
                automatic_substate = DETECT_AWAIT_SAMPLES;
            }
            break;

        case DETECT_AWAIT_SAMPLES:
            automatic_substate = DETECT_SCANNING;
            break;

        case DETECT_SCANNING:
            // monitor_data.current_sticker_measurement = servo_get_position(devices.servo_feeder) - monitor_data.last_stop_position;
            // if (monitor_data.sticker_dimensions_set &&
            //     monitor_data.current_sticker_measurement >= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
            //         automatic_substate = MONITOR_STICKER_HEIGHT_FAILURE;
            // }
            break;

// ----------------------------------------------------------------------------------------------------------
// Mark found, save positions and move to next step
        case DETECT_MARK_FOUND:
            // if (monitor_data.sticker_dimensions_set) {
            // //     automatic_substate = CUT_STOP_AT_MARK;
            // // }
            // else {
            //     if (monitor_data.first_mark_position == 0) {
            //             set_text_10(machine.F2_text, "  Zn 1 OK");
            //             automatic_substate = LEARN_FIRST_MARK;
            //         } else if (monitor_data.second_mark_position == 0) {
            //             set_text_10(machine.F2_text, "  Zn 2 OK");
            //             automatic_substate = LEARN_SECOND_MARK;
            //         } else if (monitor_data.third_mark_position == 0) {
            //             set_text_10(machine.F2_text, "  Zn 3 OK");
            //             automatic_substate = LEARN_THIRD_MARK;
            //         }
            //     }
            break;

        // Will save a first mark position and withouth stopping will continue to search the next mark
        case LEARN_FIRST_MARK:
            // monitor_data.first_mark_position = get_mark_position() + SENSOR_KNIFE_OFFSET_Y;
            // monitor_data.last_stop_position = monitor_data.first_mark_position;
            automatic_substate = PAPER_AWAIT_SPEED;
            break;
        
        // Will save a second mark position, stops and waits for user to confirm the sticker height
        case LEARN_SECOND_MARK:
            // monitor_data.second_mark_position = get_mark_position() + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            // monitor_data.sticker_height = monitor_data.second_mark_position - monitor_data.first_mark_position;
            set_text_20(machine.state_text_1, "Potvrd vysku nalepky");

            // snprintf(state_text_2, sizeof(state_text_2), "Vyska: %.1fmm", monitor_data.sticker_height);
            set_text_20(machine.state_text_2, state_text_2);
            
            set_text_10(machine.F2_text, "    Potvrd");
            if (button_raised(devices.F2)) {
                set_text_20(machine.state_text_1, "Automat");
                automatic_substate = PAPER_START_FEED;
            }
            break;

        // Will save a third mark position, stops and waits for user to confirm the mark distance
        case LEARN_THIRD_MARK:
            // monitor_data.third_mark_position = get_mark_position() + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            // monitor_data.mark_distance = monitor_data.third_mark_position - monitor_data.second_mark_position;
            set_text_20(machine.state_text_1, "Potvrd vzdial. znac.");

            // snprintf(state_text_2, sizeof(state_text_2), "Znacky: %.1fmm", monitor_data.mark_distance);
            set_text_20(machine.state_text_2, state_text_2);

            set_text_10(machine.F2_text, "    Potvrd");
            if (servo_is_idle(devices.servo_feeder) && button_raised(devices.F2)) {
                // monitor_data.sticker_dimensions_set = true;
                automatic_substate = CUT_MOVE_TO_START;
            }
            break;

        case CUT_STOP_AT_MARK:
            stop_knife_between_marks();
            // if (servo_is_idle(devices.servo_feeder)) {
            //     if (monitor_data.current_sticker_measurement <= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
            //         automatic_substate = MONITOR_STICKER_HEIGHT_FAILURE;
            //     }
            //     else {
            //         monitor_data.last_stop_position = monitor_data.current_sticker_measurement;
            //         automatic_substate = CUT_BEGIN_SEQUENCE;
            //     }
            // }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the cut position and perform the cut
        // case CUT_MOVE_TO_START:
        //     set_text_20(machine.state_text_1, "Automat");
        //     servo_goto_delayed(devices.servo_feeder, monitor_data.third_mark_position - monitor_data.mark_distance / 2.0, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
        //     automatic_substate = CUT_AWAIT_POSITION;
        //     break;

        case CUT_AWAIT_POSITION:
            if (servo_is_idle(devices.servo_feeder)) {
                set_text_10(machine.F2_text, " Rezat! :)");
                if (button_raised(devices.F2)) {
                    automatic_substate = CUT_BEGIN_SEQUENCE;
                }
            }
            break;
            
        case CUT_BEGIN_SEQUENCE:
            set_text_10(machine.F2_text, "");
            if (servo_is_idle(devices.servo_cutter)) {
                servo_goto_delayed(devices.servo_cutter, machine.paper_right_mark_position - 50.0, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = CUT_REACH_EDGE;

            }
            break;

        case CUT_REACH_EDGE:
            if (servo_is_idle(devices.servo_cutter)) {
                knife_down();
                servo_goto_delayed(devices.servo_cutter, POSITION_EDGE_RIGHT, AUTOMAT_SPEED_CUT, HALF_SECOND_DELAY);
                automatic_substate = CUT_RETURN_CENTER;
            }
            break;

        case CUT_RETURN_CENTER:
            if (servo_is_idle(devices.servo_cutter)) {
                knife_up();
                servo_goto_delayed(devices.servo_cutter, machine.paper_right_mark_position - 50 , AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = CUT_FINISH_SEQUENCE;
            }
            break;

        case CUT_FINISH_SEQUENCE:
            if (servo_is_idle(devices.servo_cutter)) {
                knife_down();
                servo_goto_delayed(devices.servo_cutter, POSITION_EDGE_LEFT, AUTOMAT_SPEED_CUT, HALF_SECOND_DELAY);
                automatic_substate = PREP_NEXT_CYCLE;
            }
            break;

        case PREP_NEXT_CYCLE:
            if (servo_is_idle(devices.servo_cutter)) {
                knife_up();
                servo_goto_delayed(devices.servo_cutter, machine.paper_right_mark_position, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                // servo_goto_delayed(devices.servo_feeder, (servo_get_position(devices.servo_feeder) + (monitor_data.mark_distance / 2)), AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = PREP_NEW_DETECTION;
            }
            break;

        case PREP_NEW_DETECTION:
            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                automatic_substate = PAPER_START_FEED;
            }
            break;

        case MONITOR_STICKER_HEIGHT_FAILURE:
            servo_stop_positioning(devices.servo_feeder);
            set_text_20(machine.state_text_1, "Nespravna vyska!");
            // snprintf(state_text_2, sizeof(state_text_2), "znacky: %.1fmm !", monitor_data.current_sticker_measurement);
            set_text_20(machine.state_text_2, state_text_2);
            set_text_10(machine.F2_text, "Reset Auto");
            if (button_raised(devices.F2)) {
                automatic_substate = IDLE;
            }
            break;

        case MONITOR_MARK_DISTANCE_FAILURE:
            servo_stop_positioning(devices.servo_feeder);
            set_text_20(machine.state_text_1, "Nespravna vyska!");
            // snprintf(state_text_2, sizeof(state_text_2), "medzery: %.1fmm !", monitor_data.current_sticker_measurement);
            set_text_20(machine.state_text_2, state_text_2);
            set_text_10(machine.F2_text, "Reset Auto");
            if (button_raised(devices.F2)) {
                automatic_substate = IDLE;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Should be used as stop case of the automatic mode
        case COMPLETE:
            automatic_substate = IDLE;
            break;
    }
            */
        }
}
