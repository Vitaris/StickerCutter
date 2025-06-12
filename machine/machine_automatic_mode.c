#include "pico/stdlib.h"
#include "machine_controller.h"
#include "machine_automatic_mode.h"
#include "machine_manual_mode.h"
#include "mark_detector.h"

static const float STICKER_HEIGHT_TOLERNACE = 10.0; // 10mm tolerance for sticker height
char state_text_1[21];
char state_text_2[21];

/**
 * @brief Structure for monitoring and storing mark positions and sticker dimensions
 * Used during the initialization phase to learn sticker dimensions and later
 * for validating proper mark detection during operation
 */
typedef struct {
    bool sticker_dimensions_set;          // Indicates if sticker dimensions are learned
    
    float sticker_height;                 // Height of a single sticker
    float mark_distance;                  // Distance between stickers in a row
    float current_sticker_measurement;    // Current sticker measurement
    
    float first_mark_position;            // Position of first detected mark
    float second_mark_position;           // Position of second detected mark
    float third_mark_position;            // Position of third detected mark

    float last_stop_position;             // Last known position of the cutting head
} marks_monitor_t;

typedef enum {
    IDLE,                           // Waiting for start command
    
    // Mark detection positioning states
    MARK_SEEK_START,               // Moving knife to initial mark position
    MARK_SEEK_MOVING,              // Waiting for knife to reach mark position
    MARK_SEEK_READY,               // Knife has reached mark position
    
    // Paper movement states
    PAPER_START_FEED,              // Begin feeding paper
    PAPER_AWAIT_SPEED,             // Waiting for stable paper feed
    
    // Detection states
    DETECT_AWAIT_SAMPLES,          // Collecting sensor readings
    DETECT_SCANNING,               // Actively looking for marks
    DETECT_MARK_FOUND,             // Mark detected
    
    // Learning/calibration states
    LEARN_FIRST_MARK,             // Recording position of first mark
    LEARN_SECOND_MARK,            // Recording position of second mark
    LEARN_THIRD_MARK,             // Recording position of third mark
    
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

    // Mark Monitor
    MONITOR_STICKER_HEIGHT_FAILURE,      // 
    MONITOR_MARK_DISTANCE_FAILURE,       //
    
    // Machine movement states
    HOME_RETURN,                 // Returning to home position
    COMPLETE                     // Cycle complete
} automatic_substate_t;


automatic_substate_t automatic_substate;
marks_monitor_t monitor_data;

void stop_knife_on_mark(void) {
    servo_set_stop_position(devices.servo_feeder, get_mark_position() + SENSOR_KNIFE_OFFSET_Y);
}

void stop_knife_between_marks(void) {
    servo_set_stop_position(devices.servo_feeder, get_mark_position() + SENSOR_KNIFE_OFFSET_Y + (monitor_data.mark_distance / 2.0));
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

    // Mark monitor
    monitor_data.first_mark_position = 0.0;
    monitor_data.second_mark_position = 0.0;
    monitor_data.third_mark_position = 0.0;
    monitor_data.sticker_height = 0.0;
    monitor_data.mark_distance = 0.0;
    monitor_data.current_sticker_measurement = 0.0;
    monitor_data.sticker_dimensions_set = false;
}

void handle_automatic_state(void) {
    set_text_20(machine.state_text_1, "Automat");
    set_text_20(machine.state_text_2, "");
    set_text_10(machine.F1_text, "Stop");

    if (button_raised(devices.F1)) {
        activate_manual_state();
        return;
    }

    // Handle automatic state transitions
    switch(automatic_substate) {
        case IDLE:
            set_text_10(machine.F2_text, "    Start ");
            if (button_raised(devices.F2)) {
                automatic_substate = MARK_SEEK_START;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the mark position
        case MARK_SEEK_START:
            if (servo_is_idle(devices.servo_cutter)) {
                servo_goto_delayed(devices.servo_cutter, machine.paper_right_mark_position, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = MARK_SEEK_MOVING;
            }
            break;

        case MARK_SEEK_MOVING:
            set_text_10(machine.F2_text, "K znacke");
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = MARK_SEEK_READY;
            }
            break;
        
        case MARK_SEEK_READY:
            set_text_10(machine.F2_text, "Na znacke");
            automatic_substate = PAPER_START_FEED;
            break;
            
// ----------------------------------------------------------------------------------------------------------
// Rolling paper at constant speed
        case PAPER_START_FEED:
            monitor_data.last_stop_position = servo_get_position(devices.servo_feeder);
            servo_goto_delayed(devices.servo_feeder, FAR_AWAY_DISTANCE, AUTOMAT_SPEED_SCAN, HALF_SECOND_DELAY);
            automatic_substate = PAPER_AWAIT_SPEED;
            break;

        case PAPER_AWAIT_SPEED:
            if (servo_is_speed_reached(devices.servo_feeder)) {
                detector_restart();
                automatic_substate = DETECT_AWAIT_SAMPLES;
            }
            break;

        case DETECT_AWAIT_SAMPLES:
            if (is_sampling_done()) {
                automatic_substate = DETECT_SCANNING;
            }
            break;

        case DETECT_SCANNING:
            monitor_data.current_sticker_measurement = servo_get_position(devices.servo_feeder) - monitor_data.last_stop_position;
            if (monitor_data.sticker_dimensions_set &&
                monitor_data.current_sticker_measurement >= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
                    automatic_substate = MONITOR_STICKER_HEIGHT_FAILURE;
                }
            if (detect_mark()) {
                automatic_substate = DETECT_MARK_FOUND;
                // devices.servo_feeder->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y) / devices.servo_feeder->scale;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Mark found, save positions and move to next step
        case DETECT_MARK_FOUND:
            if (monitor_data.sticker_dimensions_set) {
                automatic_substate = CUT_STOP_AT_MARK;
            }
            else {
                if (monitor_data.first_mark_position == 0) {
                        set_text_10(machine.F2_text, "  Zn 1 OK");
                        automatic_substate = LEARN_FIRST_MARK;
                    } else if (monitor_data.second_mark_position == 0) {
                        set_text_10(machine.F2_text, "  Zn 2 OK");
                        automatic_substate = LEARN_SECOND_MARK;
                    } else if (monitor_data.third_mark_position == 0) {
                        set_text_10(machine.F2_text, "  Zn 3 OK");
                        automatic_substate = LEARN_THIRD_MARK;
                    }
                }
            break;

        // Will save a first mark position and withouth stopping will continue to search the next mark
        case LEARN_FIRST_MARK:
            monitor_data.first_mark_position = get_mark_position() + SENSOR_KNIFE_OFFSET_Y;
            automatic_substate = PAPER_AWAIT_SPEED;
            break;
        
        // Will save a second mark position, stops and waits for user to confirm the sticker height
        case LEARN_SECOND_MARK:
            monitor_data.second_mark_position = get_mark_position() + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            monitor_data.sticker_height = monitor_data.second_mark_position - monitor_data.first_mark_position;
            set_text_20(machine.state_text_1, "Potvrd vysku nalepky");

            snprintf(state_text_2, sizeof(state_text_2), "Vyska: %.1fmm", monitor_data.sticker_height);
            set_text_20(machine.state_text_2, state_text_2);
            
            set_text_10(machine.F2_text, "    Potvrd");
            if (button_raised(devices.F2)) {
                set_text_20(machine.state_text_1, "Automat");
                automatic_substate = PAPER_START_FEED;
            }
            break;

        // Will save a third mark position, stops and waits for user to confirm the mark distance
        case LEARN_THIRD_MARK:
            monitor_data.third_mark_position = get_mark_position() + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            monitor_data.mark_distance = monitor_data.third_mark_position - monitor_data.second_mark_position;
            set_text_20(machine.state_text_1, "Potvrd vzdial. znac.");

            snprintf(state_text_2, sizeof(state_text_2), "Znacky: %.1fmm", monitor_data.mark_distance);
            set_text_20(machine.state_text_2, state_text_2);

            set_text_10(machine.F2_text, "    Potvrd");
            if (servo_is_idle(devices.servo_feeder) && button_raised(devices.F2)) {
                monitor_data.sticker_dimensions_set = true;
                automatic_substate = CUT_MOVE_TO_START;
            }
            break;

        case CUT_STOP_AT_MARK:
            stop_knife_between_marks();
            if (servo_is_idle(devices.servo_feeder)) {
                if (false && monitor_data.current_sticker_measurement <= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
                    automatic_substate = MONITOR_STICKER_HEIGHT_FAILURE;
                }
                else {
                    automatic_substate = CUT_BEGIN_SEQUENCE;
                }
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the cut position and perform the cut
        case CUT_MOVE_TO_START:
            set_text_20(machine.state_text_1, "Automat");
            servo_goto_delayed(devices.servo_feeder, monitor_data.third_mark_position - monitor_data.mark_distance / 2.0, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
            automatic_substate = CUT_AWAIT_POSITION;
            break;

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
                servo_goto_delayed(devices.servo_feeder, (servo_get_position(devices.servo_feeder) + (monitor_data.mark_distance / 2)), AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
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
            snprintf(state_text_2, sizeof(state_text_2), "znacky: %.1fmm !", monitor_data.current_sticker_measurement);
            set_text_20(machine.state_text_2, state_text_2);
            set_text_10(machine.F2_text, "Reset Auto");
            if (button_raised(devices.F2)) {
                automatic_substate = IDLE;
            }
            break;

        case MONITOR_MARK_DISTANCE_FAILURE:
            servo_stop_positioning(devices.servo_feeder);
            set_text_20(machine.state_text_1, "Nespravna vyska!");
            snprintf(state_text_2, sizeof(state_text_2), "medzery: %.1fmm !", monitor_data.current_sticker_measurement);
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
}
