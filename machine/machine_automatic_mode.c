#include "pico/stdlib.h"
#include "machine_controller.h"
#include "machine_automatic_mode.h"
#include "machine_manual_mode.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

#define STICKER_HEIGHT_TOLERNACE 10.0 // 10mm tolerance for sticker height

/**
 * @brief Structure for monitoring and storing mark positions and sticker dimensions
 * Used during the initialization phase to learn sticker dimensions and later
 * for validating proper mark detection during operation
 */
typedef struct {
    bool sticker_dimensions_set;          // Indicates if sticker dimensions are learned
    
    float sticker_height;                 // Height of a single sticker
    float mark_distance;                  // Distance between stickers in a row
    
    float first_mark_position;            // Position of first detected mark
    float second_mark_position;           // Position of second detected mark
    float third_mark_position;            // Position of third detected mark

    float last_stop_position;             // Last known position of the cutting head
} marks_monitor_t;

/**
 * @brief States for the automatic cutting mode state machine
 * Controls the sequence of operations from mark detection to cutting
 */
typedef enum {
    IDLE_A,                           // Waiting for start command
    
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
    
    // Machine movement states
    HOME_RETURN,                 // Returning to home position
    COMPLETE                     // Cycle complete
} automatic_substate_t;

/**
 * @brief Represents the current substate in automatic operation mode
 * 
 * This variable holds the state within the automatic operation mode,
 * tracking different phases of the automated process.
 * The possible values are defined in the automatic_substate_t enum.
 */
automatic_substate_t automatic_substate;

/**
 * @brief Data structure for monitoring marking device activities and status
 * 
 * This structure stores real-time monitoring information related to the
 * marking operations, including status flags, counters, and sensor data
 * for the automatic mode of the machine.
 */
marks_monitor_t monitor_data;

/**
 * @brief Stops the knife operation when a cut mark is detected
 * 
 * This function handles the automatic stopping of the knife mechanism when
 * a cut mark is encountered during the scanning process.
 */
void stop_knife_on_mark(void);

/**
 * @brief Stops the knife between marks during automatic operation
 * 
 * This function handles the stopping of the knife between marking points
 * in the automatic cutting mode. It ensures the knife stops at the correct
 * position between material marks.
 */
void stop_knife_between_marks(void);

void activate_automatic_state() {
    machine.state = AUTOMAT;
    automatic_substate = IDLE_A;

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
        case IDLE_A:
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                automatic_substate = MARK_SEEK_START;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the mark position
        case MARK_SEEK_START:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = MARK_SEEK_MOVING;
            }
            break;

        case MARK_SEEK_MOVING:
            set_text_10(display.F2_text, "K znacke");
            if (machine.servo_0->positioning == IDLE) {
                automatic_substate = MARK_SEEK_READY;
            }
            break;
        
        case MARK_SEEK_READY:
            set_text_10(display.F2_text, "Na znacke");
            automatic_substate = PAPER_START_FEED;
            break;
            
// ----------------------------------------------------------------------------------------------------------
// Rolling paper at constant speed
        case PAPER_START_FEED:
            monitor_data.last_stop_position = machine.servo_1->servo_position;
            servo_goto_delayed(machine.servo_1, FAR_AWAY_DISTANCE, AUTOMAT_SPEED_SLOW, HALF_SECOND_DELAY);
            automatic_substate = PAPER_AWAIT_SPEED;
            break;

        case PAPER_AWAIT_SPEED:
            if (machine.servo_1->nominal_speed_reached) {
                detector_restart();
                automatic_substate = DETECT_AWAIT_SAMPLES;
            }
            break;

        case DETECT_AWAIT_SAMPLES:
            if (detector.sampling_done) {
                automatic_substate = DETECT_SCANNING;
            }
            break;

        case DETECT_SCANNING:
            if (monitor_data.sticker_dimensions_set &&
                machine.servo_1->servo_position - monitor_data.last_stop_position >= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
                    // Raise error
                }
            if (detect_mark()) {
                automatic_substate = DETECT_MARK_FOUND;
                // machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y) / machine.servo_1->scale;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Mark found, save positions and move to next step
        case DETECT_MARK_FOUND:
            set_text_10(display.F2_text, "Zn Najdeny");
            if (monitor_data.sticker_dimensions_set) {
                automatic_substate = CUT_STOP_AT_MARK;
            } else {
                if (monitor_data.first_mark_position == 0) {
                    automatic_substate = LEARN_FIRST_MARK;
                } else if (monitor_data.second_mark_position == 0) {
                    automatic_substate = LEARN_SECOND_MARK;
                } else if (monitor_data.third_mark_position == 0) {
                    automatic_substate = LEARN_THIRD_MARK; }
            }
            break;

        // Will save a first mark position and withouth stopping will continue to search the next mark
        case LEARN_FIRST_MARK:
            monitor_data.first_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET_Y;
            automatic_substate = PAPER_AWAIT_SPEED;
            break;
        
        // Will save a second mark position, stops and waits for user to confirm the sticker height
        case LEARN_SECOND_MARK:
            monitor_data.second_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            monitor_data.sticker_height = monitor_data.second_mark_position - monitor_data.first_mark_position;
            set_text_20(display.state_text_1, "Potvrd vysku nalepky");
            set_text_10(display.F2_text, "    Potvrd");
            if (machine.F2->state_raised) {
                set_text_20(display.state_text_1, "Automat");
                automatic_substate = PAPER_START_FEED;
            }
            break;

        // Will save a third mark position, stops and waits for user to confirm the mark distance
        case LEARN_THIRD_MARK:
            monitor_data.third_mark_position = detector.mark_position + SENSOR_KNIFE_OFFSET_Y;
            stop_knife_on_mark();
            monitor_data.mark_distance = monitor_data.third_mark_position - monitor_data.second_mark_position;
            set_text_20(display.state_text_1, "Potvrd vzdial. znac.");
            // set_text_20(display.state_text_2, "Potvrd vzdial. znac.");
            set_text_10(display.F2_text, "    Potvrd");
            if (machine.servo_1->positioning == IDLE && machine.F2->state_raised) {
                monitor_data.sticker_dimensions_set = true;
                automatic_substate = CUT_MOVE_TO_START;
            }
            break;

        case CUT_STOP_AT_MARK:
            stop_knife_between_marks();
            if (machine.servo_1->positioning == IDLE) {
                if (machine.servo_1->servo_position - monitor_data.last_stop_position <= monitor_data.sticker_height + STICKER_HEIGHT_TOLERNACE) {
                    // Raise error
                }
                automatic_substate = CUT_BEGIN_SEQUENCE;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Navigate cutting head to the cut position and perform the cut
        case CUT_MOVE_TO_START:
            set_text_20(display.state_text_1, "Automat");
            servo_goto_delayed(machine.servo_1, monitor_data.third_mark_position - monitor_data.mark_distance / 2.0, AUTOMAT_SPEED_MID, HALF_SECOND_DELAY);
            automatic_substate = CUT_AWAIT_POSITION;
            break;

        case CUT_AWAIT_POSITION:
            if (machine.servo_1->positioning == IDLE) {
                set_text_10(display.F2_text, " Rezat! :)");
                if (machine.F2->state_raised) {
                    automatic_substate = CUT_BEGIN_SEQUENCE;
                }
            }
            break;
            
        case CUT_BEGIN_SEQUENCE:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position - 50.0, AUTOMAT_SPEED_NORMAL, HALF_SECOND_DELAY);
                automatic_substate = CUT_REACH_EDGE;

            }
            break;

        case CUT_REACH_EDGE:
            if (machine.servo_0->positioning == IDLE) {
                // knife_down();
                servo_goto_delayed(machine.servo_0, machine.paper_begin_position - SENSOR_KNIFE_OFFSET_X + CUTTING_OVERLAP, AUTOMAT_SPEED_NORMAL, HALF_SECOND_DELAY);
                automatic_substate = CUT_RETURN_CENTER;
            }
            break;

        case CUT_RETURN_CENTER:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position - 50 , AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = CUT_FINISH_SEQUENCE;
            }
            break;

        case CUT_FINISH_SEQUENCE:
            if (machine.servo_0->positioning == IDLE) {
                // knife_down();
                servo_goto_delayed(machine.servo_0, machine.paper_end_position - SENSOR_KNIFE_OFFSET_X - CUTTING_OVERLAP, AUTOMAT_SPEED_NORMAL, HALF_SECOND_DELAY);
                automatic_substate = PREP_NEXT_CYCLE;
            }
            break;

        case PREP_NEXT_CYCLE:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, machine.paper_mark_position, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                servo_goto_delayed(machine.servo_1, (machine.servo_1->servo_position + (monitor_data.mark_distance / 2)), AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = PREP_NEW_DETECTION;
            }
            break;

        case PREP_NEW_DETECTION:
            if (machine.servo_0->positioning == IDLE && machine.servo_1->positioning == IDLE) {
                automatic_substate = PAPER_START_FEED;
            }
            break;

// ----------------------------------------------------------------------------------------------------------
// Should be used as stop case of the automatic mode
        case COMPLETE:
            automatic_substate = IDLE_A;
            break;
    }
}

void stop_knife_on_mark(void) {
    machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y) / machine.servo_1->scale;
}

void stop_knife_between_marks(void) {
    machine.servo_1->next_stop = (detector.mark_position + SENSOR_KNIFE_OFFSET_Y + (monitor_data.mark_distance / 2.0)) / machine.servo_1->scale;
}
