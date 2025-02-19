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
    AUTOMATIC_IDLE,                       // Waiting for start command
    AUTOMATIC_MARK_GOTO,                 // Moving knife to initial mark position
    AUTOMATIC_MARK_GOTO_IN_PROGRESS,     // Waiting for knife to reach mark position
    AUTOMATIC_MARK_POS_REACHED,          // Knife has reached mark position
    AUTOMATIC_START_ROLLING_OUT_PAPER,   // Begin feeding paper
    AUTOMATIC_WAIT_FOR_CONSTANT_SPEED_OF_PAPER, // Waiting for stable paper feed
    AUTOMATIC_WAITING_FOR_SAMPLES,       // Collecting sensor readings
    AUTOMATIC_SCANNING_FOR_MARK,         // Actively looking for marks
    AUTOMATIC_MARK_FOUND,               // Mark detected
    AUTOMATIC_SAVE_FIRST_MARK,          // Recording position of first mark
    AUTOMATIC_SAVE_SECOND_MARK,         // Recording position of second mark
    AUTOMATIC_SAVE_THIRD_MARK,          // Recording position of third mark
    AUTOMATIC_REGULAR_CUT_STOP,         // Stop centered between two marks
    AUTOMATIC_GOTO_CUT_POSITION,        // Moving to cutting position
    AUTOMATIC_WAIT_FOR_CUT_POSITION,    // Waiting to reach cutting position
    AUTOMATIC_CUT_OPENING_SECTION,      // Starting the cut sequence
    AUTOMATIC_CUT_OPENING_SECTION2,     // Continuing cut sequence
    AUTOMATIC_CUT_RETURN_TO_MARK,       // Moving back to mark after first cut
    AUTOMATIC_CUT_REST_SECTION,         // Cutting remaining section
    AUTOMATIC_CUT_RETURN_TO_MARK_2,     // Final return to mark position
    AUTOMATIC_RETURN_NEW_DETECTION,     // Preparing for next mark detection
    AUTOMATIC_RETURN_TO_ZERO,           // Returning to home position
    AUTOMATIC_FINISHED                  // Cycle complete
} automatic_substate_t;


/**
 * @brief Current substate of the automatic operation mode
 * 
 * Keeps track of the current substate when machine is operating in automatic mode.
 * Variable is static to maintain state between function calls while limiting scope
 * to this file only.
 */
static automatic_substate_t automatic_substate;

/**
 * @brief Stores monitoring data for cut marks detection and tracking.
 * 
 * This static variable holds the state and information needed for monitoring
 * and processing fiducial marks in the automatic machine operation mode.
 */
static marks_monitor_t monitor_data;

/**
 * @brief Stops the knife operation when a cut mark is detected
 * 
 * This function handles the automatic stopping of the knife mechanism when
 * a cut mark is encountered during the scanning process.
 */
static void stop_knife_on_mark(void);

/**
 * @brief Stops the knife between marks during automatic operation
 * 
 * This function handles the stopping of the knife between marking points
 * in the automatic cutting mode. It ensures the knife stops at the correct
 * position between material marks.
 */
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
