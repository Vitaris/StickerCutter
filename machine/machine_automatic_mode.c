#include "pico/stdlib.h"
#include "../hmi/hmi.h"
#include "machine_controller.h"
#include "machine_automatic_mode.h"
#include "machine_manual_mode.h"
#include "../servo_motor/servo_motor.h"

static const float STICKER_HEIGHT_TOLERNACE = 10.0; // 10mm tolerance for sticker height
static bool is_cutting_from_right = true;

typedef enum {
    IDLE,
    
    // Generic paper movement and cutting states
    NAVIGATE_TO_LEAD_IN,
    MOVING_TO_LEAD_IN,
    LEAD_IN_READY,

    START_ENTRY_CUT,
    ENTRY_CUT_IN_PROGRESS,
    ENTRY_CUT_DONE,
    ENTRY_CUT_RETURN,
    RETURN_TO_LEAD_IN,
    PERFORM_REST_OF_CUT,
    REST_OF_CUT_IN_PROGRESS,
    REST_OF_CUT_DONE,

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
    servo_set_stop_position(devices.servo_feeder, get_mark_position() + SENSOR_KNIFE_OFFSET_Y);
}

void reset_paper_mark_positions(void) {
    machine.paper_right_mark_position = 0.0;
}

bool is_paper_positions_set(void) {
    return machine.paper_right_mark_position != 0.0;
}

void activate_automatic_state(void) {
    machine_state = AUTOMAT;
    automatic_substate = IDLE;
    is_cutting_from_right = true;
    machine.rows_cut_performed = 0;
}

void handle_automatic_state(void) {
    hmi_set_status("Automat");
    hmi_set_left_button("", "Stop");

    if (button_raised(devices.F1)) {
        activate_manual_state();
        return;
    }

    char buffer_0[32];
    char buffer_1[32];

    // Handle automatic state transitions
    switch(automatic_substate) {
        case IDLE:
            hmi_set_right_button("", "Start");
            knife_up();
            if (button_raised(devices.F2)) {
                is_cutting_from_right = true;
                machine.rows_cut_performed = 0;
                automatic_substate = NAVIGATE_TO_LEAD_IN;
            }
            break;

        case NAVIGATE_TO_LEAD_IN:
            snprintf(buffer_0, sizeof(buffer_0), "%zu", (size_t)machine.rows_to_cut);
            snprintf(buffer_1, sizeof(buffer_1), "%zu", (size_t)machine.rows_cut_performed);
            hmi_set_right_button(buffer_0, buffer_1);

            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                float start_pos = is_cutting_from_right ? 
                                  -machine.cut_overlap : 
                                  -machine.paper_width + machine.cut_overlap;
                                  
                servo_goto_delayed(devices.servo_cutter, start_pos, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                servo_goto_delayed(devices.servo_feeder, servo_get_position(devices.servo_feeder) + machine.sticker_height, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
                automatic_substate = MOVING_TO_LEAD_IN;
            }
            break;

        case MOVING_TO_LEAD_IN:
            if (servo_is_idle(devices.servo_cutter) && servo_is_idle(devices.servo_feeder)) {
                automatic_substate = LEAD_IN_READY;
            }
            break;
        
        case LEAD_IN_READY:
            automatic_substate = START_ENTRY_CUT;
            break;

        case START_ENTRY_CUT: {
            knife_down();
            float edge_pos = is_cutting_from_right ? 0.0 : -machine.paper_width;
            servo_goto_delayed(devices.servo_cutter, edge_pos, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = ENTRY_CUT_IN_PROGRESS;
            break;
        }

        case ENTRY_CUT_IN_PROGRESS:
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = ENTRY_CUT_DONE;
            }
            break;

        case ENTRY_CUT_DONE: {
            knife_up();
            float return_pos = is_cutting_from_right ? 
                               -machine.cut_overlap : 
                               -machine.paper_width + machine.cut_overlap;
                               
            servo_goto_delayed(devices.servo_cutter, return_pos, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = RETURN_TO_LEAD_IN;
            break;
        }

        case RETURN_TO_LEAD_IN:
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = PERFORM_REST_OF_CUT;
            }
            break;

        case PERFORM_REST_OF_CUT: {
            knife_down();
            float final_pos = is_cutting_from_right ? -machine.paper_width : 0.0;
            servo_goto_delayed(devices.servo_cutter, final_pos, AUTOMAT_SPEED_FAST, HALF_SECOND_DELAY);
            automatic_substate = REST_OF_CUT_IN_PROGRESS;
            break;
        }

        case REST_OF_CUT_IN_PROGRESS:
            if (servo_is_idle(devices.servo_cutter)) {
                automatic_substate = REST_OF_CUT_DONE;
            }
            break;

        case REST_OF_CUT_DONE:
            machine.rows_cut_performed++;
            is_cutting_from_right = !is_cutting_from_right;

            if (machine.rows_cut_performed >= machine.rows_to_cut) {
                automatic_substate = COMPLETE;
            } else {
                automatic_substate = NAVIGATE_TO_LEAD_IN;
            }
            break;

        case COMPLETE:
            knife_up();
            hmi_set_right_button("", "Hotovo!");
            if (button_raised(devices.F2)) {
                automatic_substate = IDLE;
            }
            break;

        case HOME_RETURN:
            automatic_substate = IDLE;
            break;
    }
}