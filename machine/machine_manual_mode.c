#include <stdio.h>
#include "pico/stdlib.h"

#include "../core1_main.h"
#include "../hmi/hmi.h"
#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "../rotary_encoder/rotary_encoder.h"

#define DESK_AREA_RIGHT -200.0
#define DESK_AREA_LEFT -1300.0

typedef enum {
    MANUAL_IDLE,      // Motors disabled, waiting for enable command
    MANUAL_READY,     // Motors enabled, ready for operations
} manual_substate_t;

typedef enum {
    HOMING_START,             // Preparing to start homing sequence
    HOMING_SCANNING,          // Moving servo while scanning for home position
    HOMING_FOUND,            // Home position detected, stopping motion
    HOMING_RETURN_TO_ZERO,   // Moving back to define zero position
    HOMING_FINISHED          // Homing sequence completed
} homing_substate_t;

typedef enum {
    PARAM_START,
    PARAM_STICKER_HEIGHT,
    PARAM_ROWS_TO_CUT,
    PARAM_PAPER_WIDTH,
    PARAM_FINSIHED,
} param_substate_t;

manual_substate_t manual_substate;
homing_substate_t homing_substate;
param_substate_t param_substate;

void activate_homing_state(void) {
    manual_substate = HOMING_START;
    machine_state = HOMING;
}

void activate_manual_state(void) {
    machine.paper_right_mark_position = 0.0;
    manual_substate = MANUAL_READY;
    machine_state = MANUAL;
    machine.enable = true;
}

void activate_param_state(void) {
    machine_state = PARAMS;
    param_substate = PARAM_START;
}

void servo_manual_movement(void) {
    servo_manual_handling(devices.servo_cutter, -1500, 20, MANUAL_SPEED_NORMAL, machine.homed);
    servo_manual_handling(devices.servo_feeder, 0, 0, MANUAL_SPEED_NORMAL, false);
}

void servo_manual_movement_slow(void) {
    servo_manual_handling(devices.servo_cutter, -1500, 20, MANUAL_SPEED_SLOW, machine.homed);
    servo_manual_handling(devices.servo_feeder, 0, 0, MANUAL_SPEED_SLOW, false);
}

int get_void_absence(void) {
    return 0;
}

int get_void_presence(void) {
    return 0;
}

void handle_manual_state(void) {
    // Update machine
    hmi_set_status(machine.homed ? "Manual" : "Manual - NO Home");
    hmi_set_left_button("Motory", machine.enable ? "vypnut" : "zapnut");

    // Always able to switch on/off servos
    if (button_raised(devices.F1)) {
        if (manual_substate == MANUAL_IDLE) {
            manual_substate = MANUAL_READY;
            machine.enable = true;
        }
        else {
            manual_substate = MANUAL_IDLE;
            machine.enable = false;
        }
    }

    // Handle state transitions
    switch(manual_substate) {
        case MANUAL_IDLE:
            hmi_set_right_button("", "");
            break;

        case MANUAL_READY:
            if (!machine.homed) {
                if (get_void_absence()) {
                    hmi_set_right_button("", "Home");
                    if (button_raised(devices.F2)) {
                        activate_homing_state();
                    }
                } 
                else {
                    // Do not allow homing if cutter head is out of cutting table
                    hmi_set_right_button("", "");
                }
            }
            else {
                if (is_paper_positions_set()) {
                    hmi_set_right_button("", "Automat");
                    if (button_raised(devices.F2)) {
                        activate_automatic_state();
                    }
                }
                else {
                    hmi_set_right_button("Nastav", "parametre");
                    if (button_raised(devices.F2)) {
                        activate_param_state();
                    }   
                }
            }
          
            servo_manual_movement();
            break;
        
    }
}

void handle_homing_state(void) {
    // Update machine
    hmi_set_status("HOMING");
    hmi_set_left_button("", "Stop");
    hmi_set_right_button("", "");

    if (button_raised(devices.F1)) {
        activate_manual_state();
    }

    // Handle state transitions
    switch(homing_substate) {
        case HOMING_START:
            if (servo_is_idle(devices.servo_cutter)) {
                servo_goto_delayed(devices.servo_cutter, 2000.0, 100.0, HALF_SECOND_DELAY);
                homing_substate = HOMING_SCANNING;
            }
            break;
        
        case HOMING_SCANNING:
            hmi_set_right_button("", "Hlada sa->");
            if (get_void_presence()) {
                homing_substate = HOMING_FOUND;
            }
            break;

        case HOMING_FOUND:
            if (servo_is_accelerating(devices.servo_cutter)) {
                servo_stop_positioning(devices.servo_cutter);
            }
            else if (servo_is_position_reached(devices.servo_cutter)) {
                homing_substate = HOMING_RETURN_TO_ZERO;
            }
            break;

        case HOMING_RETURN_TO_ZERO:
            if (servo_is_idle(devices.servo_cutter)) {
                servo_set_zero_position(devices.servo_cutter);
                servo_goto_delayed(devices.servo_cutter, -50.0, 100.0, HALF_SECOND_DELAY);
            }
            else if (servo_is_position_reached(devices.servo_cutter)) {
                homing_substate = HOMING_FINISHED;
            }
            break;

        case HOMING_FINISHED:
            machine.homed = true;
            activate_manual_state();
            break;
    }
}

void param_config_state(void) {
    switch(param_substate) {
        case PARAM_START:
            if (servo_get_position(devices.servo_feeder) - standard_screen.feeder_offset == 0.0) {
                hmi_set_right_button("", "Pokracuj");
                if (button_raised(devices.F2)) {
                    switch_screen(&hmi, INPUT_SCREEN);
                    param_substate = PARAM_STICKER_HEIGHT;
                }
            } else {
                hmi_set_right_button("Vynuluj", "feeder");
                if (button_raised(devices.F2)) {
                    machine.paper_right_mark_position = servo_get_position(devices.servo_cutter);
                    standard_screen.feeder_offset = servo_get_position(devices.servo_feeder);
                }
            }
            servo_manual_movement_slow();
            break;

        case PARAM_STICKER_HEIGHT:
            float sticker_height = rotary_encoder_get_position(devices.encoder);
            show_question(&hmi, "Nastav vysku nalepky:", sticker_height, "mm", "", "Exit", "", "Uloz vysku");
            if (button_raised(devices.F2)) {
                machine.sticker_height = sticker_height;
                param_substate = PARAM_ROWS_TO_CUT;
                rotary_encoder_set_position(devices.encoder, 1.0);
            }
            break;
            
        case PARAM_ROWS_TO_CUT:
            float rows = rotary_encoder_get_position(devices.encoder);
            show_question(&hmi, "Nastav pocet rezov:", rows, "ks", "", "Exit", "Uloz", "pocet");
            if (button_raised(devices.F2)) {
                machine.rows_to_cut = (size_t)rows;
                param_substate = PARAM_PAPER_WIDTH;
                rotary_encoder_reset_position(devices.encoder);
                rotary_encoder_set_scale(devices.encoder, -0.4);
                rotary_encoder_set_position(devices.encoder, 200.0);
            }
            break;

        case PARAM_PAPER_WIDTH:
            float width = rotary_encoder_get_position(devices.encoder);
            show_question(&hmi, "Nastav sirku papiera:", width, "ks", "", "Exit", "Uloz", "sirku");
            if (button_raised(devices.F2)) {
                machine.paper_width = width;
                switch_screen(&hmi, STANDARD_SCREEN);
                param_substate = PARAM_FINSIHED;
                rotary_encoder_reset_position(devices.encoder);
                rotary_encoder_set_scale(devices.encoder, -40.0);
            }
            break;

        case PARAM_FINSIHED:
            hmi_set_right_button("Zapnut", "Automat");
                if (button_raised(devices.F2)) {
                    servo_goto(devices.servo_cutter, -50, MANUAL_SPEED_FAST);
                    activate_automatic_state();
                }
            break;
    }
    // button F1
    hmi_set_left_button("", "Exit");
    if (button_raised(devices.F1)) {
        activate_manual_state();
    }
}
