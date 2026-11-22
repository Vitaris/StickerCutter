#include <math.h>
#include "pico/stdlib.h"

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "../rotary_encoder/rotary_encoder.h"

#define DESK_AREA_RIGHT -200.0
#define DESK_AREA_LEFT -1300.0

void activate_homing_state(void) {
    manual_substate = HOMING_START;
    machine_state = HOMING;
}

void activate_manual_state(void) {
    machine.paper_right_mark_position = 0.0;
    manual_substate = MANUAL_READY;
    machine_state = MANUAL;
    servo_stop_positioning(devices.servo_cutter);
    servo_stop_positioning(devices.servo_feeder);
    machine.enable = true;
    knife_up();
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
            break;

        case MANUAL_READY:
            if (!machine.homed) {
                if (get_void_absence()) {
                    if (button_raised(devices.F2)) {
                        activate_homing_state();
                    }
                } 
            }
            else {
                if (is_paper_positions_set()) {
                    if (button_raised(devices.F2)) {
                        activate_automatic_state();
                    }
                }
                else {
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
            if (fabsf(servo_get_position(devices.servo_feeder) - machine.feeder_offset) < 0.01f) {
                if (button_raised(devices.F2)) {
                    param_substate = PARAM_STICKER_HEIGHT;
                    rotary_encoder_set_scale(devices.encoder, -1.0);
                }
            } else {
                if (button_raised(devices.F2)) {
                    machine.paper_right_mark_position = servo_get_position(devices.servo_cutter);
                    machine.feeder_offset = servo_get_position(devices.servo_feeder);
                }
            }
            servo_manual_movement_slow();
            break;

        case PARAM_STICKER_HEIGHT:
            float sticker_height = rotary_encoder_get_position(devices.encoder);
            if (button_raised(devices.F2)) {
                machine.sticker_height = sticker_height;
                param_substate = PARAM_ROWS_TO_CUT;
                rotary_encoder_set_position(devices.encoder, 1.0);
                rotary_encoder_set_scale(devices.encoder, -1.0);
            }
            break;
            
        case PARAM_ROWS_TO_CUT:
            int rows = rotary_encoder_get_position_int(devices.encoder);
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
            if (button_raised(devices.F2)) {
                machine.paper_width = width;
                param_substate = PARAM_FINSIHED;
                rotary_encoder_reset_position(devices.encoder);
                rotary_encoder_set_scale(devices.encoder, -40.0);
            }
            break;

        case PARAM_FINSIHED:
                if (button_raised(devices.F2)) {
                    servo_goto(devices.servo_cutter, -50, MANUAL_SPEED_FAST);
                    activate_automatic_state();
                }
            break;
    }
    if (button_raised(devices.F1)) {
        activate_manual_state();
    }
}
