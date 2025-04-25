#include <stdio.h>
#include "pico/stdlib.h"

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"

#define DESK_AREA_RIGHT -200.0
#define DESK_AREA_LEFT -1300.0

typedef enum {
    MANUAL_IDLE,      // Motors disabled, waiting for enable command
    MANUAL_READY,     // Motors enabled, ready for operations
    MANUAL_SET_RIGHT
} manual_substate_t;

typedef enum {
    HOMING_START,             // Preparing to start homing sequence
    HOMING_SCANNING,          // Moving servo while scanning for home position
    HOMING_FOUND,            // Home position detected, stopping motion
    HOMING_RETURN_TO_ZERO,   // Moving back to define zero position
    HOMING_FINISHED          // Homing sequence completed
} homing_substate_t;

manual_substate_t manual_substate;
homing_substate_t homing_substate;

void activate_homing_state(void) {
    manual_substate = HOMING_START;
    machine_state = HOMING;
}

void activate_manual_state(void) {
    manual_substate = MANUAL_READY;
    machine_state = MANUAL;
    machine.enable = true;
}

void servo_manual_movement(void) {
    servo_manual_handling(devices.servo_0, -1500, 20, MANUAL_SPEED_NORMAL, machine.homed);
    servo_manual_handling(devices.servo_1, 0, 0, MANUAL_SPEED_NORMAL, false);
}

void servo_manual_movement_slow(void) {
    servo_manual_handling(devices.servo_0, -1500, 20, MANUAL_SPEED_SLOW, machine.homed);
    servo_manual_handling(devices.servo_1, 0, 0, MANUAL_SPEED_SLOW, false);
}

void handle_manual_state(void) {
    // Update machine
    set_text_20(machine.state_text_1, machine.homed ? "Manual" : "Manual - NO Home");
    set_text_20(machine.state_text_2, "");
    set_text_10(machine.F1_text, machine.enable ? "Mot->OFF" : "Mot->ON");

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
            set_text_10(machine.F2_text, "");
            break;

        case MANUAL_READY:
            if (!machine.homed) {
                if (get_void_absence()) {
                    set_text_10(machine.F2_text, "    Home");
                    if (button_raised(devices.F2)) {
                        activate_homing_state();
                    }
                } 
                else {
                    // Do not allow homing if cutter head is out of cutting table
                    set_text_10(machine.F2_text, "");
                }
            }
            else {
                if (is_paper_positions_set()) {
                    set_text_10(machine.F2_text, "   Automat");
                    if (button_raised(devices.F2)) {
                        activate_automatic_state();
                    }
                }
                else {
                    set_text_10(machine.F2_text, "Set znaky");
                    if (button_raised(devices.F2)) {
                        manual_substate = MANUAL_SET_RIGHT;
                    }   
                }
            }
          
            servo_manual_movement();
            break;
        
        case MANUAL_SET_RIGHT:
            if (servo_get_position(devices.servo_0) > DESK_AREA_RIGHT) {
                set_text_10(machine.F2_text, "Prava znck");
                servo_manual_movement_slow();
                if (button_raised(devices.F2)) {
                    machine.paper_right_mark_position = servo_get_position(devices.servo_0);
                    manual_substate = MANUAL_READY;
                }
            }
            else {
                set_text_10(machine.F2_text, "Pravy kraj");
                if (button_raised(devices.F2)) {
                    servo_goto(devices.servo_0, -50, MANUAL_SPEED_FAST);
                }
            }
            break;
    }
}

void handle_homing_state(void) {
    // Update machine
    set_text_20(machine.state_text_1, "HOMING");
    set_text_20(machine.state_text_2, "");

    set_text_10(machine.F1_text, "Stop");
    if (button_raised(devices.F1)) {
        activate_manual_state();
    }

    // Handle state transitions
    switch(homing_substate) {
        case HOMING_START:
            if (servo_is_idle(devices.servo_0)) {
                servo_goto_delayed(devices.servo_0, 2000.0, 100.0, HALF_SECOND_DELAY);
                homing_substate = HOMING_SCANNING;
            }
            break;
        
        case HOMING_SCANNING:
            set_text_10(machine.F2_text, "Hlada sa->");
            if (get_void_presence()) {
                homing_substate = HOMING_FOUND;
            }
            break;

        case HOMING_FOUND:
            if (servo_is_accelerating(devices.servo_0)) {
                servo_stop_positioning(devices.servo_0);
            }
            else if (servo_is_position_reached(devices.servo_0)) {
                homing_substate = HOMING_RETURN_TO_ZERO;
            }
            break;

        case HOMING_RETURN_TO_ZERO:
            if (servo_is_idle(devices.servo_0)) {
                servo_set_zero_position(devices.servo_0);
                servo_goto_delayed(devices.servo_0, -50.0, 100.0, HALF_SECOND_DELAY);
            }
            else if (servo_is_position_reached(devices.servo_0)) {
                homing_substate = HOMING_FINISHED;
            }
            break;

        case HOMING_FINISHED:
            machine.homed = true;
            activate_manual_state();
            break;
    }
}
