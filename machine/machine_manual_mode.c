#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

// Define and initialize the global variable
manual_substate_t manual_substate;
homing_substate_t homing_substate;
params_substate_t params_substate;

void activate_manual_state() {
    manual_substate = MANUAL_READY;
    machine.state = MANUAL;
    machine.enable = true;
}

void activate_homing_state() {
    manual_substate = HOMING_IDLE;
    machine.state = HOMING;
}

void activate_params_state() {
    params_substate = PARAMS_IDLE;
    machine.state = PARAMS;
}

void handle_manual_state() {
    // Update display
    set_text_20(display.state_text, machine.homed ? "Manual" : "Manual - NO Home");
    set_text_10(display.F1_text, machine.enable ? "Mot->OFF" : "Mot->ON");

    // Handle state transitions
    switch(manual_substate) {
        case MANUAL_IDLE:
            set_text_10(display.F2_text, "");
            if (machine.F1->state_raised) {
                machine.enable = true;
                manual_substate = MANUAL_READY;
            }
            break;

        case MANUAL_READY:
            if (!machine.homed) {
                if (get_void_absence(detector)) {
                    set_text_10(display.F2_text, "    Home");
                    if (machine.F2->state_raised) {
                        activate_homing_state();
                    }
                } 
                else {
                    // Do not allow homing if cutter head is out of cutting table
                    set_text_10(display.F2_text, "");
                }
            }
            else if (!machine.params_ready) {
                set_text_10(display.F2_text, " Parametre");
                if (machine.F2->state_raised) {
                    activate_params_state();
                }
            }
            else if (machine.homed && machine.params_ready) {
                set_text_10(display.F2_text, "   Automat");
                if (machine.F2->state_raised) {
                    activate_automatic_state();
                }
            }

            // Always able to switch to idle
            if (machine.F1->state_raised) {
                machine.enable = false;
                manual_substate = MANUAL_IDLE;
                break;
            }
           
            servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
            servo_manual_handling(machine.servo_1, 0, 0, false);

            break;
    }
}

void handle_homing_state() {
    // Update display
    set_text_20(display.state_text, "HOMING");

    set_text_10(display.F1_text, "Stop");
    if (machine.F1->state_raised) {
        activate_manual_state();
    }

    // Handle state transitions
    switch(homing_substate) {
        case HOMING_IDLE:
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                homing_substate = HOMING_START;
            }
            break;

        case HOMING_START:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 2000.0, 100.0, 500);
                homing_substate = HOMING_SCANNING;
            }
            break;
        
        case HOMING_SCANNING:
            set_text_10(display.F2_text, "Hlada sa->");
            if (get_void_presence(detector)) {
                homing_substate = HOMING_FOUND;
            }
            break;

        case HOMING_FOUND:
            if (machine.servo_0->positioning == ACCELERATING) {
                stop_positioning(machine.servo_0);
            }
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                homing_substate = HOMING_RETURN_TO_ZERO;
            }
            break;

        case HOMING_RETURN_TO_ZERO:
            if (machine.servo_0->positioning == IDLE) {
                machine.servo_0->set_zero = true;
                servo_goto_delayed(machine.servo_0, -50.0, 100.0, 500);
            }
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                homing_substate = HOMING_FINISHED;
            }
            break;

        case HOMING_FINISHED:
            machine.homed = true;
            activate_manual_state();
            break;
    }
}

void handle_params_state() {
    set_text_10(display.F1_text, "Stop");

    if (machine.F1->state_raised) {
        machine.state = MANUAL;
        return;
    }

    switch(params_substate) {
        case PARAMS_IDLE:
            set_text_20(display.state_text, "Nastavenie papiera");
            set_text_10(display.F2_text, "Start");
            if (machine.F2->state_raised) {
                params_substate = PARAMS_PAPER_BEGIN;
            }
            break;

        case PARAMS_PAPER_BEGIN:
            set_text_20(display.state_text, "Nastav zaciatok pap.");
            set_text_10(display.F2_text, "Zaciatok");
            if (machine.F2->state_raised) {
                machine.paper_begin_position = machine.servo_0->servo_position;
                params_substate = PARAMS_PAPER_MARK;
            }
            break;

        case PARAMS_PAPER_MARK:
            set_text_20(display.state_text, "Nastav znacku");
            set_text_10(display.F2_text, "Znacka");
            if (machine.F2->state_raised) {
                machine.paper_mark_position = machine.servo_0->servo_position;
                params_substate = PARAMS_PAPER_END;
            }
            break;

        case PARAMS_PAPER_END:
            set_text_20(display.state_text, "Nastav koniec pap.");
            set_text_10(display.F2_text, "Koniec");

            if (machine.F2->state_raised) {
                machine.paper_end_position = machine.servo_0->servo_position;
                machine.params_ready = true;
                activate_manual_state();
            }
            break;
    }

    // Handle servo control to set paper parameters
    servo_manual_handling(machine.servo_0, -1500, 20, machine.homed);
}
