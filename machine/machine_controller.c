#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "mark_detector.h"
#include "../lcd/display_20x4.h"

machine_t machine;

void machine_init(void) {
    // Initialize machine state
    machine.state = MANUAL;
    machine.machine_condition = OK;
    machine.auto_substate = AUTO_IDLE;

    // Init buttons
    machine.F1 = create_button(5);
    machine.F2 = create_button(2);
    machine.Right = create_button(1);
    machine.Left = create_button(3);
    machine.In = create_button(4);
    machine.Out = create_button(0);

    // Init servos
    machine.machine_error = false;
    machine.enable = false;
    machine.homed = false;
    set_text_20(machine.error_message, "OK");

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    machine.servo_0 = servo_create("Cutter", offset, 0, ENC_0, PWM_0, SCALE_CUTTER, &machine.Right, &machine.Left, &machine.enable, &machine.machine_error, &machine.error_message);
    machine.servo_1 = servo_create("Feeder", offset, 1, ENC_1, PWM_1, SCALE_FEEDER, &machine.Out, &machine.In, &machine.enable, &machine.machine_error, &machine.error_message);

    // Knife
    gpio_init(KNIFE_OUTPUT_PIN);
    gpio_set_dir(KNIFE_OUTPUT_PIN, GPIO_OUT);

    // Cutter
    machine.cutter_state = CUTTER_IDLE;

    // Mark probe
    init_detector(0, &machine.servo_1->servo_position);

    // Machine states
    machine.paper_edge_position = 0.0;
    machine.mark_position = 0.0;
    activate_manual_state();
}

void machine_compute(void) {
    // Update I/O
    servo_compute(machine.servo_0);
    servo_compute(machine.servo_1);
    button_compute(machine.F1);
    button_compute(machine.F2);
    button_compute(machine.Right);
    button_compute(machine.Left);
    button_compute(machine.In);
    button_compute(machine.Out);
    detector_compute();

    // Handle main state machine
    switch(machine.state) {
        case MANUAL:    handle_manual_state(); break;
        case HOMING:   handle_homing_state(); break;
        case AUTOMAT:   handle_automatic_state(); break;
        case FAILURE:   handle_failure_state(); break;
    }

    // Handle error conditions and cutter state machine
    if (machine.machine_error) {
        machine.state = FAILURE;
    }
    handle_cutter_state();
}



void handle_automatic_state(void) {
    set_text_20(display.state_text, "Automat");
    set_text_10(display.F1_text, "Stop");

    if (machine.F1->state_raised) {
        machine.state = MANUAL;
        machine.cutter_state = STOP_CUTTING;
        return;
    }

    // Handle mark detection states
    switch(detector.state) {
        case DETECTOR_IDLE:
            set_text_10(display.F2_text, "Hladat zn.");
            if (machine.F2->state_raised && machine.servo_1->positioning == IDLE) {
                servo_goto_delayed(machine.servo_1, machine.servo_1->enc_position + 1000.0, 15.0, 500);
                detector.state = LINE_ACTIVATED;
            }
            break;

        case LINE_SCANNING:
            set_text_10(display.F2_text, "Hlada znak");
            break;

        case LINE_FOUND:
            set_text_10(display.F2_text, "Zn Najdeny");
            machine.servo_1->next_stop = (detector.stops[0] + 14.0) / machine.servo_1->scale;
            detector.state = LINE_WAITING;
            break;

        case LINE_WAITING:
            if (machine.F2->state_raised) {
                detector.state = IDLE;
            }
            break;
    }

    
            // Handle paper edge and mark detection
            // if (machine.paper_edge_position == 0.0) {
            //     set_text_10(display.F2_text, "Paper");
            //     if (machine.F2->state_raised) {
            //         machine.paper_edge_position = machine.servo_0->servo_position;
            //     }
            // }
            // else if (machine.mark_position == 0.0) {
            //     set_text_10(display.F2_text, "Mark");
            //     if (machine.F2->state_raised) {
            //         machine.mark_position = machine.servo_0->servo_position;
            //     }
            // }
            // else {
            //     set_text_10(display.F2_text, "Start");
            //     if (machine.F2->state_raised) {
            //         stop_positioning(machine.servo_0);
            //         stop_positioning(machine.servo_1);
            //         knife_up();
            //         machine.state = AUTOMAT;
            //     }
            // }

}

void handle_failure_state(void) {
    set_text_20(display.state_text, "Porucha!");
    set_text_10(display.F1_text, "Potvrdit");
    set_text_10(display.F2_text, "");

    if (machine.F1->state_raised) {
        machine.state = MANUAL;
        set_text_20(machine.error_message, "OK");
        machine.machine_error = false;
    }
}

void handle_cutter_state(void) {
    // Existing sticker_cut_compute function renamed and content moved here
    switch(machine.cutter_state) {
        case CUTTER_IDLE:
            knife_up();
            break;

        case CUTTER_REQUESTED:
            if (machine.servo_0->enc_position == 0.0) {
                machine.cutter_state = AT_HOME;
            } else {
                machine.cutter_state = TO_HOME;
            }
            break;

        case TO_HOME:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, 0.0, 10.0, 500);
            } 
            else if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = AT_HOME;
            }
            break;

        case AT_HOME:
            // Check if knife is above the mark
            if (get_next_stop(detector, machine.servo_1->enc_position) != machine.servo_1->enc_position) {
                raise_error("Znacka nenajdena");
            }
            else {
                machine.cutter_state = TO_PRECUT;
            }
            break;

        case TO_PRECUT:
            if (machine.servo_0->positioning == IDLE) {
                knife_up();
                servo_goto_delayed(machine.servo_0, PRECUT_POSITION, 4.0, 500);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = BACK_HOME;
                }
            }
            break;

        case BACK_HOME:
            if (machine.servo_0->positioning == IDLE) {
                knife_down();
                servo_goto_delayed(machine.servo_0, 0.0, 10.0, 500);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = CUT_TO_END;
                }
            }
            break;

        case CUT_TO_END:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, CUT_LENGTH, 4.0, 500);
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = FINAL_RETURN;
                }
            }
            break;

        case FINAL_RETURN:
            if (machine.servo_0->positioning == IDLE) {
                servo_goto_delayed(machine.servo_0, 0.0, 4.0, 500);
                knife_up();
            } else {
                if (machine.servo_0->positioning == POSITION_REACHED) {
                    machine.cutter_state = CUT_DONE;
                }
            }
            break;

        case CUT_DONE:
            machine.cutter_state = ROLL_OUT_PAPER;
            break; 
        
        case ROLL_OUT_PAPER:
            if (machine.servo_0->positioning == IDLE &&  machine.servo_1->positioning == IDLE) {
                servo_goto_delayed(machine.servo_1, machine.servo_1->enc_position + 5.0, 5.0, 500);
            }
            else if (machine.servo_1->positioning == POSITION_REACHED) {
                machine.cutter_state = CUTTER_IDLE;
            }
            break;

        case STOP_CUTTING:
            machine.servo_0->positioning = IDLE;
            machine.cutter_state = CUTTER_IDLE;
            break;
    }
}

void feeder_compute(void) {
    
}

void perform_sticker_cut(void) {
}

void knife_up(void) {
    gpio_put(KNIFE_OUTPUT_PIN, false);
}

void knife_down(void) {
    gpio_put(KNIFE_OUTPUT_PIN, true);
}

void raise_error(char text[]) {
    set_text_20(machine.error_message, text);
    machine.machine_error = true;
}
