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
    // machine.auto_substate = AUTO_IDLE;

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
    machine.params_ready = false;

    // Mark probe
    init_detector(0, &machine.servo_1->servo_position, &machine.machine_error, &machine.error_message);

    // Machine states
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
        case HOMING:    handle_homing_state(); break;
        case PARAMS:    handle_params_state(); break;
        case AUTOMAT:   handle_automatic_state(); break;
        case FAILURE:   handle_failure_state(); break;
    }

    // Handle error conditions and cutter state machine
    if (machine.machine_error) {
        machine.state = FAILURE;
    }

    // This should be last in the machine_compute function for safety reasons
    if (machine.state != AUTOMAT) {
        knife_up();
    }
    
}

void handle_failure_state(void) {
    set_text_20(display.state_text_1, "Porucha!");
    set_text_10(display.F1_text, "Potvrdit");
    set_text_10(display.F2_text, "");

    if (machine.F1->state_raised) {
        machine.state = MANUAL;
        set_text_20(machine.error_message, "OK");
        machine.machine_error = false;
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
