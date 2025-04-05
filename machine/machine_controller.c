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

// Physical constants
static const uint KNIFE_OUTPUT_PIN = 17;
static const float SCALE_CUTTER = 20.0;
static const float SCALE_FEEDER = 6.4;

// LCD display configuration
#define DISPLAY_COLS 20
#define DISPLAY_ROWS 4
#define LCD_PIN_RS 10
#define LCD_PIN_RW 11
#define LCD_PIN_EN 12
#define LCD_PIN_D4 13
#define LCD_PIN_D5 14
#define LCD_PIN_D6 15
#define LCD_PIN_D7 16

machine_state_t machine_state;
devices_t devices;
machine_t machine;

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
static const uint ENC_0 = 6;
static const uint ENC_1 = 8;

// First pin of PWM couple.
static const uint PWM_0 = 18;
static const uint PWM_1 = 20;

void machine_init(void) {
    // Initialize machine state
    machine_state = MANUAL;

    // Create buttons
    devices.F1 = create_button(5);
    devices.F2 = create_button(2);
    devices.Right = create_button(1);
    devices.Left = create_button(3);
    devices.In = create_button(4);
    devices.Out = create_button(0);

    // Init servos
    machine.machine_error = false;
    machine.enable = false;
    machine.homed = false;
    set_text_20(machine.error_message, "OK");

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    // Create servos
    devices.servo_0 = servo_create("Cutter", offset, 0, ENC_0, PWM_0, SCALE_CUTTER, devices.Right, devices.Left, &machine.enable, &machine.machine_error, &machine.error_message);
    devices.servo_1 = servo_create("Feeder", offset, 1, ENC_1, PWM_1, SCALE_FEEDER, devices.Out, devices.In, &machine.enable, &machine.machine_error, &machine.error_message);

    // Create lcd
    devices.lcd = lcd_create(
        LCD_PIN_RS,
        LCD_PIN_RW,
        LCD_PIN_EN,
        LCD_PIN_D4,
        LCD_PIN_D5,
        LCD_PIN_D6,
        LCD_PIN_D7,
        DISPLAY_COLS,
        DISPLAY_ROWS
    );

    // Debug
    machine.paper_right_mark_position = -70.0;
    machine.paper_left_mark_position = -1387.0;

    // Knife
    gpio_init(KNIFE_OUTPUT_PIN);
    gpio_set_dir(KNIFE_OUTPUT_PIN, GPIO_OUT);

    // Cutter
    machine.params_ready = false;

    // Mark probe
    init_detector(0, servo_get_position_pointer(devices.servo_1), &machine.machine_error, &machine.error_message);

    // Machine states
    activate_manual_state();
}

void machine_compute(void) {
    // Update I/devices
    servo_compute(devices.servo_0);
    servo_compute(devices.servo_1);
    button_compute(devices.F1);
    button_compute(devices.F2);
    button_compute(devices.Right);
    button_compute(devices.Left);
    button_compute(devices.In);
    button_compute(devices.Out);
    detector_compute();
    
    // Handle error conditions and cutter state machine
    if (machine.machine_error) {
        activate_failure_state();
    }

    // Handle main state machine
    switch(machine_state) {
        case MANUAL:    handle_manual_state(); break;
        case HOMING:    handle_homing_state(); break;
        case AUTOMAT:   handle_automatic_state(); break;
        case FAILURE:   handle_failure_state(); break;
    }
}

void activate_failure_state(void) {
    machine_state = FAILURE;
    machine.enable = false;
    knife_up();
}

void handle_failure_state(void) {
    set_text_20(machine.state_text_1, "Porucha!");
    set_text_10(machine.F1_text, "Potvrdit");
    set_text_10(machine.F2_text, "");

    if (button_raised(devices.F1)) {
        activate_manual_state();
        set_text_20(machine.error_message, "OK");
        machine.machine_error = false;
    }
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

char* get_error_message(void) {
    return machine.error_message;
}

void set_text(char LCD_text[], char text[], uint8_t len) {
    bool fill_spaces = false;
    uint8_t i = 0;
    while (i < len) {
        if (text[i] == '\0') {
            fill_spaces = true;
        }
        LCD_text[i] = fill_spaces ? ' ' : text[i];
        i++;
    }
    LCD_text[len] = '\0';
}

void set_text_10(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 10);
}

void set_text_20(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 20);
}
