#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include <string.h>
#include <stdint.h>

// Add these missing includes
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "quadrature_encoder.pio.h"

#include "machine_controller.h"
#include "machine_manual_mode.h"
#include "machine_automatic_mode.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"
#include "../rotary_encoder/rotary_encoder.h"
#include "../core1_main.h"

// Physical constants
#define KNIFE_OUTPUT_PIN 17
#define SCALE_CUTTER 20.0
#define SCALE_FEEDER 6.4

machine_state_t machine_state;
devices_t devices;
machine_t machine;
manual_substate_t manual_substate;
homing_substate_t homing_substate;
param_substate_t param_substate;
automatic_substate_t automatic_substate;

static spin_lock_t *display_snapshot_lock;
static machine_display_snapshot_t display_snapshot;

static void publish_display_snapshot(void) {
    uint32_t saved_irq = spin_lock_blocking(display_snapshot_lock);
    display_snapshot.state = machine_state;
    display_snapshot.manual_substate = manual_substate;
    display_snapshot.homing_substate = homing_substate;
    display_snapshot.param_substate = param_substate;
    display_snapshot.automatic_substate = automatic_substate;
    display_snapshot.homed = machine.homed;
    display_snapshot.enabled = machine.enable;
    display_snapshot.paper_positions_set = is_paper_positions_set();
    display_snapshot.feeder_zeroed =
        fabsf(servo_get_position(devices.servo_feeder) - machine.feeder_offset) < 0.01f;
    display_snapshot.pos_cutter = servo_get_position(devices.servo_cutter);
    display_snapshot.pos_feeder = servo_get_position(devices.servo_feeder);
    display_snapshot.feeder_offset = machine.feeder_offset;
    display_snapshot.encoder_value = rotary_encoder_get_position(devices.encoder);
    display_snapshot.encoder_value_int = rotary_encoder_get_position_int(devices.encoder);
    display_snapshot.rows_to_cut = machine.rows_to_cut;
    display_snapshot.rows_cut_performed = machine.rows_cut_performed;
    memcpy(display_snapshot.error_message, machine.error_message, sizeof(display_snapshot.error_message));
    spin_unlock(display_snapshot_lock, saved_irq);
}

void machine_get_display_snapshot(machine_display_snapshot_t *snapshot) {
    hard_assert(snapshot != NULL);
    uint32_t saved_irq = spin_lock_blocking(display_snapshot_lock);
    *snapshot = display_snapshot;
    spin_unlock(display_snapshot_lock, saved_irq);
}

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
#define ENC_0 6
#define ENC_1 8

// First pin of PWM couple.
#define PWM_0 18
#define PWM_1 20

void machine_init(void) {
    display_snapshot_lock = spin_lock_instance(spin_lock_claim_unused(true));

    // Initialize machine state
    machine_state = MANUAL;

    // Create buttons
    devices.F1 = create_button(5);
    devices.F2 = create_button(2);
    devices.Right = create_button(1);
    devices.Left = create_button(3);
    devices.In = create_button(4);
    devices.Out = create_button(0);
    devices.home_switch = create_button(22);

    // Init Rotary encoder
    int offset_1 = pio_add_program(pio1, &quadrature_encoder_program);
    devices.encoder = create_rotary_encoder(26, 28, 0, offset_1);
    rotary_encoder_set_position(devices.encoder, 50.0);

    // Init servos
    machine.machine_error = false;
    machine.enable = false;

    strcpy(machine.error_message, "OK");
    machine.feeder_offset = 0.0f;

    // Init PIO
    int offset = pio_add_program(pio0, &quadrature_encoder_program);

    // Create servos
    devices.servo_cutter = servo_create("Cutter", offset, 0, ENC_0, PWM_0, SCALE_CUTTER, devices.Right, devices.Left, &machine.enable, &machine.machine_error, &machine.error_message);
    devices.servo_feeder = servo_create("Feeder", offset, 1, ENC_1, PWM_1, SCALE_FEEDER, devices.Out, devices.In, &machine.enable, &machine.machine_error, &machine.error_message);
   
    // Knife
    gpio_init(KNIFE_OUTPUT_PIN);
    gpio_set_dir(KNIFE_OUTPUT_PIN, GPIO_OUT);

    // Cutter
    machine.params_ready = false;
    machine.rows_to_cut = 0;
    machine.paper_width = 200.0;
    machine.cut_overlap = 10.0;

    // Force homed state on beginning
    machine.homed = true;

    // Machine states
    activate_manual_state();

    if (watchdog_caused_reboot()) {
        raise_error("watchdog reset");
    }

    publish_display_snapshot();
}

void machine_compute(void) {
    // Update I/devices
    servo_compute(devices.servo_cutter);
    servo_compute(devices.servo_feeder);
    button_compute(devices.F1);
    button_compute(devices.F2);
    button_compute(devices.Right);
    button_compute(devices.Left);
    button_compute(devices.In);
    button_compute(devices.Out);
    button_compute(devices.home_switch);
    rotary_encoder_compute(devices.encoder);
    
    // Handle error conditions and cutter state machine
    if (machine.machine_error && machine_state != FAILURE) {
        activate_failure_state();
    }

    // Handle main state machine
    switch(machine_state) {
        case MANUAL:    handle_manual_state(); break;
        case HOMING:    handle_homing_state(); break;
        case PARAMS:    param_config_state(); break;
        case AUTOMAT:   handle_automatic_state(); break;
        case FAILURE:   handle_failure_state(); break;
    }

    publish_display_snapshot();

    static uint32_t previous_core1_heartbeat;
    uint32_t heartbeat = __atomic_load_n(&core1_heartbeat, __ATOMIC_RELAXED);
    if (heartbeat != previous_core1_heartbeat) {
        previous_core1_heartbeat = heartbeat;
        watchdog_update();
    }
}

void activate_failure_state(void) {
    machine_state = FAILURE;
    machine.enable = false;
    knife_up();
}

void handle_failure_state(void) {
    if (button_raised(devices.F1)) {
        activate_manual_state();
        strcpy(machine.error_message, "OK");
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
    strcpy(machine.error_message, text);
    machine.machine_error = true;
}
