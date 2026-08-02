#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
#include "core1_main.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "quadrature_encoder.pio.h"
#include "machine/machine_controller.h"

#include "servo_motor/servo_motor.h"
#include "servo_motor/button.h"

// Timers
struct repeating_timer servo_timer;

bool servo_timer_callback(struct repeating_timer *t) {
    machine_compute();
    return true;
}

int main() {
    machine_init();

    // Initialize
    stdio_init_all();

    // Timer for servo control
    add_repeating_timer_ms(-1, servo_timer_callback, NULL, &servo_timer);

    // Launch hmi, lcd screen
    multicore_launch_core1(core1_main);

    // Initial wait 
    busy_wait_ms(500);

    while (1)
    {
        tight_loop_contents();
    }
}