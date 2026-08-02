#include <stdlib.h>
#include <string.h>
#include "hmi/hmi.h"
#include "core1_main.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include "../machine/machine_controller.h"
#include "../rotary_encoder/rotary_encoder.h"
#include "../lcd/ant_lcd.h"

hmi_t* hmi;

// Internal state hidden from main
static struct repeating_timer lcd_refresh_timer;

// Private timer callback
static bool lcd_refresh_timer_callback(struct repeating_timer *t) {
    hmi_compute(hmi);
    return true;
}

// Private entry point for Core 1
void core1_main(void) {

    hmi = hmi_create();

    // Set up a repeating timer to refresh the LCD every 20 ms (50 Hz)
    add_repeating_timer_ms(-20, lcd_refresh_timer_callback, NULL, &lcd_refresh_timer);
    
    if (true) {
        tight_loop_contents(); 
    }
}
