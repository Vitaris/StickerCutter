#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "machine/machine_controller.h"

#include "lcd/display_20x4.h"
#include "servo_motor/servo_motor.h"
#include "servo_motor/button.h"

// Timers
uint64_t old_cycle_time = 0;
struct repeating_timer servo_timer;
struct repeating_timer LCD_refresh_timer;

// Display controller
// display_20x4_t* display;

bool blink_500ms;
bool lcd_refresh;

// Machine states
machine_t* cutter;

void core1_entry() {
    display_create();

    while (1) {
        if (lcd_refresh == true) { 
            display_update_line(0, display.state_text);
            display_update_line(1, cutter->error_message);
            
            display_number(1, 10, cutter->detector->current_reflectivity, 0);

            display_number(2, 0, cutter->servo_0->servo_position, 2);
            display_update_line(2, "      mm");
            display_number(2, 10, cutter->servo_1->servo_position, 2);
            display_update_line(2, "      mm");

            display_update_line(3, display.F1_text);
            display_number(3, 10, 0, 0);
            display_update_line(3, display.F2_text);

            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {
    machine_compute();
    return true;
}

bool LCD_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    return true;
}

int main() {
    machine_init();

    // Initialize
    stdio_init_all();

    // Timer for servo control
    add_repeating_timer_ms(-1, servo_timer_callback, NULL, &servo_timer);

    // 100ms LCD refresh timer
    add_repeating_timer_ms(100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
    // Launch core1
    multicore_launch_core1(core1_entry);

    // Initial wait 
    busy_wait_ms(500);

    while (1)
    {
        tight_loop_contents();
    }
}