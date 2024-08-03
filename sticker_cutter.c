#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "machine/machine_controller.h"

#include "lcd/ant_lcd.h"
#include "servo_motor/servo_motor.h"
#include "servo_motor/button.h"

// Timers
uint64_t old_cycle_time = 0;
struct repeating_timer servo_timer;
struct repeating_timer LCD_refresh_timer;

// LCD
lcd_t lcd;

bool blink_500ms;
bool lcd_refresh;

// Machine states
machine_t cutter;

void core1_entry() {

    // LCD
    lcd = lcd_create(10, 11, 12, 13, 14, 15, 16, 16, 4);
    while (1)
    {
        if (lcd_refresh == true)
        { 
            string2LCD(lcd, 0, 0, cutter->state_text);
            string2LCD(lcd, 0, 1, cutter->error_message);

            float2LCD(lcd, 0, 2, 8, cutter->servo_0->current_pos);
            string2LCD(lcd, 8, 2, "mm");
            
            float2LCD(lcd, 10, 2, 8, cutter->servo_1->current_pos);
            string2LCD(lcd, 18, 2, "mm");

            string2LCD(lcd, 0, 3, cutter->F1_text);
            string2LCD(lcd, 10, 3, cutter->F2_text);

            int2LCD(lcd, 10, 1, 10, cutter->detector->result);

            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {
    machine_compute(cutter);
    return true;
}

bool LCD_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    return true;
}

int main() {
    cutter = create_machine();

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