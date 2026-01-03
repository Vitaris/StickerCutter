#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
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
struct repeating_timer LCD_refresh_timer;

bool lcd_refresh;

void core1_entry() {
    // Intro Screen
    string2LCD(devices.lcd, 3, 1, "Sticker Cutter");
    string2LCD(devices.lcd, 16, 3, "V1.1");
    busy_wait_ms(2000);

    while (1)
    {
        if (lcd_refresh == true)
        { 
            string2LCD(devices.lcd, 0, 0, machine.state_text_1);
            if (machine.machine_error) {
                string2LCD(devices.lcd, 0, 1, get_error_message());
            }
            else {
                string2LCD(devices.lcd, 0, 1, machine.state_text_2);
            }

            float2LCD(devices.lcd, 0, 2, 8, servo_get_position(devices.servo_cutter));
            string2LCD(devices.lcd, 8, 2, "mm");
            
            float2LCD(devices.lcd, 10, 2, 8, servo_get_position(devices.servo_feeder));
            string2LCD(devices.lcd, 18, 2, "mm");

            string2LCD(devices.lcd, 0, 3, machine.F1_text);
            string2LCD(devices.lcd, 10, 3, machine.F2_text);

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
    add_repeating_timer_ms(-100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
    // Launch core1
    multicore_launch_core1(core1_entry);

    // Initial wait 
    busy_wait_ms(500);

    while (1)
    {
        tight_loop_contents();
    }
}