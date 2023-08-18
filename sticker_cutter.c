#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

#include "lcd/ant_lcd.h"
#include "servo_motor/servo_motor.h"
#include "servo_motor/button.h"
#include "machine/machine_controller.h"



// Servo Motors
uint64_t old_cycle_time = 0;

// Timers
struct repeating_timer servo_timer;
struct repeating_timer blink_timer;
struct repeating_timer LCD_refresh_timer;

// Machine controller
machine_t machine;

// LCD
struct lcd_controller lcd_ctrl;
lcd_t lcd;

bool blink_500ms;
bool lcd_refresh;

// Debug
#define MAN false
#define AUTO true
bool mode = MAN;
bool allways_true = true;


void core1_entry() {

    // LCD
    lcd = lcd_create(&lcd_ctrl, 10, 11, 12, 13, 14, 15, 16, 16, 4);
    string2LCD(lcd, 0, 0, "Mode:");

    while (1)
    {
        if (lcd_refresh == true)
        {
            if (mode == MAN && false)
            {
            string2LCD(lcd, 6, 0, "MAN");
            }

            float2LCD(lcd, 2, 1, 6, machine.test_servo_0.current_pos);
            // float2LCD(lcd, 2, 1, 6, 0.0);
            string2LCD(lcd, 0, 1, "P:");
            float2LCD(lcd, 2, 2, 6, machine.test_servo_1.current_pos);
            // float2LCD(lcd, 2, 2, 6, 0.0);
            string2LCD(lcd, 0, 2, "P:");

            // float2LCD(lcd, 14, 1, 6, machine.test_servo_0.current_vel);
            // float2LCD(lcd, 14, 1, 6, 0.0);
            string2LCD(lcd, 12, 1, "S:");
            // float2LCD(lcd, 14, 2, 6, machine.test_servo_1.current_vel);
            float2LCD(lcd, 14, 2, 6, machine.ctrldata_detector.positions[0]);
            // float2LCD(lcd, 14, 2, 6, 0.0);
            string2LCD(lcd, 12, 2, "S:");

            // int2LCD(lcd, 14, 0, 6, machine.test_servo_0.enc_old);
            int2LCD(lcd, 14, 0, 6, machine.ctrldata_detector.result);
            int2LCD(lcd, 14, 1, 6, machine.ctrldata_detector.average);
            // int2LCD(lcd, 14, 0, 6, 0.0);

            // Machine state
            string2LCD(lcd, 6, 0, machine.state_text);
            string2LCD(lcd, 10, 0, machine.condition_text);
            string2LCD(lcd, 0, 3, machine.F1_text);
            string2LCD(lcd, 10, 3, machine.F2_text);

            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {

    // Get current time 
	float current_cycle_time = (float)(time_us_64() - old_cycle_time) * 1e-6;
	old_cycle_time = time_us_64();

    // servo_compute(test_servo_0, current_cycle_time);
    // servo_compute(test_servo_1, current_cycle_time);

    // Machine
    machine_compute(&machine, current_cycle_time);

    return true;
}

bool blink_timer_callback(struct repeating_timer *t) {
    if (blink_500ms == true)
    {
        blink_500ms = false;
    }
    else
    {
        blink_500ms = true;
    }
    
    return true;
}

bool LCD_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    
    return true;
}

int main() {
    // Init machine controller
    create_machine(&machine);

    // Initialize all of the present standard stdio types that are linked into the binary. 
    stdio_init_all();

    // Timer for servo control
    add_repeating_timer_ms(1, servo_timer_callback, NULL, &servo_timer);

    // 1s blink timer for lcd
    add_repeating_timer_ms(500, blink_timer_callback, NULL, &blink_timer);

    // 100ms LCD refresh timer
    add_repeating_timer_ms(100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
    // Launch core1
    multicore_launch_core1(core1_entry);

    while (1)
    {
        tight_loop_contents();
    }
}