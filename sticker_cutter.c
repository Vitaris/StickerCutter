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
        // LCD Layout
        //      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16   
        //   0         State                 
        //   1      OK / Error Message
        //   2      Position Cutter    |     Position Feeder
        //   3          F1             |            F2
        // 
        
        // LCD Layout
        //      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15   
        //   0  *  *  *  *  *  *  *  *| *  *  *  *  *  *  *  *
        //   1  *  *  *  *  *  *  *  *| *  *  *  *  *  *  *  *
        //   2  *  *  *  *  *  *  *  *| *  *  *  *  *  *  *  *
        //   3  *  *  *  *  *  *  *  *| *  *  *  *  *  *  *  *
        // 

        if (lcd_refresh == true)
        {   
            string2LCD(lcd, 0, 0, cutter->state_text);
            string2LCD(lcd, 1, 0, cutter->condition_text);
            string2LCD(lcd, 2, 0, cutter->position_cutter);
            string2LCD(lcd, 2, 7, cutter->position_feeder);
            string2LCD(lcd, 3, 0, cutter->F1_text);
            string2LCD(lcd, 3, 7, cutter->F2_text);
            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {

    // Get current time 
	float current_cycle_time = (float)(time_us_64() - old_cycle_time) * 1e-6;
	old_cycle_time = time_us_64();

    machine_compute(cutter, current_cycle_time);

    // if (servo_error) {
    //     enable = false;
    // }

    // // if (F1->state_raised) {
    // //     if (mode == MANUAL) {
    // //         mode = AUTOMAT;
    // //     }
    // //     else {
    // //         mode = MANUAL;
    // //     }
    // // }

    // if (F2->state_raised) {
    //     servo_error = false;
    //     enable = true;
    //     auto_man = false;
    //     strcpy(*error_message_0, "OK                  ");
    // }

    // button_compute(F1);
    // button_compute(F2);
    // button_compute(Right);
    // button_compute(Left);
    // button_compute(In);
    // button_compute(Out);
 
    // servo_compute(servo_0, current_cycle_time);
    // servo_compute(servo_1, current_cycle_time);


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
    add_repeating_timer_ms(1, servo_timer_callback, NULL, &servo_timer);

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