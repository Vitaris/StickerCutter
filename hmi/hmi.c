#include "hmi.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include <stdio.h>
#include "../machine/machine_controller.h"
#include "../rotary_encoder/rotary_encoder.h"

// Internal state hidden from main
static struct repeating_timer lcd_refresh_timer;
static volatile bool lcd_refresh = false;

// Private timer callback
static bool lcd_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    return true;
}

// Private entry point for Core 1
static void hmi_core1_entry(void) {
    // Intro Screen
    string2LCD(devices.lcd, 3, 1, "Sticker Cutter");
    string2LCD(devices.lcd, 16, 3, "V1.1");
    busy_wait_ms(2000);
    
    int i = 0;
    // Set up a repeating timer to refresh the LCD every 20 ms (50 Hz)
    add_repeating_timer_ms(-20, lcd_refresh_timer_callback, NULL, &lcd_refresh_timer);
    
    while (1)
    {
        if (lcd_refresh == true)
        { 
            string2LCD(devices.lcd, 0, 0, machine.state_text_1);
            // if (machine.machine_error) {
            //     string2LCD(devices.lcd, 0, 1, get_error_message());
            // }
            // else {
            //     string2LCD(devices.lcd, 0, 1, machine.state_text_2);
            // }

            float2LCD(devices.lcd, 0, 2, 8, 2, servo_get_position(devices.servo_cutter));
            string2LCD(devices.lcd, 8, 2, "mm");
            
            float2LCD(devices.lcd, 10, 2, 8, 2, servo_get_position(devices.servo_feeder));
            string2LCD(devices.lcd, 18, 2, "mm");

            string2LCD(devices.lcd, 0, 3, machine.F1_text);
            string2LCD(devices.lcd, 10, 3, machine.F2_text);

            float2LCD(devices.lcd, 0, 1, 8, 1, get_rotary_encoder_position(devices.encoder));

            lcd_refresh = false;
        }
        tight_loop_contents(); 
    }
}

void hmi_init(void) {
    multicore_launch_core1(hmi_core1_entry);
}

void hmi_request_refresh(void) {
    lcd_refresh = true;
}