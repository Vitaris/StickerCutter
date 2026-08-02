#include <stdlib.h>
#include <string.h>
#include "hmi.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include "../machine/machine_controller.h"
#include "../rotary_encoder/rotary_encoder.h"
#include "../lcd/ant_lcd.h"

#define LINE_COUNT 4
#define LINE_LENGTH 20

typedef enum {
    WELCOME_SCREEN,
    STANDARD_SCREEN,
    INPUT_SCREEM,
    FAILURE_SCREEN
} screen_state_t;

typedef char lcd_line_t[LINE_LENGTH];
typedef char lcd_halfline[LINE_LENGTH / 2];

struct lcd_screen {
    lcd_line_t screen[LINE_COUNT];
};

struct hmi {
    screen_state_t screen_state;
    uint64_t start_us; 
    lcd_screen_t welcome;
    lcd_screen_t standard;
    lcd_screen_t screen_input;
    lcd_screen_t screen_actual;
};

void place_to_left(lcd_line_t line, const char *string) {
    snprintf(line, LINE_LENGTH, "%s", string);
}

void place_to_center(lcd_line_t line, const char *string) {
    size_t len = strlen(string);
    int pad = (len >= LINE_LENGTH) ? 0 : (LINE_LENGTH - (int)len) / 2;

    memset(line, ' ', pad);
    snprintf(line + pad, LINE_LENGTH - pad, "%s", string);
}

void place_to_right(lcd_line_t line, const char *string) {
    size_t len = strlen(string);
    int pad = (len >= LINE_LENGTH) ? 0 : LINE_LENGTH - 1 - (int)len;

    memset(line, ' ', pad);
    snprintf(line + pad, LINE_LENGTH - pad, "%s", string);
}

hmi_t* hmi_create() {
    hmi_t* hmi = calloc(1, sizeof(struct hmi));

    // Welcome Screen
    place_to_center(hmi->welcome.screen[1], "Sticker Cutter");
    place_to_right(hmi->welcome.screen[3], "V1.1");

    // Standard Screen

    
    hmi->start_us = time_us_64();
    return hmi;
}

void hmi_compute(hmi_t* hmi) {

    switch(hmi->screen_state) {
        case WELCOME_SCREEN:
			hmi->screen_actual = hmi->welcome;
            if (time_us_64() - hmi->start_us > 2000000) {
                hmi->screen_state = STANDARD_SCREEN;
            }
			break;
		
		case STANDARD_SCREEN:
            hmi->screen_actual = hmi->standard;

            // if (machine.test != true) {

            //     string2LCD(devices.lcd, 0, 0, machine.state_text_1);
            //     if (machine.machine_error) {
            //         string2LCD(devices.lcd, 0, 1, get_error_message());
            //     }
            //     else {
            //         string2LCD(devices.lcd, 0, 1, machine.state_text_2);
            //     }

            //     float2LCD(devices.lcd, 0, 2, 8, 2, servo_get_position(devices.servo_cutter));
            //     string2LCD(devices.lcd, 8, 2, "mm");
                
            //     float2LCD(devices.lcd, 10, 2, 8, 2, servo_get_position(devices.servo_feeder));
            //     string2LCD(devices.lcd, 18, 2, "mm");

            //     string2LCD(devices.lcd, 0, 3, machine.F1_text);
            //     string2LCD(devices.lcd, 10, 3, machine.F2_text);

            //     machine.position_cutter[0] = '\0';

            //     // float2LCD(devices.lcd, 0, 1, 8, 1, get_rotary_encoder_position(devices.encoder));
            // }

            break;
            
        case INPUT_SCREEM:
            break;
        
        case FAILURE_SCREEN:
            break;
    }

};
