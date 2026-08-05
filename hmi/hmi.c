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

#define BLINK_INTERVAL_US 500000ULL // 0.5 seconds (500 ms)
#define WELCOME_INTERVAL_US 2000000ULL // welcome screen duration

struct standard_screen standard_screen1 = {
        .status = "Mode:",
        .button_text_0 = "",
        .button_text_1 = ""
};

void place_to_left(lcd_line_t line, const char *string) {
    memset(line, ' ', LINE_LENGTH - 1);

    snprintf(line, LINE_LENGTH, "%s", string);
}

void place_to_center(lcd_line_t line, const char *string, uint8_t line_length) {
    memset(line, ' ', line_length - 1);
    size_t len = strlen(string);
    int pad = (len >= line_length) ? 0 : (line_length - (int)len) / 2;
    memcpy(line + pad, string, len);
    line[line_length] = '\0';
}

void place_to_right(lcd_line_t line, const char *string) {
    memset(line, ' ', LINE_LENGTH - 1);
    size_t len = strlen(string);
    int pad = (len >= LINE_LENGTH) ? 0 : LINE_LENGTH - 1 - (int)len;

    snprintf(line + pad, LINE_LENGTH - pad, "%s", string);
}

void clear_line(lcd_line_t line) {
    memset(line, ' ', LINE_LENGTH - 1);
    line[20] = '\0';
}

void clear_screen(lcd_screen_t *screen) {
    for (size_t i = 0; i < LINE_COUNT; i++) {
        clear_line(screen->line[i]);
    }
}

void merge_halfs_lines(lcd_line_t line, lcd_halfline_t half_first, lcd_halfline_t half_second) {
    snprintf(line, LINE_LENGTH, "%-*.*s%-*.*s", 
             LINE_LENGTH_HALF, LINE_LENGTH_HALF, half_first, 
             LINE_LENGTH_HALF, LINE_LENGTH_HALF, half_second);
}

void draw_screen(hmi_t* hmi) {
    for (size_t i = 0; i < LINE_COUNT; i++) {
        string2LCD(hmi->lcd, 0, i, hmi->actual_screen.line[i]);
    }
}

void switch_screen(hmi_t* hmi, screen_state_t screen_state) {
    hmi->screen_state = screen_state;
    clear_screen(&hmi->actual_screen);
}

void hmi_init(hmi_t* hmi, const lcd_config_t* config) {
    hard_assert(hmi != NULL);

    memset(hmi, 0, sizeof(hmi_t));
    clear_screen(&hmi->welcome_screen);
    clear_screen(&hmi->standard_screen);
    clear_screen(&hmi->input_screen);
    clear_screen(&hmi->failure_screen);
    clear_screen(&hmi->actual_screen);
    
    // Welcome Screen
    place_to_center(hmi->welcome_screen.line[1], "Sticker Cutter", LINE_LENGTH);
    place_to_right(hmi->welcome_screen.line[3], "V1.1");
    
    // Standard Screen
    
    // Initialize home switch
    hmi->home_switch = create_button(22);

    // Init Rotary encoder
    int offset_1 = pio_add_program(pio1, &quadrature_encoder_program);
    hmi->encoder = create_rotary_encoder(26, 28, 0, offset_1);
    
    hmi->lcd = lcd_create(config);
    hmi->start_us = time_us_64();

    
}

void hmi_compute(hmi_t* hmi) {
    rotary_encoder_compute(hmi->encoder);
    button_compute(hmi->home_switch);

    switch(hmi->screen_state) {
        case WELCOME_SCREEN:
			hmi->actual_screen = hmi->welcome_screen;
            if (time_us_64() - hmi->start_us > WELCOME_INTERVAL_US) {
                switch_screen(hmi, STANDARD_SCREEN);
            }
			break;
		
		case STANDARD_SCREEN:
            place_to_left(hmi->standard_screen.line[0], standard_screen1.status);
            merge_halfs_lines(hmi->standard_screen.line[3], standard_screen1.button_text_0, standard_screen1.button_text_1);
            hmi->actual_screen = hmi->standard_screen;

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
            
        case INPUT_SCREEN:
            break;
        
        case FAILURE_SCREEN:
            place_to_center(hmi->failure_screen.line[0], "* PORUCHA! *", LINE_LENGTH);
            uint64_t elapsed = time_us_64() - hmi->start_us;
            
            // Determines toggle state: 0 = visible, 1 = hidden/blank
            bool blink_off = (elapsed / BLINK_INTERVAL_US) % 2;
            
            // Start with base failure screen buffer
            hmi->actual_screen = hmi->failure_screen;
            
            if (blink_off) {
                clear_line(hmi->actual_screen.line[0]); 
            }
            if (time_us_64() - hmi->start_us > 10000000ULL) {
                switch_screen(hmi, STANDARD_SCREEN);
            }
            break;
    }

    draw_screen(hmi);

};
