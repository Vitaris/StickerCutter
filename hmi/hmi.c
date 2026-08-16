#include <stdlib.h>
#include <string.h>
#include "hmi.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include "../machine/machine_controller.h"
#include "../lcd/ant_lcd.h"

#ifndef GIT_VERSION
#define GIT_VERSION "unknown"
#endif

#define BLINK_INTERVAL_US 500000ULL // 0.5 seconds (500 ms)
#define WELCOME_INTERVAL_US 2000000ULL // welcome screen duration

struct standard_screen standard_screen1 = {
        .status = "Mode:",
        .float_test = "",
        .pos_cutter = 0.0,
        .pos_feeder = 0.0,
        .feeder_offset = 0.0,
        .button_text_line_0_F1 = "",
        .button_text_line_1_F1 = "",
        .button_text_line_0_F2 = "",
        .button_text_line_1_F2 = ""
};

struct input_screen input_screen1 = {
        .status = "Mode:",
        .sticker_row_height = 0.0,
        .button_text_line_0_F1 = "",
        .button_text_line_1_F1 = "",
        .button_text_line_0_F2 = "",
        .button_text_line_1_F2 = ""
};

void place_string_to_left(lcd_line_t line, const char *string) {
    snprintf(line, LINE_LENGTH, "%-*s", LINE_LENGTH - 1, string);
}

void place_string_centered(lcd_line_t line, const char *string) {
    int len = strlen(string);
    int max_len = LINE_LENGTH - 1;

    if (len >= max_len) {
        snprintf(line, LINE_LENGTH, "%s", string);
        return;
    }

    // Calculate how many spaces belong on the left and right
    int left_pad = (max_len - len) / 2;
    int right_pad = max_len - len - left_pad;

    snprintf(line, LINE_LENGTH, "%*s%s%*s", left_pad, "", string, right_pad, "");
}

void place_string_to_right(lcd_line_t line, const char *string) {
    snprintf(line, LINE_LENGTH, "%*s", LINE_LENGTH - 1, string);
}

void place_float_to_left(lcd_line_t line, float value, int decimals, size_t line_length) {
    snprintf(line, line_length, "%-*.*f", line_length - 1, decimals, value);
}

void place_float_centered(lcd_line_t line, float value, int decimals, size_t line_length) {
    char temp_buf[line_length];
    snprintf(temp_buf, line_length, "%.*f", decimals, value);
    place_string_centered(line, temp_buf);
}

void place_float_to_right(lcd_line_t line, float value, int decimals, size_t line_length) {
    snprintf(line, line_length, "%*.*f", line_length - 1, decimals, value);
}

void place_float_with_unit_to_right(lcd_line_t line, float value, int decimals, const char *unit, size_t line_length) {
    char temp_buf[line_length];
    snprintf(temp_buf, line_length, "%.*f%s", decimals, value, unit);
    snprintf(line, line_length, "%*s", line_length - 1, temp_buf);
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
    clrscr(hmi->lcd);
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
    place_string_centered(hmi->welcome_screen.line[1], "Sticker Cutter");
    place_string_to_right(hmi->welcome_screen.line[3], GIT_VERSION);
    
    // Standard Screen
       
    // LCD screen
    hmi->lcd = lcd_create(config);
    hmi->start_us = time_us_64();
}

void hmi_compute(hmi_t* hmi) {
    

    switch(hmi->screen_state) {
        case WELCOME_SCREEN:
			hmi->actual_screen = hmi->welcome_screen;
            if (time_us_64() - hmi->start_us > WELCOME_INTERVAL_US) {
                switch_screen(hmi, STANDARD_SCREEN);
            }
			break;
		
		case STANDARD_SCREEN:
            place_string_to_left(hmi->standard_screen.line[0], standard_screen1.status);
            merge_halfs_lines(hmi->standard_screen.line[2], standard_screen1.button_text_line_0_F1, standard_screen1.button_text_line_0_F2);
            merge_halfs_lines(hmi->standard_screen.line[3], standard_screen1.button_text_line_1_F1, standard_screen1.button_text_line_1_F2);
            // place_float_to_left(hmi->standard_screen.line[2], 125.05, 2);
            lcd_halfline_t number_0;
            lcd_halfline_t number_1;
            place_float_with_unit_to_right(number_0, standard_screen1.pos_cutter, 2, "mm", LINE_LENGTH_HALF);
            place_float_with_unit_to_right(number_1, standard_screen1.pos_feeder - standard_screen1.feeder_offset, 2, "mm", LINE_LENGTH_HALF);
            merge_halfs_lines(hmi->standard_screen.line[1], number_0, number_1);
            // place_float_with_unit_to_right(hmi->standard_screen.line[2], standard_screen1.pos_cutter, 2, "mm");
            // float_test
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
            place_string_to_left(hmi->input_screen.line[0], "Nastav vysku nalepky:");
            place_string_to_left(hmi->input_screen.line[1], "");
            place_float_with_unit_to_right(hmi->input_screen.line[2], input_screen1.sticker_row_height, 2, "mm    ", LINE_LENGTH);

            place_string_to_left(hmi->input_screen.line[3], "Exit      Uloz vysku");
            hmi->actual_screen = hmi->input_screen;
            break;
        
        case FAILURE_SCREEN:
            place_string_centered(hmi->failure_screen.line[0], "* PORUCHA! *");
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
