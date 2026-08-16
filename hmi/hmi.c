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

screen_t standard_screen = {
        .status = "Mode:",
        .pos_cutter = 0.0,
        .pos_feeder = 0.0,
        .feeder_offset = 0.0,
        .F1_0 = "",
        .F1_1 = "",
        .F2_0 = "",
        .F2_1 = ""
};

void place_string_to_left(char *line, const char *string, size_t line_length) {
    snprintf(line, line_length, "%-*s", line_length - 1, string);
}

void place_string_centered(char *line, const char *string, size_t line_length) {
    int len = strlen(string);
    int max_len = line_length - 1;

    if (len >= max_len) {
        snprintf(line, line_length, "%s", string);
        return;
    }

    // Calculate how many spaces belong on the left and right
    int left_pad = (max_len - len) / 2;
    int right_pad = max_len - len - left_pad;

    snprintf(line, line_length, "%*s%s%*s", left_pad, "", string, right_pad, "");
}

void place_string_to_right(char *line, const char *string, size_t line_length) {
    snprintf(line, line_length, "%*s", line_length - 1, string);
}

void place_float_to_left(char *line, float value, int decimals, size_t line_length) {
    snprintf(line, line_length, "%-*.*f", line_length - 1, decimals, value);
}

void place_float_centered(char *line, float value, int decimals, size_t line_length) {
    char temp_buf[line_length];
    snprintf(temp_buf, line_length, "%.*f", decimals, value);
    place_string_centered(line, temp_buf, line_length);
}

void place_float_to_right(char *line, float value, int decimals, size_t line_length) {
    snprintf(line, line_length, "%*.*f", line_length - 1, decimals, value);
}

void place_float_with_unit_to_right(char *line, float value, int decimals, const char *unit, size_t line_length) {
    char temp_buf[line_length];
    snprintf(temp_buf, line_length, "%.*f%s", decimals, value, unit);
    snprintf(line, line_length, "%*s", line_length - 1, temp_buf);
}

void hmi_set_status(const char *status) {
    place_string_to_left(standard_screen.status, status, LINE_LENGTH_FULL);
}

void hmi_set_left_button(const char *top, const char *bottom) {
    place_string_to_left(standard_screen.F1_0, top, LINE_LENGTH_HALF);
    place_string_to_left(standard_screen.F1_1, bottom, LINE_LENGTH_HALF);
}

void hmi_set_right_button(const char *top, const char *bottom) {
    place_string_to_right(standard_screen.F2_0, top, LINE_LENGTH_HALF);
    place_string_to_right(standard_screen.F2_1, bottom, LINE_LENGTH_HALF);
}

void hmi_clear_buttons(void) {
    hmi_set_left_button("", "");
    hmi_set_right_button("", "");
}

void clear_line(lcd_line_t line) {
    memset(line, ' ', LINE_LENGTH_FULL - 1);
    line[20] = '\0';
}

void clear_screen(screen_buffer_t *screen) {
    for (size_t i = 0; i < LINE_COUNT; i++) {
        clear_line(screen->line[i]);
    }
}

void merge_halfs_lines(lcd_line_t line, lcd_halfline_t half_first, lcd_halfline_t half_second) {
    snprintf(line, LINE_LENGTH_FULL, "%-*.*s%-*.*s", 
             LINE_LENGTH_HALF - 1, LINE_LENGTH_HALF - 1, half_first, 
             LINE_LENGTH_HALF - 1, LINE_LENGTH_HALF - 1, half_second);
}

void show_question(hmi_t* hmi, const char *question, float var, const char *units, const char *f1_0, const char *f1_1, const char *f2_0, const char *f2_1) {
    place_string_to_left(hmi->input_screen_buffer.line[0], question, LINE_LENGTH_FULL);
    place_float_centered(hmi->input_screen_buffer.line[1], var, 2, LINE_LENGTH_FULL);

    // button F1
    lcd_halfline_t line_half_0;
    lcd_halfline_t line_half_1;
    place_string_to_left(line_half_0, f1_0, LINE_LENGTH_HALF);
    place_string_to_right(line_half_1, f2_0, LINE_LENGTH_HALF);
    merge_halfs_lines(hmi->input_screen_buffer.line[2], line_half_0, line_half_1);

    // button F2
    place_string_to_left(line_half_0, f1_1, LINE_LENGTH_HALF);
    place_string_to_right(line_half_1, f2_1, LINE_LENGTH_HALF);
    merge_halfs_lines(hmi->input_screen_buffer.line[3], line_half_0, line_half_1);
}

void draw_screen(hmi_t* hmi) {
    for (size_t i = 0; i < LINE_COUNT; i++) {
        string2LCD(hmi->lcd, 0, i, hmi->actual_screen_buffer.line[i]);
    }
}

void switch_screen(hmi_t* hmi, screen_state_t screen_state) {
    hmi->screen_state = screen_state;
    clear_screen(&hmi->actual_screen_buffer);
    clrscr(hmi->lcd);
}

void hmi_init(hmi_t* hmi, const lcd_config_t* config) {
    hard_assert(hmi != NULL);

    memset(hmi, 0, sizeof(hmi_t));
    clear_screen(&hmi->welcome_screen_buffer);
    clear_screen(&hmi->standard_screen_buffer);
    clear_screen(&hmi->input_screen_buffer);
    clear_screen(&hmi->failure_screen_buffer);
    clear_screen(&hmi->actual_screen_buffer);
    
    // Welcome Screen
    place_string_centered(hmi->welcome_screen_buffer.line[1], "Sticker Cutter", LINE_LENGTH_FULL);
    place_string_to_right(hmi->welcome_screen_buffer.line[3], GIT_VERSION, LINE_LENGTH_FULL);
    
    // Standard Screen
       
    // LCD screen
    hmi->lcd = lcd_create(config);
    hmi->start_us = time_us_64();
}

void hmi_compute(hmi_t* hmi) {
    

    switch(hmi->screen_state) {
        case WELCOME_SCREEN:
			hmi->actual_screen_buffer = hmi->welcome_screen_buffer;
            if (time_us_64() - hmi->start_us > WELCOME_INTERVAL_US) {
                switch_screen(hmi, STANDARD_SCREEN);
            }
			break;
		
		case STANDARD_SCREEN:
            place_string_to_left(hmi->standard_screen_buffer.line[0], standard_screen.status, LINE_LENGTH_FULL);
            merge_halfs_lines(hmi->standard_screen_buffer.line[2], standard_screen.F1_0, standard_screen.F2_0);
            merge_halfs_lines(hmi->standard_screen_buffer.line[3], standard_screen.F1_1, standard_screen.F2_1);
            // place_float_to_left(hmi->standard_screen_buffer.line[2], 125.05, 2);
            lcd_halfline_t number_0;
            lcd_halfline_t number_1;
            place_float_with_unit_to_right(number_0, standard_screen.pos_cutter, 2, "mm", LINE_LENGTH_HALF);
            place_float_with_unit_to_right(number_1, standard_screen.pos_feeder - standard_screen.feeder_offset, 2, "mm", LINE_LENGTH_HALF);
            merge_halfs_lines(hmi->standard_screen_buffer.line[1], number_0, number_1);
            // place_float_with_unit_to_right(hmi->standard_screen_buffer.line[2], standard_screen.pos_cutter, 2, "mm");
            // float_test
            hmi->actual_screen_buffer = hmi->standard_screen_buffer;

            break;
            
        case INPUT_SCREEN:
            hmi->actual_screen_buffer = hmi->input_screen_buffer;
            break;
        
        case FAILURE_SCREEN:
            place_string_centered(hmi->failure_screen_buffer.line[0], "* PORUCHA! *", LINE_LENGTH_FULL);
            place_string_centered(hmi->failure_screen_buffer.line[2], get_error_message(), LINE_LENGTH_FULL);
            place_string_to_left(hmi->failure_screen_buffer.line[3], "Potvrdit", LINE_LENGTH_FULL);
            uint64_t elapsed = time_us_64() - hmi->start_us;
            
            // Determines toggle state: 0 = visible, 1 = hidden/blank
            bool blink_off = (elapsed / BLINK_INTERVAL_US) % 2;
            
            // Start with base failure screen buffer
            hmi->actual_screen_buffer = hmi->failure_screen_buffer;
            
            if (blink_off) {
                clear_line(hmi->actual_screen_buffer.line[0]); 
            }
            // Exit is driven by the machine controller (operator confirmation).
            break;
    }

    draw_screen(hmi);

};
