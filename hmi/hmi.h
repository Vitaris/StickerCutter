#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "../lcd/ant_lcd.h"
#include "../servo_motor/button.h"

#define LINE_COUNT 4
#define LINE_LENGTH 21 // 20 printable characters + '\0'
#define LINE_LENGTH_HALF 11 // 10 printable characters + '\0'

typedef enum {
    WELCOME_SCREEN,
    STANDARD_SCREEN,
    INPUT_SCREEN,
    FAILURE_SCREEN
} screen_state_t;

typedef char lcd_line_t[LINE_LENGTH];
typedef char lcd_halfline_t[LINE_LENGTH / 2];
typedef struct {
    lcd_line_t line[LINE_COUNT];
} lcd_screen_t;

struct standard_screen {
    // char status[LINE_LENGTH];
    const char *status;
    const char *float_test;
    float pos_feeder;
    float pos_cutter;
    float feeder_offset;
    lcd_halfline_t F1_0;
    lcd_halfline_t F1_1;
    lcd_halfline_t F2_0;
    lcd_halfline_t F2_1;
};

struct input_screen {
    // char status[LINE_LENGTH];
    lcd_line_t status;
    float sticker_row_height;
    lcd_halfline_t F1_0;
    lcd_halfline_t F1_1;
    lcd_halfline_t F2_0;
    lcd_halfline_t F2_1;
};

extern struct standard_screen standard_screen1;
extern struct input_screen input_screen1;

typedef struct hmi {
    screen_state_t screen_state;
    uint64_t start_us; 
    lcd_screen_t welcome_screen;
    lcd_screen_t standard_screen;
    lcd_screen_t input_screen;
    lcd_screen_t failure_screen;
    lcd_screen_t actual_screen;
    lcd_t* lcd;
} hmi_t;

// Public API: Starts Core 1 and sets up the display task
void hmi_init(hmi_t* hmi, const lcd_config_t* config);
void hmi_compute(hmi_t* hmi);
void switch_screen(hmi_t* hmi, screen_state_t screen_state);

void place_string_to_left(char *line, const char *string, size_t line_length);
void place_string_centered(char *line, const char *string, size_t line_length);
void place_string_to_right(char *line, const char *string, size_t line_length);
void place_float_to_left(char *line, float value, int decimals, size_t line_length);
void place_float_centered(char *line, float value, int decimals, size_t line_length);
void place_float_to_right(char *line, float value, int decimals, size_t line_length);
void place_float_with_unit_to_right(char *line, float value, int decimals, const char *unit, size_t line_length);

#endif // HMI_H