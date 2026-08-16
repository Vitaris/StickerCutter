#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "../lcd/ant_lcd.h"
#include "../servo_motor/button.h"

#define LINE_COUNT 4
#define LINE_LENGTH_FULL 21 // 20 printable characters + '\0'
#define LINE_LENGTH_HALF 11 // 10 printable characters + '\0'

typedef enum {
    WELCOME_SCREEN,
    STANDARD_SCREEN,
    INPUT_SCREEN,
    FAILURE_SCREEN
} screen_state_t;

typedef char lcd_line_t[LINE_LENGTH_FULL];
typedef char lcd_halfline_t[LINE_LENGTH_HALF];
typedef struct {
    lcd_line_t line[LINE_COUNT];
} screen_buffer_t;

typedef struct {
    // char status[LINE_LENGTH];
    lcd_line_t status;
    lcd_halfline_t var_0;
    lcd_halfline_t var_1;
    float pos_feeder;
    float pos_cutter;
    float feeder_offset;
    lcd_halfline_t F1_0;
    lcd_halfline_t F1_1;
    lcd_halfline_t F2_0;
    lcd_halfline_t F2_1;
} screen_t;

struct input_screen {
    lcd_line_t status;
    lcd_line_t input;
    lcd_halfline_t F1_0;
    lcd_halfline_t F1_1;
    lcd_halfline_t F2_0;
    lcd_halfline_t F2_1;
};

extern screen_t standard_screen;
extern screen_t input_screen;

typedef struct hmi {
    screen_state_t screen_state;
    uint64_t start_us; 
    screen_buffer_t welcome_screen_buffer;
    screen_buffer_t standard_screen_buffer;
    screen_buffer_t input_screen_buffer;
    screen_buffer_t failure_screen_buffer;
    screen_buffer_t actual_screen_buffer;
    lcd_t* lcd;
} hmi_t;

// Public API: Starts Core 1 and sets up the display task
void hmi_init(hmi_t* hmi, const lcd_config_t* config);
void hmi_compute(hmi_t* hmi);
void switch_screen(hmi_t* hmi, screen_state_t screen_state);

// Standard-screen helpers: set the status line and the two soft-button
// labels (F1 = left, F2 = right), each spanning two 10-character lines.
void hmi_set_status(const char *status);
void hmi_set_left_button(const char *top, const char *bottom);
void hmi_set_right_button(const char *top, const char *bottom);
void hmi_clear_buttons(void);

void place_string_to_left(char *line, const char *string, size_t line_length);
void place_string_centered(char *line, const char *string, size_t line_length);
void place_string_to_right(char *line, const char *string, size_t line_length);
void place_float_to_left(char *line, float value, int decimals, size_t line_length);
void place_float_centered(char *line, float value, int decimals, size_t line_length);
void place_float_to_right(char *line, float value, int decimals, size_t line_length);
void place_float_with_unit_to_right(char *line, float value, int decimals, const char *unit, size_t line_length);

void show_question(hmi_t* hmi, const char *question, float var, const char *units, const char *f1_0, const char *f1_1, const char *f2_0, const char *f2_1);
void close_question(hmi_t* hmi);

#endif // HMI_H