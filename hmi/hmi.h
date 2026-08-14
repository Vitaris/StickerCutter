#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "../lcd/ant_lcd.h"
#include "../servo_motor/button.h"
#include "../rotary_encoder/rotary_encoder.h"

#define LINE_COUNT 4
#define LINE_LENGTH 21 // 20 printable characters + '\0'
#define LINE_LENGTH_HALF 11 // 10 printable characters + '\0'

typedef enum {
    WELCOME_SCREEN,
    STANDARD_SCREEN,
    INPUT_SCREEN,
    FAILURE_SCREEN
} screen_state_t;

struct standard_screen {
    // char status[LINE_LENGTH];
    const char *status;
    const char *float_test;
    float pos_feeder;
    float pos_cutter;
    const char *button_text_line_0_F1;
    const char *button_text_line_1_F1;
    const char *button_text_line_0_F2;
    const char *button_text_line_1_F2;
};

extern struct standard_screen standard_screen1;


typedef char lcd_line_t[LINE_LENGTH];
typedef char lcd_halfline_t[LINE_LENGTH / 2];
typedef struct {
    lcd_line_t line[LINE_COUNT];
} lcd_screen_t;

typedef struct hmi {
    screen_state_t screen_state;
    uint64_t start_us; 
    lcd_screen_t welcome_screen;
    lcd_screen_t standard_screen;
    lcd_screen_t input_screen;
    lcd_screen_t failure_screen;
    lcd_screen_t actual_screen;
    lcd_t* lcd;
    button_t* home_switch;
    rotary_encoder_t* encoder;
} hmi_t;

// Public API: Starts Core 1 and sets up the display task
void hmi_init(hmi_t* hmi, const lcd_config_t* config);
void hmi_compute(hmi_t* hmi);
void switch_screen(hmi_t* hmi, screen_state_t screen_state);

#endif // HMI_H