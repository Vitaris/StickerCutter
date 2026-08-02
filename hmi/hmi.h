#ifndef HMI_H
#define HMI_H

#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "../lcd/ant_lcd.h"

#define LINE_COUNT 4
#define LINE_LENGTH 21 // 20 printable characters + '\0'

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

#endif // HMI_H