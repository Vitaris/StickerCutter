#ifndef HMI_H
#define HMI_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "../lcd/ant_lcd.h"
#include "../machine/machine_controller.h"

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

typedef struct hmi {
    screen_state_t screen_state;
    uint64_t start_us;
    screen_buffer_t welcome_screen_buffer;
    screen_buffer_t actual_screen_buffer;
    screen_buffer_t displayed_screen_buffer;
    lcd_t* lcd;
} hmi_t;

void hmi_init(hmi_t* hmi, const lcd_config_t* config);
void hmi_compute(hmi_t* hmi);

#endif // HMI_H