#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ant_lcd.h"
#include "display_20x4.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "display_20x4.h"

#define DISPLAY_COLS 20
#define DISPLAY_ROWS 4
#define LCD_PIN_RS 10
#define LCD_PIN_RW 11
#define LCD_PIN_EN 12
#define LCD_PIN_D4 13
#define LCD_PIN_D5 14
#define LCD_PIN_D6 15
#define LCD_PIN_D7 16

struct display_20x4{
    lcd_t* lcd;
	
	// LCD Texts
	char state_text_1[21];
	char condition_text[10];
	char position_cutter[8];
	char position_feeder[8];
	char F1_text[11];
	char F2_text[11];
};

display_20x4_t display;

display_20x4_t* display_create(void) {
    display_20x4_t* display = calloc(1, sizeof(struct display_20x4));
    // Initialize LCD hardware
    display->lcd = lcd_create(
        LCD_PIN_RS,
        LCD_PIN_RW,
        LCD_PIN_EN,
        LCD_PIN_D4,
        LCD_PIN_D5,
        LCD_PIN_D6,
        LCD_PIN_D7,
        DISPLAY_COLS,
        DISPLAY_ROWS
    );

    // Clear display and home cursor
    clrscr(display->lcd);
}

void display_update_line(display_20x4_t* const display, uint8_t row, const char* text) {
    if (row >= DISPLAY_ROWS) return;
    string2LCD(display->lcd, 0, row, text);
}

void display_clear_line(display_20x4_t* const display, uint8_t row) {
    if (row >= DISPLAY_ROWS) return;
    char spaces[DISPLAY_COLS + 1];
    memset(spaces, ' ', DISPLAY_COLS);
    spaces[DISPLAY_COLS] = '\0';
    string2LCD(display->lcd, 0, row, spaces);
}

void display_number(display_20x4_t* const display, uint8_t row, uint8_t col, float value, uint8_t precision) {
    if (row >= DISPLAY_ROWS || col >= DISPLAY_COLS) return;
    float2LCD(display->lcd, col, row, 8, value);
}

void set_text(display_20x4_t* const display, char LCD_text[], char text[], uint8_t len) {
    bool fill_spaces = false;
    uint8_t i = 0;
    while (i < len) {
        if (text[i] == '\0') {
            fill_spaces = true;
        }
        LCD_text[i] = fill_spaces ? ' ' : text[i];
        i++;
    }
    LCD_text[len] = '\0';
}

void set_text_10(display_20x4_t* const display, char LCD_text[], char text[]) {
    set_text(display, LCD_text, text, 10);
}

void set_text_20(display_20x4_t* const display, char LCD_text[], char text[]) {
    set_text(display, LCD_text, text, 20);
}
