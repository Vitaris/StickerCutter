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

display_20x4_t display;

void display_create(void) {
    // Initialize LCD hardware
    display.lcd = lcd_create(
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
    clrscr(display.lcd);
}

void display_update_line(uint8_t row, const char* text) {
    if (row >= DISPLAY_ROWS) return;
    string2LCD(display.lcd, 0, row, text);
}

void display_clear_line(uint8_t row) {
    if (row >= DISPLAY_ROWS) return;
    char spaces[DISPLAY_COLS + 1];
    memset(spaces, ' ', DISPLAY_COLS);
    spaces[DISPLAY_COLS] = '\0';
    string2LCD(display.lcd, 0, row, spaces);
}

void display_number(uint8_t row, uint8_t col, float value, uint8_t precision) {
    if (row >= DISPLAY_ROWS || col >= DISPLAY_COLS) return;
    float2LCD(display.lcd, col, row, 8, value);
}

void set_text(char LCD_text[], char text[], uint8_t len) {
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

void set_text_10(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 10);
}

void set_text_20(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 20);
}
