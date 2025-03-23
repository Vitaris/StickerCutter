#ifndef DISPLAY_20X4_H
#define DISPLAY_20X4_H

#include <stdbool.h>
#include <stdint.h>
#include "ant_lcd.h"

typedef struct display_20x4 display_20x4_t;

/**
 * @brief Creates and initializes a 20x4 LCD display
 */
display_20x4_t* display_create(void);

/**
 * @brief Updates specific line on display
 * @param row Row number (0-3)
 * @param text Text to display
 */
void display_update_line(display_20x4_t* const display, uint8_t row, const char* text);

/**
 * @brief Clears specific line on display
 * @param row Row number (0-3)
 */
void display_clear_line(display_20x4_t* const display, uint8_t row);

/**
 * @brief Displays a number at specified position
 * @param row Row number (0-3)
 * @param col Column number (0-19)
 * @param value Number to display
 * @param precision Number of decimal places for floating point
 */
void display_number(display_20x4_t* const display, uint8_t row, uint8_t col, float value, uint8_t precision);

/**
 * @brief Sets text with padding for display fields
 * @param LCD_text Destination text buffer
 * @param text Source text
 * @param len Maximum length of text
 */
void set_text(display_20x4_t* const display, char LCD_text[], char text[], uint8_t len);

/**
 * @brief Sets 10-character text with padding
 * @param LCD_text Destination text buffer
 * @param text Source text
 */
void set_text_10(display_20x4_t* const display, char LCD_text[], char text[]);

/**
 * @brief Sets 20-character text with padding
 * @param LCD_text Destination text buffer
 * @param text Source text
 */
void set_text_20(display_20x4_t* const display, char LCD_text[], char text[]);

#endif // DISPLAY_20X4_H
