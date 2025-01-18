#ifndef DISPLAY_20X4_H
#define DISPLAY_20X4_H

#include <stdbool.h>
#include <stdint.h>
#include "ant_lcd.h"

/**
 * @brief Display configuration for 20x4 LCD
 */
#define DISPLAY_COLS 20
#define DISPLAY_ROWS 4

/**
 * @brief Display GPIO pin assignments
 */
#define LCD_PIN_RS 11
#define LCD_PIN_RW 12
#define LCD_PIN_EN 13
#define LCD_PIN_D4 14
#define LCD_PIN_D5 15
#define LCD_PIN_D6 16
#define LCD_PIN_D7 17

/**
 * @brief Display controller structure
 * 
 * Manages a 20x4 character LCD display with text formatting capabilities
 */
typedef struct {
    lcd_t lcd;          // LCD controller handle
	
	// LCD Texts
	char state_text[21];
	char condition_text[10];
	char position_cutter[8];
	char position_feeder[8];
	char F1_text[11];
	char F2_text[11];
} display_20x4_t;

/**
 * @brief Global display controller instance
 * Accessible from other modules that include this header
 */
extern display_20x4_t display;

/**
 * @brief Creates and initializes a 20x4 LCD display
 */
void display_create(void);

/**
 * @brief Updates specific line on display
 * @param row Row number (0-3)
 * @param text Text to display
 */
void display_update_line(uint8_t row, const char* text);

/**
 * @brief Clears specific line on display
 * @param row Row number (0-3)
 */
void display_clear_line(uint8_t row);

/**
 * @brief Displays a number at specified position
 * @param row Row number (0-3)
 * @param col Column number (0-19)
 * @param value Number to display
 * @param precision Number of decimal places for floating point
 */
void display_number(uint8_t row, uint8_t col, float value, uint8_t precision);

/**
 * @brief Sets text with padding for display fields
 * @param LCD_text Destination text buffer
 * @param text Source text
 * @param len Maximum length of text
 */
void set_text(char LCD_text[], char text[], uint8_t len);

/**
 * @brief Sets 10-character text with padding
 * @param LCD_text Destination text buffer
 * @param text Source text
 */
void set_text_10(char LCD_text[], char text[]);

/**
 * @brief Sets 20-character text with padding
 * @param LCD_text Destination text buffer
 * @param text Source text
 */
void set_text_20(char LCD_text[], char text[]);

#endif // DISPLAY_20X4_H
