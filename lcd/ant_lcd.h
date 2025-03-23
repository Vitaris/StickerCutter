#ifndef ANT_LCD_H
#define ANT_LCD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct lcd_controller lcd_t;

/**
 * @brief Creates a new LCD controller
 *
 * Creates a LCD controller and initializes it�s outputs
 *
 * @param lcd LCD controller handle
 * @param RS RS pin number
 * @param RW RW pin number
 * @param EN EN pin number
 * @param D4 D4 pin number
 * @param D5 D5 pin number
 * @param D6 D6 pin number
 * @param D7 D7 pin number
 * @param COL LCD columns
 * @param ROW LCD rows
 * 
 * @return returns a lcd_t controller handle
 * 
 * @note HD44780-compatible LCD in 4-bit mode
 */
lcd_t* lcd_create(uint32_t RS, uint32_t RW, uint32_t EN, uint32_t D4, uint32_t D5,
				uint32_t D6, uint32_t D7, uint32_t COL, uint32_t ROW);

/**
 * @brief Puts Int on LCD
 * 
 * @param lcd LCD controller handle
 * @param x X position
 * @param y Y position
 * @param max_length Max length of the number
 * @param number Int to be displayed
 * 
 */
void int2LCD(lcd_t* lcd, uint8_t x, uint8_t y, uint8_t max_length, int number);

/**
 * @brief Puts Float on LCD
 * 
 * @param lcd LCD controller handle
 * @param x X position
 * @param y Y position
 * @param max_length Max length of the number
 * @param number Float to be displayed
 * 
 */
void float2LCD(lcd_t* lcd, uint8_t x, uint8_t y, uint8_t max_length, float number);

/**
 * @brief Puts String on LCD
 * 
 * @param lcd LCD controller handle
 * @param x X position
 * @param y Y position
 * @param string String to be displayed
 * 
 */
void string2LCD(lcd_t* lcd, uint8_t x, uint8_t y, char string[]);

/**
 * @brief Clears the entire LCD screen
 * 
 * @param lcd Pointer to the LCD structure containing display information
 * 
 * @note This function will clear all characters and reset cursor to home position
 */
void clrscr(lcd_t* lcd);

#endif
// End of Header file
