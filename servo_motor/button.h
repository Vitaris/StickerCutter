#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/** 
 * @brief Structure representing a physical button
 * 
 * Contains the button state and debouncing information
 */
typedef struct button button_t;

/**
 * @brief Creates a new button instance
 * @param gpio_pin_num The GPIO pin number where the button is connected
 * @return Pointer to the newly created button instance
 */
button_t* create_button(const uint8_t gpio_pin_num);

/**
 * @brief Updates the button state and handles debouncing
 * @param button Pointer to the button instance
 */
void button_compute(button_t* const button);

/**
 * @brief Checks if the button was just pressed
 * @param button Pointer to the button instance
 * @return true if button was just pressed, false otherwise
 */
bool button_raised(button_t* const button);

/**
 * @brief Checks if the button was just released
 * @param button Pointer to the button instance
 * @return true if button was just released, false otherwise
 */
bool button_dropped(button_t* const button);

#endif
// End of Header file