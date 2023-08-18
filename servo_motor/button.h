#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define BUTTON_DEBOUNCE_TIME 1000

typedef struct button {
	uint8_t gpio_pin_num;
	bool state;
	bool state_raised;
	bool state_dropped;
	bool state_3s_pressed;

	time_t time_last_changed;
	time_t time_pressed;
	time_t time_released;
} button_t;

#ifdef	__cplusplus
extern "C" {
#endif

	/**
		 * @brief 
		 *
		 * @param
		 * 
		 * @return Slice number of pwm module
		 */
	void create_button(button_t* button, uint8_t gpio_pin_num);

	void button_compute(button_t* button);

#ifdef	__cplusplus
}
#endif

#endif
// End of Header file