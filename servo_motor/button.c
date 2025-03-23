#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdlib.h>
#include "button.h"

#define BUTTON_DEBOUNCE_TIME 1000

struct button {
	uint8_t gpio_pin_num;
	bool state;
	bool state_raised;
	bool state_dropped;

	time_t time_pressed;
	time_t time_released;
};

button_t* create_button(const uint8_t gpio_pin_num)
{
    button_t* button = calloc(1, sizeof(struct button));

    // Set button GPIO to be an input
    gpio_init(gpio_pin_num);
    gpio_set_dir(gpio_pin_num, GPIO_IN);

    button->gpio_pin_num = gpio_pin_num;

    return button;
}

void button_compute(button_t* const button)
{
    // Read the button state
    bool current_state = gpio_get(button->gpio_pin_num);
    int currenttime = time_us_64();

    // Positive change detection (off -> on)
    if (current_state != button->state && button->state == false)
    {
        // Debounce
        if (time_us_64() - button->time_pressed > BUTTON_DEBOUNCE_TIME)
        {
            button->state_raised = true;
            button->time_pressed = time_us_64();
        }
    }
    else 
    {
        button->state_raised = false;
    }

    // Negative change detection (on -> off)
    if (current_state != button->state && button->state == true)
    {
        // Debounce
        if (time_us_64() - button->time_released > BUTTON_DEBOUNCE_TIME)
        {
            button->state_dropped = true;
            button->time_released = time_us_64();
        }
    }
    else
    {
        button->state_dropped = false;
    }

    button->state = current_state;
}

bool button_raised(button_t* const button)
{
    return button->state_raised;
}

bool button_dropped(button_t* const button)
{
    return button->state_dropped;
}
