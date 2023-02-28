#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "button.h"



button_t create_button(button_t button, uint8_t gpio_pin_num)
{
    // Set button GPIO to be an input
    gpio_init(gpio_pin_num);
    gpio_set_dir(gpio_pin_num, GPIO_IN);

    button->gpio_pin_num = gpio_pin_num;

    return button;
}

void button_compute(button_t button)
{
    // Read the button state
    bool current_state = gpio_get(button->gpio_pin_num);

    // Change detection
    if (current_state != button->state)
    {
        // Button state has changed
        button->state_changed = true;
        button->time_last_changed = time_us_64();
    }
    else
    {
        button->state_changed = false;
    }

    button->state = current_state;
    
}



