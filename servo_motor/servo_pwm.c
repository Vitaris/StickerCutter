#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "servo_pwm.h"


uint pwm_chan_init(uint gpio_pin_num){
    gpio_set_function(gpio_pin_num, GPIO_FUNC_PWM);
    gpio_set_function(gpio_pin_num + 1, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO # (it's slice #)
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin_num);
    pwm_set_clkdiv(slice_num, gpio_pin_num); // PWM clock divider
    pwm_set_wrap(slice_num, 1023);  // Set period of 1024 cycles (0 to 1023 inclusive)
    pwm_set_enabled(slice_num, true);
    return slice_num;
}

void set_two_chans_pwm(uint slice_num, int speed){
    if (speed >= 0)
    {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, speed);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 0); 
    }
    else
    {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, -speed);
    }
}

