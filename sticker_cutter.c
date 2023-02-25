#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
// #include "pico/binary_info.h"

#include "lcd/ant_lcd.h"

// pwm
#include "servo_motor/servo_pwm.h"
#include "servo_motor/pos_controller.h"
#include <math.h>

// Servo motor
#include "servo_motor/servo_motor.h"

#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "hardware/timer.h"
#include "pid/PID.h"

// Multicore
#include "pico/multicore.h"

// ADC
#include "hardware/adc.h"


// LCD
struct lcd_controller lcd_ctrl;
lcd_t lcd;

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
#define ENC_0 6
#define ENC_1 8

// First pin of PWM couple.
#define PWM_0 17
#define PWM_1 19

uint pwm_0_A;
uint pwm_0_B;
uint pwm_1_A;
uint slice_num_0_A;
uint slice_num_0_B;
uint slice_num_1_A;

struct servo_motor servo_ctrl_0;
struct servo_motor servo_ctrl_1;

struct repeating_timer PID_timer;
struct repeating_timer LCD_timer;

servo_t test_servo_0;
servo_t test_servo_1;

bool LCD_timer_callback(struct repeating_timer *t) 
{
    float2LCD(lcd, 0, 1, 8, 0.0);
    float2LCD(lcd, 0, 2, 8, 0.0);
    // float2LCD(lcd, 0, 2, 8, test_servo_1->in_pos);

    return true;

}

void core1_entry() {

    // LCD
    lcd = lcd_create(&lcd_ctrl, 10, 11, 12, 13, 14, 15, 16, 16, 4);
    string2LCD(lcd, 3, 0, "StickerCutter!");

    // int adc_val;
    // adc_val = adc_read();

    // add_repeating_timer_ms(100, LCD_timer_callback, NULL, &LCD_timer);

    while (1)
    {
        float2LCD(lcd, 2, 1, 6, test_servo_0->in_pos);
        string2LCD(lcd, 0, 1, "P:");
        float2LCD(lcd, 2, 2, 6, test_servo_1->in_pos);
        string2LCD(lcd, 0, 2, "P:");

        float2LCD(lcd, 12, 1, 6, test_servo_0->in_vel);
        string2LCD(lcd, 10, 1, "S:");
        float2LCD(lcd, 12, 2, 6, test_servo_1->in_vel);
        string2LCD(lcd, 10, 2, "S:");
        
    }
}

bool PID_timer_callback(struct repeating_timer *t) {

    motor_compute(test_servo_0);
    motor_compute(test_servo_1);
    
    if (test_servo_0->out_vel >= 0)
    {
        pwm_set_chan_level(slice_num_0_A, PWM_CHAN_B, test_servo_0->out_vel);
        pwm_set_chan_level(slice_num_0_B, PWM_CHAN_A, 0); 
    }
    else
    {
        pwm_set_chan_level(slice_num_0_A, PWM_CHAN_B, 0);
        pwm_set_chan_level(slice_num_0_B, PWM_CHAN_A, -test_servo_0->out_vel);
    }

    if (test_servo_1->out_vel >= 0)
    {
        pwm_set_chan_level(slice_num_0_B, PWM_CHAN_B, test_servo_1->out_vel);
        pwm_set_chan_level(slice_num_1_A, PWM_CHAN_A, 0); 
    }
    else
    {
        pwm_set_chan_level(slice_num_0_B, PWM_CHAN_B, 0);
        pwm_set_chan_level(slice_num_1_A, PWM_CHAN_A, -test_servo_1->out_vel);
    }

    return true;
}

int main() {

    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    test_servo_0 = servo_motor_create(&servo_ctrl_0, offset, 0, ENC_0, PWM_0);
    test_servo_1= servo_motor_create(&servo_ctrl_1, offset, 1, ENC_1, PWM_1);

    // Temporary fix - PCB design error
    // PWM channel are coupled together, I should choose even number for first one
    pwm_0_A = 17;
    pwm_0_B = 18;
    pwm_1_A = 20;
    gpio_set_function(pwm_0_A, GPIO_FUNC_PWM);
    gpio_set_function(pwm_0_B, GPIO_FUNC_PWM);

    gpio_set_function(19, GPIO_FUNC_PWM);
    gpio_set_function(20, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO # (it's slice #)
    slice_num_0_A = pwm_gpio_to_slice_num(pwm_0_A);
    pwm_set_clkdiv(slice_num_0_A, pwm_0_A); // PWM clock divider
    pwm_set_wrap(slice_num_0_A, 1023);  // Set period of 1024 cycles (0 to 1023 inclusive)
    pwm_set_enabled(slice_num_0_A, true);

    // Find out which PWM slice is connected to GPIO # (it's slice #)
    slice_num_0_B = pwm_gpio_to_slice_num(pwm_0_B);
    pwm_set_clkdiv(slice_num_0_B, pwm_0_B); // PWM clock divider
    pwm_set_wrap(slice_num_0_B, 1023);  // Set period of 1024 cycles (0 to 1023 inclusive)
    pwm_set_enabled(slice_num_0_B, true);

    // Find out which PWM slice is connected to GPIO # (it's slice #)
    slice_num_1_A = pwm_gpio_to_slice_num(pwm_1_A);
    pwm_set_clkdiv(slice_num_1_A, pwm_1_A); // PWM clock divider
    pwm_set_wrap(slice_num_1_A, 1023);  // Set period of 1024 cycles (0 to 1023 inclusive)
    pwm_set_enabled(slice_num_1_A, true);

    stdio_init_all();

    // Repeat timer
    add_repeating_timer_ms(1, PID_timer_callback, NULL, &PID_timer);
    
    // adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    // adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    // adc_select_input(0);
    

    multicore_launch_core1(core1_entry);

    busy_wait_ms(1000);

    // Position ctrl
    // pos_goto(pos, 50.0);
    motor_goto(test_servo_0, 50.0, 1.0);
    motor_goto(test_servo_1, 50.0, 0.25);

    
    while (1)
    {
        tight_loop_contents();
    }
}