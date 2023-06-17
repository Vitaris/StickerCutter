#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

#include "lcd/ant_lcd.h"
#include "servo_motor/servo_motor.h"
#include "servo_motor/button.h"
#include "machine/machine_controller.h"

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

// Servo Motors
uint64_t old_cycle_time = 0;
struct servo_motor servo_ctrl_0;
struct servo_motor servo_ctrl_1;

servo_t test_servo_0;
servo_t test_servo_1;

// Timers
struct repeating_timer servo_timer;
struct repeating_timer blink_timer;
struct repeating_timer LCD_refresh_timer;

// Buttons
struct button button_data_F1;
struct button button_data_F2;
struct button button_data_Right;
struct button button_data_Left;
struct button button_data_In;
struct button button_data_Out;

button_t F1;
button_t F2;
button_t Right;
button_t Left;
button_t In;
button_t Out;

// Machine controller
struct machine machine_data;
machine_t machine;

// LCD
struct lcd_controller lcd_ctrl;
lcd_t lcd;

bool blink_500ms;
bool lcd_refresh;

// Debug
#define MAN false
#define AUTO true
bool mode = MAN;
bool allways_true = true;

bool LCD_timer_callback(struct repeating_timer *t) 
{
    float2LCD(lcd, 0, 1, 8, 0.0);
    float2LCD(lcd, 0, 2, 8, 0.0);
    // float2LCD(lcd, 0, 2, 8, test_servo_1->current_pos);

    return true;
}

void core1_entry() {

    // LCD
    lcd = lcd_create(&lcd_ctrl, 10, 11, 12, 13, 14, 15, 16, 16, 4);

    // int adc_val;
    // adc_val = adc_read();

    // add_repeating_timer_ms(100, LCD_timer_callback, NULL, &LCD_timer);
    string2LCD(lcd, 0, 0, "Mode:");

    while (1)
    {
        if (lcd_refresh == true)
        {
            if (mode == MAN && false)
            {
            string2LCD(lcd, 6, 0, "MAN");
            }

            float2LCD(lcd, 2, 1, 6, test_servo_0->current_pos);
            string2LCD(lcd, 0, 1, "P:");
            float2LCD(lcd, 2, 2, 6, test_servo_1->current_pos);
            string2LCD(lcd, 0, 2, "P:");

            float2LCD(lcd, 14, 1, 6, test_servo_0->current_vel);
            string2LCD(lcd, 12, 1, "S:");
            float2LCD(lcd, 14, 2, 6, test_servo_1->current_vel);
            string2LCD(lcd, 12, 2, "S:");

            int2LCD(lcd, 14, 0, 6, test_servo_0->enc_old);

            // Machine state
            if (machine->machine_state == MANUAL_M)
            {
                string2LCD(lcd, 6, 0, "MAN");
            }
            else if (machine->machine_state == AUTOMAT)
            {
                string2LCD(lcd, 6, 0, "AUT");
            }

            string2LCD(lcd, 0, 3, machine->F1_text);
            string2LCD(lcd, 10, 3, machine->F2_text);

            /*
            if (blink_500ms)
            {
                string2LCD(lcd, 14, 0, "Error!");
            }
            else
            {
                string2LCD(lcd, 14, 0, "      ");
            }
            */

            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {

    // Get current time 
	float current_cycle_time = (float)(time_us_64() - old_cycle_time) * 1e-6;
	old_cycle_time = time_us_64();

    servo_compute(test_servo_0, current_cycle_time);
    servo_compute(test_servo_1, current_cycle_time);

    // Buttons
    button_compute(F1);
    button_compute(F2);
    button_compute(Right);
    button_compute(Left);
    button_compute(In);
    button_compute(Out);

    // Machine
    machine_compute(machine);

    
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

bool blink_timer_callback(struct repeating_timer *t) {

    if (blink_500ms == true)
    {
        blink_500ms = false;
    }
    else
    {
        blink_500ms = true;
    }
    
    return true;
}

bool LCD_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    
    return true;
}

int main() {
    
    // Init buttons
    F1 = create_button(&button_data_F1, 5);
    F2 = create_button(&button_data_F2, 2);
    Right = create_button(&button_data_Right, 1);
    Left = create_button(&button_data_Left, 3);
    In = create_button(&button_data_In, 4);
    Out = create_button(&button_data_Out, 0);

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    // Init servos
    test_servo_0 = servo_create(&servo_ctrl_0, offset, 0, ENC_0, PWM_0, 1.0, FEEDER, &Right->state_changed, &Left->state_changed);
    test_servo_1 = servo_create(&servo_ctrl_1, offset, 1, ENC_1, PWM_1, 1.0, MANUAL, &In->state, &Out->state);
    // test_servo_1 = servo_create(&servo_ctrl_1, offset, 1, ENC_1, PWM_1, 1.0, FEEDER, &In->state_changed, &Out->state_changed);

    // Init machine controller
    machine = create_machine(&machine_data, &F1->state, &F2->state, &allways_true, &allways_true);

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

    // Initialize all of the present standard stdio types that are linked into the binary. 
    stdio_init_all();

    // Timer for servo control
    add_repeating_timer_ms(1, servo_timer_callback, NULL, &servo_timer);

    // 1s blink timer
    add_repeating_timer_ms(500, blink_timer_callback, NULL, &blink_timer);

    // 100ms LCD refrehs timer
    add_repeating_timer_ms(100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
    // Launch core1
    multicore_launch_core1(core1_entry);

    while (1)
    {
        tight_loop_contents();
    }
}