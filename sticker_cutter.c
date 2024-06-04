#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

#include "lcd/ant_lcd.h"
#include "servo_motor/servo_motor.h"
#include "servo_motor/button.h"

// Servo Motors
uint64_t old_cycle_time = 0;

servo_t test_servo_0;
servo_t test_servo_1;

// Timers
struct repeating_timer servo_timer;
struct repeating_timer blink_timer;
struct repeating_timer LCD_refresh_timer;

// Buttons
button_t F1;
button_t F2;
button_t Right;
button_t Left;
button_t In;
button_t Out;

bool servo_error;
bool dummy2;
char (*error_message_0)[16];
char text_0[16] = {'\0'};
char (*servo_name_0)[10];
char (*servo_name_1)[10];
char servo_name_text_0[10] = {"Servo_0"};
char servo_name_text_1[10] = {"Servo_1"};

// LCD
lcd_t lcd;

bool blink_500ms;
bool lcd_refresh;

// Debug
#define MAN false
#define AUTO true
bool mode = MAN;
bool allways_true = true;

void core1_entry() {

    // LCD
   lcd = lcd_create(10, 11, 12, 13, 14, 15, 16, 16, 4);

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

            float2LCD(lcd, 8, 0, 6, test_servo_0->out_vel);
            float2LCD(lcd, 2, 1, 6, test_servo_0->current_pos);
            // float2LCD(lcd, 2, 1, 6, 0.0);
            string2LCD(lcd, 0, 1, "P:");
            float2LCD(lcd, 2, 2, 6, test_servo_1->current_pos);
            // float2LCD(lcd, 2, 2, 6, 0.0);
            string2LCD(lcd, 0, 2, "P:");

            int2LCD(lcd, 14, 1, 6, test_servo_0->enc_old);
            // float2LCD(lcd, 14, 1, 6, 0.0);
            string2LCD(lcd, 12, 1, "S:");
            // float2LCD(lcd, 14, 2, 6, machine.test_servo_1.current_vel);
            // float2LCD(lcd, 14, 2, 6, machine.ctrldata_detector.positions[0]);
            // float2LCD(lcd, 14, 2, 6, 0.0);
            string2LCD(lcd, 12, 2, "S:");
            int2LCD(lcd, 14, 2, 6, test_servo_1->enc_old);

            // int2LCD(lcd, 14, 0, 6, machine.test_servo_0.enc_old);
            // int2LCD(lcd, 14, 0, 6, machine.ctrldata_detector.result);
            // int2LCD(lcd, 14, 1, 6, machine.ctrldata_detector.average);
            // int2LCD(lcd, 14, 0, 6, 0.0);

            // Machine state
            // string2LCD(lcd, 6, 0, machine.state_text);
            // string2LCD(lcd, 10, 0, machine.condition_text);
            // string2LCD(lcd, 0, 3, machine.F1_text);
            // string2LCD(lcd, 10, 3, machine.F2_text);

            lcd_refresh = false;

            string2LCD(lcd, 0, 3, *error_message_0);
            // string2LCD(lcd, 8, 3, *error_message_1);


        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {

    // Get current time 
	float current_cycle_time = (float)(time_us_64() - old_cycle_time) * 1e-6;
	old_cycle_time = time_us_64();

    button_compute(F1);
    button_compute(F2);
    button_compute(Right);
    button_compute(Left);
    button_compute(In);
    button_compute(Out);
 
    servo_compute(test_servo_0, current_cycle_time);
    servo_compute(test_servo_1, current_cycle_time);


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
    F1 = create_button(5);
    F2 = create_button(2);
    Right = create_button(1);
    Left = create_button(3);
    In = create_button(4);
    Out = create_button(0);

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    // Init servos
    servo_error = false;
    error_message_0 = &text_0;

    servo_name_0 = &servo_name_text_0;
    servo_name_1 = &servo_name_text_1;

    strcpy(*error_message_0, "OK");
    test_servo_0 = servo_create(servo_name_0, offset, 0, ENC_0, PWM_0, 1.0, POSITIONER, &Right, &Left, &servo_error, error_message_0);
    test_servo_1 = servo_create(servo_name_1, offset, 1, ENC_1, PWM_1, 1.0, POSITIONER, &In, &Out, &servo_error, error_message_0);

    // servo_goto(test_servo_0, 10.0, 1.0);
    // servo_goto(test_servo_1, 10.0, 1.0);
    // test_servo_1 = servo_create(&servo_ctrl_1, offset, 1, ENC_1, PWM_1, 1.0, FEEDER, &In->state_changed, &Out->state_changed);

    // Initialize all of the present standard stdio types that are linked into the binary. 
    stdio_init_all();

    // Timer for servo control
    add_repeating_timer_ms(1, servo_timer_callback, NULL, &servo_timer);

    // 1s blink timer for lcd
    add_repeating_timer_ms(500, blink_timer_callback, NULL, &blink_timer);

    // 100ms LCD refresh timer
    add_repeating_timer_ms(100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
    // Launch core1
    multicore_launch_core1(core1_entry);

    while (1)
    {
        tight_loop_contents();
    }
}