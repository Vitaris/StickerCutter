#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "lcd/LCDops.h"
#include "lcd/generalOps.h"
#include "lcd/presetChars.h"
#include "lcd/presetMessages.h"
#include <math.h>

#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "hardware/timer.h"
#include "pid/PID.h"


int LCDpins[14] = {12,13,14,15,18,19,20,21,16,17,11,20,4};
volatile int new_value, enc_new, enc_old, enc_dif = 0;

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
const uint PIN_AB_0 = 2;
const uint PIN_AB_1 = 4;


// Quad encoder
PIO pio_qEnc = pio0;
const uint sm_0 = 0;


// PID
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata;
pidc_t pid;

// Control loop input,output and setpoint variables
float input = 0.0, output = 0.0;
float setpoint = 2000.0;

// Control loop gains
float kp = 2.5, ki = 1.5, kd = 0.1;

int i = 900;
float j = 9.0;
int speed = 150;
double ramp = 0.0;

float goal_speed = 400.0;
float curr_speed = 0.0;
float diff_speed = 0.0;

int* lcd_speed;

bool repeating_timer_callback(struct repeating_timer *t) {
        enc_old = enc_new;
        enc_new = quadrature_encoder_get_count(pio_qEnc, sm_0);
        enc_dif = enc_new - enc_old;
        // lcd_speed = &enc_dif;

        setpoint = 2000 + (sin(ramp) * 1500);
        // setpoint = 1000;
        input = (enc_dif * 1000.0 / 4000.0) * 60.0;
        // printf("Repeat at %lld\n", time_us_64());
        
        pid_compute(pid);

        speed = (int)output / 4.0;
        ramp += 0.001;
        return true;
    }

float enc2speed(int enc){
    // 1000 - 1ms
    // 4000 - encoder tics per rev
    return ((enc * 1000.0) / 4000.0) * 60.0;
}

struct repeating_timer timer;


int main() {
    stdio_init_all();

    uint offset = pio_add_program(pio_qEnc, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_qEnc, sm_0, offset, PIN_AB_0, 0);
     

    // *****   PWM module part   *****
    gpio_set_function(8, GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to GPIO 8 (it's slice 8)
    uint slice_num = pwm_gpio_to_slice_num(8);
    pwm_set_clkdiv(slice_num, 8); // PWM clock divider
    pwm_set_wrap(slice_num, 1023);  // Set period of 1024 cycles (0 to 1023 inclusive)
    pwm_set_enabled(slice_num, true);


    // *****   PID module part   *****
    // Prepare PID controller for operation
    pid = pid_create(&ctrldata, &input, &output, &setpoint, kp, ki, kd);
    // Set controler output limits from 0 to 200
    pid_limits(pid, 0, 4000);
    // Allow PID to compute and change output
    pid_auto(pid);

    

    //Initialize all needed pins as defined in LCDpins, set them as
    // outputs and then pull them low
    for(int gpio = 0; gpio < 11; gpio++){
        gpio_init(LCDpins[gpio]);
        gpio_set_dir(LCDpins[gpio], true);
        gpio_put(LCDpins[gpio], false);
    }

    //Initialize and clear the LCD, prepping it for characters / instructions
    LCDinit();
    LCDclear();
    sleep_ms(8);
    LCDgoto("00");
    LCDsendRawInstruction(0,0,"00001100");
    LCDwriteMessage("Quadrature encoder:");

    /* LCDgoto("1C");
    LCDwriteMessage("RPM"); */

    LCDgoto("48");
    LCDwriteMessage("I:");
    LCDgoto("1E");
    LCDwriteMessage("Out:");

    int pwm_value = 0;
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pwm_value);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 1024 - pwm_value);

    add_repeating_timer_ms(1, repeating_timer_callback, NULL, &timer);

    
    while (1)
    {
        // LCD
        // printf("Pos: %d\n", enc_new);
        // printf("dif: %d\n", enc_dif);
        /* int2LCD("40", enc_new, 4);
        float2LCD("14", enc2speed(enc_dif), 4);
        printf("enc_diff: %d\n", enc_dif);
        int2LCD("54", speed, 4);

        // PID
        float2LCD("4A", pid->lastin, 4);
        float2LCD("22", output, 4); */

        // float2LCD("14", enc2speed(*lcd_speed), 4);
        printf("Speed: %.2f\r", enc2speed(enc_dif)); 
        // PWM        
        int pwm_value = new_value * 5;
        speed = (speed >= 900) ? 900: speed;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, speed);
        pwm_set_chan_level(slice_num, PWM_CHAN_B, 1024 - speed); 
    }
}
