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

// pwm
#include "pwm/servo_pwm.h"
#include <math.h>

#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "pio_rotary_encoder.pio.h"
#include "hardware/timer.h"
#include "pid/PID.h"


int LCDpins[14] = {12,13,14,15,18,19,20,21,16,17,11,20,4};
volatile int new_value, enc_new, enc_old, enc_dif = 0;
uint pwm_slice_0;

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
const uint PIN_AB_0 = 2;
const uint PIN_AB_1 = 4;


// Quad encoder
PIO pio_qEnc = pio0;
const uint sm_0 = 0;


// PID Speed
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata_speed;
pidc_t pid_speed;

// Control loop input,output and setpoint variables
float input_speed = 0.0, output_speed = 0.0;
float setpoint_speed = 12.5;

// Control loop gains
float kp_speed = 2.5, ki_speed = 1.5, kd_speed = 0.1;


// PID Position
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata_pos;
pidc_t pid_pos;

// Control loop input,output and setpoint variables
float input_pos = 0.0, output_pos = 0.0;
float setpoint_pos = 12.5;

// Control loop gains
float kp_pos = 2.5, ki_pos = 1.5, kd_pos = 0.1;


int speed = 150;
double ramp = 0.0;

float goal_speed = 400.0;
float curr_speed = 0.0;
float diff_speed = 0.0;

int *lcd_speed;

// Positon Controler
float a = 96.0;
float ramp_time = 0.0;

bool PID_timer_callback(struct repeating_timer *t) {
    ramp_time += 0.001;
    // uint64_t t1 = time_us_64();
    enc_old = enc_new;
    enc_new = quadrature_encoder_get_count(pio_qEnc, sm_0);
    enc_dif = enc_new - enc_old;
    // lcd_speed = &enc_dif;

    // setpoint_speed = 12.5 + (sin(ramp) * 8.0);
    // setpoint = 1000;
    input_speed = (enc_dif * 1000.0 / 4000.0);
    setpoint_speed = (ramp_time <= 24.0 / a) ? (a * ramp_time) : 24.0;
 
    pid_compute(pid_speed);

    speed = (int)output_speed * 15.5; // 15.5 = 1024 / 66.6 [ot/s]
    ramp += 0.008;

    if (ramp_time > 1.5)
    {
        ramp_time = 0.0;
        setpoint_speed = 0.0;
    }
        
    

    pwm_set_chan_level(pwm_slice_0, PWM_CHAN_A, speed);
    pwm_set_chan_level(pwm_slice_0, PWM_CHAN_B, 1024 - speed); 

    // printf("Takes: %lld\n", time_us_64() - t1);
    return true;
}

float enc2speed(int enc){
    // 1000 - 1ms
    // 4000 - encoder tics per rev
    return ((enc * 1000.0) / 4000.0);
}

bool LCD_timer_callback(struct repeating_timer *t) {
    printf("Speed: %.2f, Pos: %d\r", enc2speed(enc_dif), enc_new); 
    // float2LCD("14", enc2speed(enc_dif), 4);
    return true;
}




struct repeating_timer timer;
struct repeating_timer LCD_timer;


int main() {
    stdio_init_all();

    uint offset = pio_add_program(pio_qEnc, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_qEnc, sm_0, offset, PIN_AB_0, 0);
    
    // *****   PWM module part   *****
    pwm_slice_0 = pwm_chan_init(8);
    pwm_slice_0 = pwm_chan_init(9);




    // *****   PID module part   *****
    // Prepare PID controller for operation
    pid_speed = pid_create(&ctrldata_speed, &input_speed, &output_speed, &setpoint_speed, kp_speed, ki_speed, kd_speed);
    // Set controler output limits from 0 to 200
    pid_limits(pid_speed, 0, 66);
    // Allow PID to compute and change output
    pid_auto(pid_speed);

    // Position PID
    pid_pos = pid_create(&ctrldata_pos, &input_pos, &output_pos, &setpoint_pos, kp_pos, ki_pos, kd_pos);
    // Set controler output limits from 0 to 200
    pid_limits(pid_pos, 0, 66);
    // Allow PID to compute and change output
    pid_auto(pid_pos);

    

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
    pwm_set_chan_level(pwm_slice_0, PWM_CHAN_A, pwm_value);
    pwm_set_chan_level(pwm_slice_0, PWM_CHAN_B, 1024 - pwm_value);

    add_repeating_timer_ms(1, PID_timer_callback, NULL, &timer);
    add_repeating_timer_ms(250, LCD_timer_callback, NULL, &LCD_timer);

    
    while (1)
    {
        tight_loop_contents();
    }
}
