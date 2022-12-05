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


// ******   PID Speed    ******
//
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata_speed;
pidc_t pid_speed;

// Control loop input,output and setpoint variables
float input_speed = 0.0, output_speed = 0.0;
float setpoint_speed = 12.5;

// Control loop gains
float kp_speed = 7.0, ki_speed = 2.5, kd_speed = 1.0;


// ******   PID Position    ******
//
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata_pos;
pidc_t pid_pos;

// Control loop input,output and setpoint variables
float input_pos = 0.0, output_pos = 0.0;
float setpoint_pos = 0.0;

// Control loop gains
// float kp_pos = 1.5, ki_pos = 0.5, kd_pos = 0.01;
float kp_pos = 20.0, ki_pos = 15.0, kd_pos = 1.0;

int speed = 150;
double ramp = 0.0;

float goal_speed = 400.0;
float curr_speed = 0.0;
float diff_speed = 0.0;

int *lcd_speed;

// Positon Controler
float a = 900.0;
float p_speed = 50.0;
float ramp_time = 0.0;
bool noticed = false;
bool start = false;
uint64_t start_time;
uint64_t meas_time;
uint64_t sum_time = 0;

float enc2speed(int enc){
    // 1000 - 1ms
    // 4000 - encoder tics per rev
    return ((enc * 1000.0) / 4000.0);
}

int i = 0;
uint64_t first_start = 0;
uint64_t last_start = 0;
uint64_t current_start = 0;
float current_cycle_time;
bool first_cycle = true;
float distance = 25.0;
float t_ramp;
float t_const;
float s_ramp;
float s_conts;

bool PID_timer_callback(struct repeating_timer *t) {
    // current_start = time_us_64() - last_start;
    current_cycle_time = (float)(time_us_64() - last_start) * 0.001;
    // printf("cycle: %.2f\n", current_cycle_time); 
    last_start = time_us_64();
    if (first_cycle) {
        first_start = time_us_64();
        current_cycle_time = 1.0;

        // Pos regulator precomputation
        t_ramp = p_speed / a;
        s_ramp = 0.5 * p_speed * t_ramp;
        s_conts = distance - (s_ramp * 2);
        t_const = s_conts / p_speed;

        first_cycle = false;
    }
    ramp_time = (float)(time_us_64() - first_start) * 1.0e-6;

    if (!start){
        start_time = time_us_64();
        start = true;
    }


    // uint64_t t1 = time_us_64();
    enc_old = enc_new;
    enc_new = quadrature_encoder_get_count(pio_qEnc, sm_0);
    enc_dif = enc_new - enc_old;
    // lcd_speed = &enc_dif;

    // setpoint_speed = 12.5 + (sin(ramp) * 8.0);
    // setpoint = 1000;
    
    // setpoint_speed = (ramp_time < 24.0 / a) ? (a * ramp_time) : 0.0;

    //setpoint_pos = (ramp_time < 24.0 / a) ? (0.5 * a * pow(ramp_time, 2)) : (24.0 * ramp_time - 48);

    // setpoint pos
    if (ramp_time < t_ramp)
    {
        setpoint_pos = (0.5 * a * pow(ramp_time, 2));
    }
    else if (ramp_time >= t_ramp && ramp_time < (t_ramp + t_const))
    {
        setpoint_pos = (p_speed * ramp_time - s_ramp);
    }
    else if (ramp_time >= (t_ramp + t_const) && ramp_time < (t_ramp * 2) + t_const)
    {
        setpoint_pos = (p_speed * ramp_time - s_ramp) - (0.5 * a * pow(ramp_time - t_ramp - t_const, 2));
    }
    else if (ramp_time >= (t_ramp * 2) + t_const)
    {   
        setpoint_pos = setpoint_pos;
    }
  
    
    


    input_pos = ((float)enc_new / 4000.0);

    pid_compute(pid_pos);
    
    setpoint_speed = output_pos;
    input_speed = (enc_dif * 1000.0 / 4000.0) / current_cycle_time;
    pid_compute(pid_speed);
    

    speed = (int)output_speed * 15.5; // 15.5 = 1024 / 66.6 [ot/s]
    ramp += 0.008;

    if (ramp_time >= (float)24.0 / a && !noticed && false)
    {
        printf("Pos: %d, Setpoint_pos: %d, time: %d, Speed: %.2f, Calculated pos: %.2f, linear: %.2f\n",  enc_new, setpoint_pos, time_us_64() - start_time,
        (enc_dif * 1000.0 / 4000.0) / current_cycle_time, (0.5 * a * pow(ramp_time, 2)), (0.5 * 24.0 * ramp_time)); 
        noticed = true;
    }

    if (i == 100){
        printf("Pos: %.2f, Setpoint_pos: %d, speed: %.2f\r", input_pos, setpoint_pos, input_speed);
        i = 0;
    }
  

        

    pwm_set_chan_level(pwm_slice_0, PWM_CHAN_A, speed);
    pwm_set_chan_level(pwm_slice_0, PWM_CHAN_B, 1024 - speed); 

    // printf("Takes: %lld\n", time_us_64() - t1);
    i++;
    sum_time += (time_us_64() - meas_time);
    // printf("Avg time: %lld\n", sum_time / i); 
    return true;
}



bool LCD_timer_callback(struct repeating_timer *t) {
    // printf("Speed: %.2f, Pos: %d\r", enc2speed(enc_dif), enc_new); 
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
