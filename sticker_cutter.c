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
#include "pos_controller/pos_controller.h"
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

// BEST!
// float kp_speed = 5.0, ki_speed = 4.0, kd_speed = 3.0;
// float kp_pos = 50.0, ki_pos = 0.0, kd_pos = 0.0;

// ******   PID Speed    ******
//
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata_speed;
pidc_t pid_speed;

// Control loop input,output and setpoint variables
float input_speed = 0.0, output_speed = 0.0;
float setpoint_speed = 0.0;

// Control loop gains
float kp_speed = 4.0, ki_speed = 2.0, kd_speed = 1.0;


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
float kp_pos = 30.0, ki_pos = 0.0, kd_pos = 0.0;

int speed;


// Pos ctrl
struct pos_controller pos_data;
posc_t pos;
float test_pos;

// Positon Controler
float a = 100.0;
float p_speed = 20.0;
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
float distance = 50.0;
float t_ramp;
float t_const;
float s_ramp;
float s_conts;

bool PID_timer_callback(struct repeating_timer *t) {
    // Encoder reading
    enc_old = enc_new;
    enc_new = quadrature_encoder_get_count(pio_qEnc, sm_0);
    enc_dif = enc_new - enc_old;

    input_pos = ((float)enc_new / 4000.0);
    pos_compute(pos, input_pos);

    setpoint_pos = test_pos;

    pid_compute(pid_pos);
    
    setpoint_speed = output_pos;
    input_speed = (enc_dif * 1000.0 / 4000.0) / pos->current_cycle_time;
    pid_compute(pid_speed);


    speed = (int)output_speed * 15.5; // 15.5 = 1024 / 66.6 [ot/s]

    // Printf 10 values/s
    if (i == 100){
        printf("%.8f;%.2f;%.2f;%.2f\n", input_pos, setpoint_pos, input_speed, test_pos);
        i = 0;
    }
    i++;

    // Set motor PWMs
    set_two_chans_pwm(pwm_slice_0, speed);

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

    pos = pos_control_create(&pos_data, &test_pos, a, p_speed);


    // *****   PID module part   *****
    // Prepare PID controller for operation
    pid_speed = pid_create(&ctrldata_speed, &input_speed, &output_speed, &setpoint_speed, kp_speed, ki_speed, kd_speed);
    // Set controler output limits from 0 to 200
    pid_limits(pid_speed, -66, 66);
    // Allow PID to compute and change output
    pid_auto(pid_speed);

    // Position PID
    pid_pos = pid_create(&ctrldata_pos, &input_pos, &output_pos, &setpoint_pos, kp_pos, ki_pos, kd_pos);
    // Set controler output limits from 0 to 200
    pid_limits(pid_pos, -66, 66);
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

    set_two_chans_pwm(pwm_slice_0, 0);

    add_repeating_timer_ms(1, PID_timer_callback, NULL, &timer);
    add_repeating_timer_ms(250, LCD_timer_callback, NULL, &LCD_timer);


    // Position ctrl
    pos_goto(pos, 50.0);

    
    while (1)
    {
        tight_loop_contents();
    }
}