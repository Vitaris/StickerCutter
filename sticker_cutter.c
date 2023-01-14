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


// LCD
int LCDpins[14] = {12,13,14,15,18,19,20,21,16,17,11,20,4};
uint pwm_slice_0;

// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
const uint PIN_AB_0 = 2;
const uint PIN_AB_1 = 4;



struct servo_motor servo_ctrl_1;
struct servo_motor servo_ctrl_2;

servo_t test_servo_1;
servo_t test_servo_2;
int i = 0;
int j = 0;

void core1_entry() {

    // LCD
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
    busy_wait_ms(8);
    LCDgoto("00");
    LCDsendRawInstruction(0,0,"00001100");
    LCDwriteMessage("Quadrature encoder:");

    /* LCDgoto("1C");
    LCDwriteMessage("RPM"); */

    LCDgoto("40");
    LCDwriteMessage("Pos:");
    LCDgoto("14");
    LCDwriteMessage("Speed:");

    while (1)
    {
        float2LCD("45", test_servo_1->in_pos, 5);
        float2LCD("1B", test_servo_1->in_vel, 5);
        int2LCD("56", test_servo_1->enc_old, 5);
        /*
        ii++;
        busy_wait_ms(1000);
        */
    }
}


bool PID_timer_callback(struct repeating_timer *t) {

    // Printf 10 values/s
    if (i == 100){
        printf("\x1b[2J"); // Clear screen
        printf("+------------------------------------------+\n" 
               " Pos:    %.8f  \n" 
               " Speed:  %.2f  \n" 
               " PID Vel out:  %.2f  \n" 
               " Cycle time :  %.6f  \n" 
               " j:  %d        \n" 
               "+------------------------------------------+\n", 
               test_servo_1->in_pos, test_servo_1->in_vel, test_servo_1->out_vel, test_servo_1->pos->current_cycle_time, j);
        i = 0;
        if (j == 0)
            j = 1;
        else
            j = 0;
    }
    i++;

    motor_compute(test_servo_1);
    motor_compute(test_servo_2);

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

    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    test_servo_1 = servo_motor_create(&servo_ctrl_1, offset, 0, PIN_AB_0, 8);
    test_servo_2 = servo_motor_create(&servo_ctrl_2, offset, 1, PIN_AB_1, 10);

    stdio_init_all();

    

    // Repeat timer
    add_repeating_timer_ms(1, PID_timer_callback, NULL, &timer);
    add_repeating_timer_ms(250, LCD_timer_callback, NULL, &LCD_timer);


    

    multicore_launch_core1(core1_entry);

    busy_wait_ms(2500);

    // Position ctrl
    // pos_goto(pos, 50.0);
    motor_goto(test_servo_1, 50.0);

    
    while (1)
    {
        tight_loop_contents();
    }
}