#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"

char empty_20[21] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
char empty_10[11] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};

machine_t create_machine()
{
    // Create machine data structure
    machine_t machine = (machine_t)malloc(sizeof(struct machine));
    machine->state = MANUAL_DISABLED_MOTORS;
    machine->machine_condition = OK;

    // Init buttons
    machine->F1 = create_button(5);
    machine->F2 = create_button(2);
    machine->Right = create_button(1);
    machine->Left = create_button(3);
    machine->In = create_button(4);
    machine->Out = create_button(0);

    // Init servos
    machine->machine_error = false;
    machine->enable = true;
    machine->homed = false;
    set_text_20(machine->error_message, "OK");

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    machine->servo_0 = servo_create("Cutter", offset, 0, ENC_0, PWM_0, 1.0, &machine->Right, &machine->Left, &machine->enable, &machine->machine_error, &machine->error_message);
    machine->servo_1 = servo_create("Feeder", offset, 1, ENC_1, PWM_1, 1.0, &machine->In, &machine->Out, &machine->enable, &machine->machine_error, &machine->error_message);

    // Mark probe
    // create_detector(&machine->ctrldata_detector, 0, &machine->test_servo_0.current_pos);
    return machine;
}

void machine_compute(machine_t machine, const float current_cycle_time)
{
    servo_compute(machine->servo_0, current_cycle_time);
    servo_compute(machine->servo_1, current_cycle_time);

    button_compute(machine->F1);
    button_compute(machine->F2);
    button_compute(machine->Right);
    button_compute(machine->Left);
    button_compute(machine->In);
    button_compute(machine->Out);

    // detector_compute(&machine->ctrldata_detector);

    // State machine
    switch(machine->state) {
        case MANUAL_DISABLED_MOTORS:
            set_text_20(machine->state_text, "Manual - Volne mot.");
            set_text_10(machine->F1_text, "Mot->ON");
            set_text_10(machine->F2_text, "");

            if (machine->F1->state_raised == true) {
                machine->state = MANUAL;
            }
            machine->enable = false;
            break;

        case MANUAL:
            set_text_20(machine->state_text, "Manual");
            set_text_10(machine->F1_text, "Mot->OFF");

            if (machine->homed == true) {
                set_text_10(machine->F2_text, "Start");
                if (machine->F2->state_raised == true) {
                    machine->state = AUTOMAT;
                }
            }
            else {
                set_text_10(machine->F2_text, "Noz->0");
                if (machine->F2->state_raised == true) {
                    machine->homed = true;
                }
            }

            machine->enable = true;
            servo_manual_handling(machine->servo_0);
            servo_manual_handling(machine->servo_1);

            if (machine->F1->state_raised == true) {
                machine->state = MANUAL_DISABLED_MOTORS;
            }

            break;

        case AUTOMAT:
            set_text_20(machine->state_text, "Automat");
            break;

        case FAILURE:
            set_text_20(machine->state_text, "Porucha!");
            machine->enable = false;
            break;
    }
}

void set_text(char LCD_text[], char text[], uint8_t len) {
    bool fill_spaces = false;
    uint8_t i = 0;
	while (i < len)	{
        if (text[i] == '\0') {
            fill_spaces = true;
        }
        if (fill_spaces == false) {
            LCD_text[i] = text[i];
        } else {
            LCD_text[i] = ' ';
        }
		i++;
	}
    LCD_text[len] = '\0';
}

void set_text_10(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 11);
}

void set_text_20(char LCD_text[], char text[]) {
    set_text(LCD_text, text, 21);
}
