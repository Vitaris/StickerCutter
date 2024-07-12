#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"


machine_t create_machine()
{
    // Create machine data structure
    machine_t machine = (machine_t)malloc(sizeof(struct machine));

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
    strcpy(machine->error_message, "OK");

    // Init PIO
    uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    machine->servo_0 = servo_create("Cutter", offset, 0, ENC_0, PWM_0, 1.0, &machine->Right, &machine->Left, &machine->enable, &machine->machine_error, &machine->error_message);
    machine->servo_1 = servo_create("Feeder", offset, 1, ENC_1, PWM_1, 1.0, &machine->In, &machine->Out, &machine->enable, &machine->machine_error, &machine->error_message);

    machine->machine_state = MANUAL;
    machine->machine_condition = OK;



    // machine->machine_state = AUTOMAT;
    // strcpy(machine->state_text, "AUT");
    // machine->machine_condition = OK;
    // strcpy(machine->condition_text, "OK");

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

    if (machine->machine_state == MANUAL) {
        servo_manual_handling(machine->servo_0);
        servo_manual_handling(machine->servo_1);
    }

    if (machine->F1->state_raised == true) {
        if (machine->machine_state == AUTOMAT) {
            machine->machine_state = MANUAL;
        }
        else if (machine->machine_state == MANUAL) {
            machine->machine_state = AUTOMAT;
        }
    }

    if (machine->F2->state_raised == true) {
        if (machine->machine_error == true) {
            machine->machine_error = false;
            strcpy(machine->error_message, "OK                  ");
        }
    }

    if (machine->machine_error == true) {
        machine->enable = false;
    }

    // F1, F2, variable text labels
    if (machine->machine_state == AUTOMAT)
    {
        strcpy(machine->state_text, "Automat");
        strcpy(machine->F1_text, "  Manual  ");
        strcpy(machine->F2_text, "  Start   ");
    }
    else if (machine->machine_state == MANUAL)
    {
        strcpy(machine->state_text, "Manual");
        strcpy(machine->F1_text, " Automat  ");
        strcpy(machine->F2_text, "    Knife ");
    }
}


