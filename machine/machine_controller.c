#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"
#include "../servo_motor/servo_motor.h"
#include "../servo_motor/button.h"



void create_machine(machine_t* machine)
{
    // Init buttons
    create_button(&machine->F1, 5);
    create_button(&machine->F2, 2);
    create_button(&machine->Right, 1);
    create_button(&machine->Left, 3);
    create_button(&machine->In, 4);
    create_button(&machine->Out, 0);

    // Init servos
    uint offset = pio_add_program(pio0, &quadrature_encoder_program); // Init PIO program

    servo_create(&machine->test_servo_0, offset, 0, ENC_0, PWM_0, 1.0, FEEDER, &machine->Right, &machine->Left);
    servo_create(&machine->test_servo_1, offset, 1, ENC_1, PWM_1, 1.0, MANUAL, &machine->In, &machine->Out);

    machine->machine_state = AUTOMAT;
    strcpy(machine->state_text, "AUT");
    machine->machine_condition = OK;
    strcpy(machine->condition_text, "OK");

     // Mark probe
    create_detector(&machine->ctrldata_detector, 0, &machine->test_servo_0.current_pos);
}

void machine_compute(machine_t* machine, const float current_cycle_time)
{
    servo_compute(&machine->test_servo_0, current_cycle_time);
    servo_compute(&machine->test_servo_1, current_cycle_time);

    button_compute(&machine->F1);
    button_compute(&machine->F2);
    button_compute(&machine->Right);
    button_compute(&machine->Left);
    button_compute(&machine->In);
    button_compute(&machine->Out);

    detector_compute(&machine->ctrldata_detector);
        
    // If any servo fails
    if (machine->test_servo_0.state == ERR || machine->test_servo_1.state == ERR)
    {
        machine->machine_condition = ERROR;
        strcpy(machine->condition_text, "ERR");
    }
    else
    {
        machine->machine_condition = OK;
        strcpy(machine->condition_text, "OK ");
    }

    if (machine->F1.state_raised == true)
    {
        if (machine->machine_state == AUTOMAT)
        {
            machine->machine_state = MANUAL_M;
        }
        else if (machine->machine_state == MANUAL_M)
        {
            machine->machine_state = AUTOMAT;
        }
    }

    if (machine->F2.state_raised == true)
    {
        machine->machine_state = MANUAL_M;
        
    }

    // F1, F2, variable text labels
    if (machine->machine_state == AUTOMAT)
    {
        strcpy(machine->state_text, "AUT");
        strcpy(machine->F1_text, "  Manual  ");
        strcpy(machine->F2_text, "  Start   ");
    }
    else if (machine->machine_state == MANUAL_M)
    {
        strcpy(machine->state_text, "MAN");
        strcpy(machine->F1_text, "  Automat ");
        strcpy(machine->F2_text, "  Knife   ");
    }
}


