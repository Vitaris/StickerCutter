#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"



machine_t create_machine(machine_t machine, bool *F1, bool *F2, bool *servo_state_01, bool *servo_state_02)
{
    machine->machine_state = AUTOMAT;
    machine->machine_condition = OK;

    machine->F1 = F1;
    machine->F2 = F2;

    machine->servo_state_01 = servo_state_01;
    machine->servo_state_02 = servo_state_02;

    
    // Set mark probe
    // Set GPIO 5 to be an input
    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);

    // adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    // adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    // adc_select_input(0);

    float* dummy_float;
    float a = 0.0;
    dummy_float = &a;
    // Mark probe
    machine->detector = create_detector(&machine->ctrldata_detector, 1, dummy_float);

    return machine;
}

void machine_compute(machine_t machine)
{
    // If any servo fails
    if (machine->servo_state_01 == false || machine->servo_state_02 == false)
    {
        machine->machine_condition = ERROR;
    }

    if (*machine->F1 == true)
    {
        machine->machine_state = AUTOMAT;
    }

    if (*machine->F2 == true)
    {
        machine->machine_state = MANUAL_M;
    }

    // F1, F2, variable text labels
    if (machine->machine_state == AUTOMAT)
    {
        strcpy(machine->F1_text, "  Start   ");
        strcpy(machine->F2_text, "  Stop    ");
    }
    else
    {
        strcpy(machine->F1_text, "  Move    ");
        strcpy(machine->F2_text, "  Click   ");
    }


    
}


