#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "machine_controller.h"



machine_t create_machine(machine_t machine, bool *F1, bool *F2, bool *servo_state_01, bool *servo_state_02)
{
    machine->machine_state = MANUAL_M;
    machine->machine_condition = OK;

    machine->F1 = F1;
    machine->F2 = F2;

    machine->servo_state_01 = servo_state_01;
    machine->servo_state_02 = servo_state_02;
    

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
        strcpy(machine->F1_text, "  START   ");
        strcpy(machine->F2_text, "  STOP    ");
    }
    else
    {
        strcpy(machine->F1_text, "  Move    ");
        strcpy(machine->F2_text, "  Click   ");
    }


    
}


