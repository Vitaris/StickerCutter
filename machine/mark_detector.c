#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

#include "mark_detector.h"



detector_t create_detector(detector_t detector, uint8_t sensor_pin, float *feeder_position)
{
    // Feeder position pointer
    detector->feeder_position = feeder_position;

    // Set gpio pin as ADC
    // Avaible pins:    26, 27, 28, 29 (29 is cpu temperature)
    // Inputs:           0,  1,  2,  3
    adc_init();
    adc_gpio_init(sensor_pin);
    adc_select_input(sensor_pin - 26);

    detector->sensor_pin = sensor_pin;
    detector->shift_size = sizeof(uint16_t) * MEM_SIZE - 1;
    
    return detector;
}

void detector_compute(detector_t detector)
{
    // Get new value of reflectivity
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    // uint16_t result = adc_read() * conversion_factor;
    uint16_t result = adc_read();

    // Shift the memory of 1 position (it will free position 0 and delete pos 500)
    uint16_t tmp_memory[MEM_SIZE - 1];
    memcpy(tmp_memory, detector->memory, detector->shift_size)
    memcpy(detector->memory + 1, tmp_memory, detector->shift_size)
    // Add new value to the memory
    detector->memory[0] = result;


    // Do the same with positions
    float tmp_positions[MEM_SIZE - 1];
    memcpy(tmp_positions, detector->positions, detector->shift_size)
    memcpy(detector->positions + 1 , tmp_positions, detector->shift_size)
    // Add new position to the positions
    detector->positions[0] = *detector->feeder_position;

    // Incerement occupancy if it is not full
    if (detector->occupancy < MEM_SIZE)
    {
        detector->occupancy++;
    }

    

    // Detect a spike on memory data



    

    






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


