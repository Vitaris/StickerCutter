#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "mark_detector.h"

void create_detector(detector_t* detector, uint8_t sensor_pin, float *feeder_position)
{
    // Feeder position pointer
    detector->feeder_position = feeder_position;

    // Set gpio pin as ADC
    // Avaible pins:    26, 27, 28, 29 (29 is cpu temperature)
    // Inputs:           0,  1,  2,  3
    adc_init();
    adc_gpio_init(sensor_pin + 26);
    adc_select_input(sensor_pin);

    detector->sensor_pin = sensor_pin;
    detector->shift_size = sizeof(uint16_t) * MEM_SIZE - 1;
    
}

void detector_compute(detector_t* detector)
{
    // Get new value of reflectivity
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    // uint16_t result = adc_read() * conversion_factor;
    detector->result = adc_read();
    
    // Shift the memory of 1 position (it will free position 0 and delete last pos)
    uint16_t tmp_memory[MEM_SIZE - 1];
    memcpy(tmp_memory, detector->memory, detector->shift_size);
    memcpy(detector->memory + 1, tmp_memory, detector->shift_size);
    // Add new value to the memory
    detector->memory[0] = detector->result;

    // Acumulate average value by keeping every 100th value (will take 1 second)
    if (detector->sampling_done == false)
    {
        if (detector->samples == 100 && detector->sampling_done == false )
        {
            detector->samples = 0;
            uint16_t tmp_memory[AVG_SIZE - 1];
            memcpy(tmp_memory, detector->average_memory, detector->shift_size);
            memcpy(detector->average_memory + 1, tmp_memory, detector->shift_size);
            detector->average_memory[0] = detector->result;
            detector->average_samples++;
            if (detector->average_samples == AVG_SIZE)
            {
                detector->sampling_done = true;
                // Compute average
                uint32_t sum = 0;
                for (size_t i = 0; i < AVG_SIZE; i++)
                {
                    sum += detector->average_memory[i];
                }
                detector->average = sum / AVG_SIZE;
            }
            
        }
        else
        {
            detector->samples++;
        }
    }

    // Compute difference between current and previous value
    detector->diff = detector->result - detector->memory[1];
    

    // Detect moment when diff goes from positive to negative
    if (detector->diff > 50 && detector->diff_old < -50)
    {
        detector->positions[0] = *detector->feeder_position;
    }
    detector->diff_old = detector->diff;

    // Detect a spike on memory data
    

    
}
