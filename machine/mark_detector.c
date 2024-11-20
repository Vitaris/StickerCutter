#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "mark_detector.h"

detector_t create_detector(uint8_t sensor_pin, float *feeder_position) {
    // Create machine data structure
    detector_t detector = (detector_t)malloc(sizeof(struct detector));

    // Feeder position pointer
    // detector->feeder_position = feeder_position;

    // Set gpio pin as ADC
    // Avaible pins:    26, 27, 28, 29 (29 is cpu temperature)
    // Inputs:           0,  1,  2,  3
    adc_init();
    adc_gpio_init(sensor_pin + 26);
    adc_select_input(sensor_pin);

    // Initialize array
    memset(detector->stops, 0.0, sizeof(detector->stops));
    detector->samples = 0;
    detector->sampling_done = false;

    detector->sensor_pin = sensor_pin;
    detector->shift_size = sizeof(uint16_t) * MEM_SIZE - 1;
    detector->float_shift_size = sizeof(float) * MEM_SIZE - 1;
    detector->feeder_position = feeder_position;

    detector->diff_old = 0;

    detector->detector_state = DETECTOR_IDLE;

    // Calibration
    detector->calibrated = false;
    detector->calibration_sum = 0;
	detector->calibration_samples = 0;
	detector->calibration_min = 0;
	detector->calibration_max = 0;
    
    // Simulation
    size_calibration = CALIBRATION_SIZE;
    sample_calibration = 0;
    fill_calibration_data();

    size_simulation = SIMULATION_SIZE;
    sample_simulation = 0;
    fill_simulation_data();

    return detector;
}

void detector_compute(detector_t detector)
{
    // Get new value of reflectivity
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    // uint16_t result = adc_read() * conversion_factor;
    detector->result = adc_read();
    // detector->result = adc_read_simulation(data_calibration, &sample_calibration, size_calibration);
    // detector->result = adc_read_simulation(data_simulation, &sample_simulation, size_simulation);

    // Shift the memory of 1 position (it will free position 0 and delete last pos)
    uint16_t tmp_memory[MEM_SIZE - 1];
    memcpy(tmp_memory, detector->memory, detector->shift_size);
    memcpy(detector->memory + 1, tmp_memory, detector->shift_size);
    // Add new value to the memory
    detector->memory[0] = detector->result;

    // *****   EXPERIMENTAL CONTENT   *****

    float tmp_positions[MEM_SIZE - 1];
    memcpy(tmp_positions, detector->positions, detector->float_shift_size);
    memcpy(detector->positions + 1, tmp_positions, detector->float_shift_size);
    // Add new value to the memory
    detector->positions[0] = *detector->feeder_position;


    // Initila fill of samples and positions arrays
    if (!detector->sampling_done) {
        detector->samples++;
        if (detector->samples >= MEM_SIZE) {
            detector->initial_average = calculate_average(detector->memory, MEM_SIZE, 0); 
            detector->sampling_done = true;
        }
    }

    // 
    if (detector->sampling_done) {
        if (detector->detector_state == DETECTOR_SCANNING) {
            // Calculate "base" value for every evaluation
            detector->average = calculate_average(detector->memory, MEM_SIZE, detector->initial_average);

            // Find range of valid data to search for the local minimum
            int16_t a, b = 0;
            bool error;
            int8_t error_code;
            int16_t index_of_minimum = 0;
            bool range_found = find_range(detector->memory, MEM_SIZE, detector->average, &a, &b, &error, &error_code);
            if (range_found) {
                bool minimum_found = find_minimum_at_range(detector->memory, MEM_SIZE, &index_of_minimum, &a, &b, &error, &error_code);
                if (minimum_found) {
                    detector->stops[0] = detector->positions[index_of_minimum];
                    detector->detector_state = DETECTOR_LINE_FOUND;
                }
            }
        }
    }











    // *****   EXPERIMENTAL CONTENT   *****





    // Acumulate average value by keeping every 100th value (will take 1 second)
    if (detector->sampling_done == false && false)
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
    detector->samples++;



    // Compute difference between current and previous value
    detector->diff = detector->result - detector->memory[1];

    // Detect moment when diff goes from positive to negative
    if (detector->diff > 50 && detector->diff_old < -50 && false)
    {
        detector->positions[0] = *detector->feeder_position;
    }
    detector->diff_old = detector->diff;

    // Detect a spike on memory data

    switch(detector->detector_state) {
        case DETECTOR_IDLE:
            break;	    
	    case DETECTOR_SCANNING:
            break;
	    case DETECTOR_LINE_FOUND:
            break;
	    case DETECTOR_APPEND_STOP:
            break;
    }
    
}

void calibration(detector_t detector) {
    
}

uint16_t calculate_average(uint16_t data_array[], uint16_t array_length, uint16_t initial_average) {
    int i;
    uint16_t no_of_elements = 0;
    uint32_t sum = 0;
    for (i = 0; i < array_length; i++) {

        // If inital average is 0 means that take any value to the average (usefull for the first computation)
        if (initial_average > 0) {
            if (data_array[i] < initial_average - BELLOW_AVG_MIN) {
                continue;
            }
        }
        sum += data_array[i];
        no_of_elements++;
    }

    return sum / no_of_elements;
}

bool find_range(uint16_t data_array[], uint16_t array_length, uint16_t base_value,
                int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code) {
    int16_t tolerance_line = base_value - BELLOW_AVG_MIN;
    bool a_point_found = false;
    bool b_point_found = false;
    *error = false;
    *error_code = 0;

    // check if range is not at the begging or end of array, it could produce a false mininum
    if ((data_array[0] < tolerance_line) || (data_array[array_length - 1] < tolerance_line)){
        return false;
    }

    // find a point A when value went out from tolerance area
    // and point B when value returns back to tolerance area 
    for (int i = 1; i < array_length; i++) {
        if (data_array[i - 1] > tolerance_line &&  data_array[i] <= tolerance_line) {
            *point_a = i;
            // there should be only one point A and point B
            if (a_point_found) { 
                *error = true;
                *error_code = 1;
                return false;
            }
            a_point_found = true;
        }
        if (data_array[i - 1] < tolerance_line &&  data_array[i] >= tolerance_line) {
            *point_b = i;
            // there should be only one point A and point B
            if (b_point_found) { 
                *error = true;
                *error_code = 2;
                return false;
            }
            b_point_found = true;
        }

        // Point A and B should always be found, if not, raise the error.
        if (a_point_found && b_point_found) {
            return true;
        } else if (a_point_found && !b_point_found) {
            *error = true;
            *error_code = 3;
        } else if (!a_point_found && b_point_found)
        {
            *error = true;
            *error_code = 4;
        }
    }
}

bool find_minimum_at_range(uint16_t data_array[], uint16_t array_length, uint16_t *index_of_minimum,
                int16_t *point_a, int16_t *point_b, bool *error, int8_t *error_code) {
    *error = false;
    *error_code = 0;
    int16_t minimum = 10000;
    *index_of_minimum = 0;

    // Check that point A and B are within the data range.
    if (*point_a < 0) {
        *error = true;
        *error_code = 1;
        return false;
    }

    if (*point_b > array_length) {
        *error = true;
        *error_code = 2;
        return false;
    }

    if (*point_a > *point_b) {
        *error = true;
        *error_code = 3;
        return false;
    }

    // Find the minimum value in a given range of data
    for (int16_t i = *point_a; i < *point_b; i++) {
        if (data_array[i] < minimum) {
            minimum = data_array[i];
            *index_of_minimum = i; 
        }
    }
    return true;
}

uint16_t adc_read_simulation(uint16_t data[], uint16_t *sample, uint8_t size) {
    if (*sample >= size) {
        *sample = 0;
    }
    return data[(*sample)++];
}

void fill_calibration_data() {
    memcpy(data_calibration, (uint16_t[]) { 
        2006,
        2056,
        2064,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        1977,
        2000,
        2030,
        2076,
        2086,
        2066,
        2062,
        2066,
        2080,
        2077,
        2074,
        2067,
        2045,
        2026,
        2034,
        2032,
        2035,
        2042,
        2065,
        2075,
        2076,
        2058,
        2034,
        2005,
        2006,
        2056,
        2064,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        1977,
        2000,
        2030,
        2076,
        2086,
        2066,
        2062,
        2066,
        2080,
        2077,
        2074,
        2067,
        2045,
        2026,
        2034,
        2032,
        2035,
        2042,
        2065,
        2075,
        2076,
        2058,
        2034,
        2005,
        2006,
        2056,
        2064,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994
    }, sizeof (data_calibration));
}

void fill_simulation_data() {
    memcpy(data_simulation, (uint16_t[]) { 
        2053,
        2076,
        2083,
        2073,
        2061,
        2041,
        2026,
        2021,
        2022,
        2038,
        2072,
        2081,
        2089,
        2072,
        2043,
        2013,
        1970,
        1926,
        1892,
        1854,
        1818,
        1780,
        1734,
        1682,
        1631,
        1594,
        1568,
        1535,
        1571,
        1603,
        1643,
        1681,
        1728,
        1748,
        1776,
        1789,
        1828,
        1838,
        1867,
        1914,
        1965,
        1992,
        2029,
        2026,
        2023,
        2007,
        2003,
        1989,
        1998,
        2006,
        2056,
        2064,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        1977,
        2000,
        2030,
        2076,
        2100,
        2124,
        2137,
        2120,
        2105,
        2086,
        2066,
        2062,
        2066,
        2080,
        2077,
        2074,
        2067,
        2045,
        2026,
        2034,
        2032,
        2035,
        2042,
        2065,
        2075,
        2076,
        2058,
        2034,
        2005,
        1990,
        1965,
        1955,
        1947,
        1929,
        1912,
        1875,
        1811,
        1750,
        1678,
        1614,
        1561,
        1508,
        1493,
        1453,
        1467,
        1482,
        1505,
        1528,
        1549,
        1597,
        1647,
        1720,
        1803,
        1883,
        1964,
        2017,
        2056,
        2060,
        2064,
        2054,
        2041,
        2036,
        2022,
        2018,
        2009,
        2008,
        2001,
        2001,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994,
        1989,
        1992,
        2003,
        2043,
        2063,
        2066,
        2061,
        2036,
        2011,
        1984,
        1969,
        1963,
        1979,
        2002,
        2024,
        2023,
        2032,
        2015,
        2000,
        1994,
        1985,
        2011,
        2034,
        2060,
        2079,
        2082,
        2083,
        2056,
        2037,
        2012,
        2004,
        2003,
        2016,
        2041,
        2050,
        2041,
        2035,
        2026,
        2000,
        1997,
        1977,
        1980,
        1996,
        2010,
        2019,
        2019,
        2009,
        1994,
        1978,
        1977,
        2064,
        2056,
        2043,
        2013,
        1994,
        1990,
        1984,
        2000,
        2022,
        2029,
        2040,
        2036,
        2020,
        2004,
        1994
    }, sizeof (data_simulation));
}

void evaluate_calibration_data(detector_t detector) {

}

float get_next_stop(detector_t detector, float current_pos) {
    // If there are no stops in memory, send feeder somewhere far away to find some
    if (detector->stops[0] == 0.0) {
        return current_pos + 1000.0;
    }
    else
    {
        return detector->stops[0];
    }
}