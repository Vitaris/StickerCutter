#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "quadrature_encoder.pio.h"

typedef struct rotary_encoder rotary_encoder_t;


rotary_encoder_t* create_rotary_encoder(const uint8_t enc_pin_num, uint8_t switch_pin_num, uint sm, uint pio_ofset);

void rotary_encoder_compute(rotary_encoder_t* encoder);

float get_rotary_encoder_position(rotary_encoder_t* encoder);

#endif