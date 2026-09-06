#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "quadrature_encoder.pio.h"

typedef struct rotary_encoder rotary_encoder_t;


rotary_encoder_t* create_rotary_encoder(const uint8_t enc_pin_num, uint8_t switch_pin_num, uint sm, uint pio_ofset);

void rotary_encoder_compute(rotary_encoder_t* encoder);

float rotary_encoder_get_position(rotary_encoder_t* encoder);

int rotary_encoder_get_position_int(rotary_encoder_t* encoder);

void rotary_encoder_set_position(rotary_encoder_t* encoder, float new_position);

void rotary_encoder_reset_position(rotary_encoder_t* encoder);

void rotary_encoder_set_scale(rotary_encoder_t* encoder, float new_scale);

bool rotary_encoder_button_raised(rotary_encoder_t* encoder);

bool rotary_encoder_button_dropped(rotary_encoder_t* encoder);

#endif