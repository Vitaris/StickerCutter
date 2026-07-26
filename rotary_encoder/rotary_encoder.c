#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "rotary_encoder.h"
#include "../servo_motor/button.h"

struct rotary_encoder {
	// Encoder
	int32_t pos_raw;
	float position;
    uint sm;

    button_t *knob_switch;
};

rotary_encoder_t* create_rotary_encoder(const uint8_t enc_pin_num, uint8_t switch_pin_num, uint sm, uint pio_ofset) {
    rotary_encoder_t* encoder = calloc(1, sizeof(struct rotary_encoder));
    encoder->pos_raw = 0;
    encoder->position = 0.0;
    encoder->sm = sm;

    gpio_init(enc_pin_num);
    gpio_init(enc_pin_num + 1);

    // Encoder
	quadrature_encoder_program_init(pio1, sm, pio_ofset, enc_pin_num, 10000);
	encoder->sm = sm;

    encoder->knob_switch = create_button(switch_pin_num);

    return encoder;
}

void rotary_encoder_compute(rotary_encoder_t* encoder) {
    // Update encoder position
    encoder->pos_raw = quadrature_encoder_get_count(pio1, encoder->sm);
    encoder->position = (float)encoder->pos_raw / 40.0;
    
    // Update knob switch state
    button_compute(encoder->knob_switch);
}

float get_rotary_encoder_position(rotary_encoder_t* encoder) {
    return encoder->position;
}