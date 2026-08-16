#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <float.h>
#include <math.h>

#include "rotary_encoder.h"
#include "../servo_motor/button.h"

#define SCALE_FAST -40
#define SCALE_SLOW -4

struct rotary_encoder {
	// Encoder
	int32_t pos_raw;
	float position;
    float delta_position;
    int32_t last_encoder_count;
    float scale;
    float limit_max;
    float limit_min;
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
    gpio_pull_up(switch_pin_num);
    encoder->scale = SCALE_FAST;

    encoder->limit_max = FLT_MAX;
    // encoder->limit_min = -FLT_MAX;

    encoder->limit_min = 0.0;

    return encoder;
}

void rotary_encoder_compute(rotary_encoder_t* encoder) {
    // Update encoder position
    int32_t raw_current = quadrature_encoder_get_count(pio1, encoder->sm);

    int16_t delta = (int16_t)(raw_current - encoder->last_encoder_count);
    encoder->last_encoder_count = raw_current;

    encoder->delta_position = (float)delta / encoder->scale;
    encoder->position += encoder->delta_position;
    encoder->position = fmaxf(encoder->limit_min, fminf(encoder->position, encoder->limit_max));

    // Update knob switch state
    button_compute(encoder->knob_switch);
    if (button_raised(encoder->knob_switch)) {
        if (encoder->scale == SCALE_FAST) {
            encoder->scale = SCALE_SLOW;
        } else {
            encoder->scale = SCALE_FAST;
        }
    }
}

float rotary_encoder_get_position(rotary_encoder_t* encoder) {
    return encoder->position;
}

void rotary_encoder_set_position(rotary_encoder_t* encoder, float new_position) {
    encoder->position = new_position;
}

void rotary_encoder_reset_position(rotary_encoder_t* encoder) {
    encoder->position = 0.0;
}

void rotary_encoder_set_scale(rotary_encoder_t* encoder, float new_scale) {
    encoder->scale = new_scale;
}

bool rotary_encoder_button_raised(rotary_encoder_t* encoder) {
    return button_raised(encoder->knob_switch);
}

bool rotary_encoder_button_dropped(rotary_encoder_t* encoder) {
    return button_dropped(encoder->knob_switch);
}