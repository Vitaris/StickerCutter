#include <stdlib.h>
#include <string.h>
#include "servo_motor.h"
#include "../servo_motor/button.h"

#define CYCLE_TIME 0.001
#define FOLLOWING_ERROR 1.0 // Maximum permisible position deviation

struct servo_motor {
	// Encoder
	uint sm;
	int32_t enc_old;

	// PWM
	uint pwm_slice;
	
	// PID Position
	pid_data_t* pid_pos;
	float enc_position;
	float out_pos;
	float set_pos;

	// PID Velocity
	pid_data_t* pid_vel;
	float enc_speed;
	float out_vel;
	float set_vel;

	// Error handling
	bool *error; // Pointer to bool
	char (*error_message)[21]; // Error message
	bool pos_error_internal;

	// Feeder controller
	float next_stop;

	// Controller state
	char servo_name[10];

	// Servo controler
	enum positioning{
		IDLE,
		REQUESTED,
		ACCELERATING,
		BRAKING,
		POSITION_REACHED
	} positioning;

	float servo_position;
	float servo_speed;
	uint32_t delay_start;
	bool *enable;
	bool enable_previous;
	float computed_speed;
	bool positive_direction;
	bool set_zero;
	bool nominal_speed_reached;
	float enc_offset;

	// Default movement
	float nominal_speed; 	// Desired motor speed
	float nominal_acc;		// Motor acceleration
	float current_speed; 	// Desired motor speed
	float current_acc;		// Motor acceleration
	float scale;			// Scale factor of the servo motor

	// Manual control
	button_t *man_plus;
	button_t *man_minus;
};

servo_t* servo_create(const char servo_name[7], const uint pio_ofset, const uint sm, 
                    const uint encoder_pin, const uint pwm_pin, const float scale,
                    button_t *const man_plus, button_t *const man_minus, 
                    bool *const enable, bool *const error, char (*const message)[21]) {
	// Create servo data structure
	servo_t* servo = calloc(1, sizeof(struct servo_motor));
	strcpy(servo->servo_name, servo_name);
	servo->enable = enable;

	// Encoder
	quadrature_encoder_program_init(pio0, sm, pio_ofset, encoder_pin, 0);
	servo->sm = sm;
	servo->scale = scale;
	servo->enc_old = 0;
			
	// PWM
	servo->pwm_slice = pwm_chan_init(pwm_pin);

	// PID
	servo->pid_pos = pid_create(&servo->enc_position, &servo->out_pos, &servo->set_pos, 40.0, 0.0, 0.5);
	servo->pid_vel = pid_create(&servo->enc_speed, &servo->out_vel, &servo->set_vel, 5.0, 3.0, 1.0);

	// Positional controller
	servo->nominal_acc = 100.0;
	servo->nominal_speed = 30.0;
	servo->enc_old = 0;
	servo->computed_speed = 0.0;
	servo->servo_speed = 0.0;

	// Limits
	servo->error = error;
	servo->error_message = message;
	strcpy(*servo->error_message, "OK");
	servo->pos_error_internal = false;
	servo->enc_offset = 0.0;
	servo->set_zero = false;

	// Buttons 
	servo->man_plus = man_plus;
	servo->man_minus = man_minus;

	return servo;
}

float enc2speed(const int32_t enc_diff) {
	// Time difference of measured encoder tics
	// is ~1 milisecond but we want to get it in seconds,
	// so we have to multiply by 1000
    
	// 4000 - encoder tics per rev

	// So for speed we have to multiply by 1000 and divide by 4000
	// (enc_diff * 1000.0) / 4000.0
	
	// return (float)enc_diff / 4.0;
	return (float)enc_diff * (1.0 / CYCLE_TIME) / 4000.0;
}

float get_breaking_distance(const servo_t* const servo) {
	return 0.5 * (pow(servo->computed_speed, 2) / servo->current_acc);
}

void servo_stop_positioning(servo_t* const servo) {
	servo->next_stop = servo->set_pos + get_breaking_distance(servo);
}

void next_positon_compute(servo_t* const servo) {
	switch(servo->positioning) {
        case IDLE:
			servo->nominal_speed_reached = false;
			break;
		
		case REQUESTED:
			// First occurence of movement request, save the position of movement beginning
			if (servo->next_stop >= servo->enc_position) {
				// Positive direction
				servo->positive_direction = true;
				servo->current_acc = servo->nominal_acc;
				servo->current_speed = servo->nominal_speed;
			} else {
				// Negative direction
				servo->positive_direction = false;
				servo->current_acc = -servo->nominal_acc;
				servo->current_speed = -servo->nominal_speed;
			}
			if (servo->delay_start > 0) {
				servo->delay_start--;
				break;
			}
			servo->computed_speed = 0.0;
			servo->positioning = ACCELERATING;
			break;

		case ACCELERATING:
			servo->computed_speed += servo->current_acc * CYCLE_TIME;

			// check if nominal speed has been reached
			if (fabs(servo->computed_speed) > fabs(servo->current_speed)) {
				servo->nominal_speed_reached = true;
				servo->computed_speed = servo->current_speed;
			}

			// Compute position for next cycle time
			servo->set_pos += servo->computed_speed * CYCLE_TIME;

			// Check if braking is needed
			if (servo->positive_direction) {
				if (servo->next_stop - servo->set_pos < get_breaking_distance(servo)) {
					servo->positioning = BRAKING;
				}
			} else {
				if (servo->next_stop - servo->set_pos > get_breaking_distance(servo)) {
					servo->positioning = BRAKING;
				}
			}
			break;

		case BRAKING:
			servo->computed_speed -= servo->current_acc * CYCLE_TIME;
			servo->set_pos += servo->computed_speed * CYCLE_TIME;
			servo->nominal_speed_reached = false;
			
			// Check if desired position has been reached
			if (servo->positive_direction) {
				if (servo->computed_speed <= 0.0) {
					servo->positioning = POSITION_REACHED;
				}
			} else {
				if (servo->computed_speed >= 0.0) {
					servo->positioning = POSITION_REACHED;
				}
			}
			break;

		case POSITION_REACHED:
			servo->set_pos = servo->next_stop;
			servo->positioning = IDLE;
			break;
	}
}

void servo_reset_all(servo_t* const servo) {
	pid_reset_all(servo->pid_pos);
	pid_reset_all(servo->pid_vel);
	servo->positioning = IDLE;
	servo->pos_error_internal = false;
	servo->computed_speed = 0.0;
	servo->set_pos = servo->enc_position;
}

void servo_compute(servo_t* const servo) { 
	// Get current position, calculate velocity
	int32_t enc_new = quadrature_encoder_get_count(pio0, servo->sm);
	servo->enc_position = ((float)enc_new / 4000.0) - servo->enc_offset;
	if (servo->set_zero) {
		servo->enc_offset = servo->enc_position;
		servo->enc_position = 0.0;
		pid_reset_all(servo->pid_pos);
		pid_reset_all(servo->pid_vel);
		servo->set_pos = 0.0;
		servo->set_zero = false;
	}
	servo->enc_speed = enc2speed(enc_new - servo->enc_old);
	servo->enc_old = enc_new; // Needed for velocity calculation

	if (*servo->enable) {
		// Reset All on positive edge of enable
		if (servo->enable_previous) {
			servo->enable_previous = false;
			servo_reset_all(servo);
		}

		// Evaluate following error
		if (fabs(servo->enc_position - servo->set_pos) >= FOLLOWING_ERROR)
			servo->pos_error_internal = true;

		next_positon_compute(servo);

		// PID Computation
		pid_compute(servo->pid_pos);
		servo->set_vel = servo->out_pos; // Positional --> Velocity PID
		pid_compute(servo->pid_vel);
		
		// set_two_chans_pwm(servo->pwm_slice, servo->out_vel);
		if (!*servo->error && servo->pos_error_internal) {
			strcpy(*servo->error_message, servo->servo_name);
			strcat(*servo->error_message, ": Pos Error");
			*servo->error = true;
		}

		if (!*servo->error && (pid_get_error(servo->pid_pos) || pid_get_error(servo->pid_vel))) {
			strcpy(*servo->error_message, servo->servo_name);
			strcat(*servo->error_message, ": PID Error");
			*servo->error = true;
		}

		// PWM output
		set_two_chans_pwm(servo->pwm_slice, servo->out_vel);
	} else {
		set_two_chans_pwm(servo->pwm_slice, 0.0);
		servo->enable_previous = true;
	}
	servo->servo_position = servo->enc_position * servo->scale;
	servo->servo_speed = servo->enc_speed * servo->scale;
}  

void _servo_goto(servo_t* const servo, const float position, const float speed) {
	servo->next_stop = position / servo->scale;
	servo->nominal_speed = speed / servo->scale;
	if (servo->delay_start == 0) {
		servo->delay_start = 500;
	}
	else if (servo->delay_start == UINT32_MAX) { // TODO, add some sign to not add delay, e.g. maximum number
		servo->delay_start = 0;
	}
	servo->positioning = REQUESTED;
}

void servo_goto_delayed(servo_t* const servo, const float position, const float speed, const uint32_t delay) {
	servo->delay_start = delay;
	_servo_goto(servo, position, speed);
}

void servo_goto(servo_t* const servo, const float position, const float speed) {
	servo->delay_start = UINT32_MAX;
	_servo_goto(servo, position, speed);
}

void servo_manual_handling(servo_t* const servo, const float min, const float max, const float speed, bool homed) {
	float limit_min;
	float limit_max;

	if (homed) {
		limit_min = min;
		limit_max = max; 
	}
	else {
		limit_min = -2000;
		limit_max = 2000;
	}
	if (button_raised(servo->man_plus)) {
		servo->delay_start = UINT32_MAX;
		servo_goto(servo, servo->next_stop = limit_max, speed);
	}
	else if (button_raised(servo->man_minus)) {
		servo->delay_start = UINT32_MAX;
		servo_goto(servo, servo->next_stop = limit_min, speed);
	}
	else if (button_dropped(servo->man_plus) || button_dropped(servo->man_minus)) {
		servo_stop_positioning(servo);
	}
}

float servo_get_position(const servo_t* const servo) {
	return servo->servo_position;
}

float* servo_get_position_pointer(servo_t* const servo) {
	return &servo->servo_position;
}

bool servo_is_idle(const servo_t* const servo){
	return servo->positioning == IDLE;
}

bool servo_is_accelerating(const servo_t* const servo) {
	return servo->positioning == ACCELERATING;
}

bool servo_is_position_reached(const servo_t* const servo) {
	return servo->positioning == POSITION_REACHED;
}

bool servo_is_speed_reached(const servo_t* const servo) {
	return servo->nominal_speed_reached;
}

void servo_set_stop_position(servo_t* const servo, const float position) {
	servo->next_stop = position / servo->scale;
}

void servo_set_zero_position(servo_t* const servo) {
	servo->set_zero = true;
}
