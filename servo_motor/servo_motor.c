#include "servo_motor.h"
#include <stdlib.h>
#include <string.h>

servo_t servo_create(char servo_name[10], uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin, float scale, 
							button_t *man_plus, button_t *man_minus, bool *enable, bool *error, char (*message)[21])
{
	// Create servo data structure
	servo_t servo = (servo_t)malloc(sizeof(struct servo_motor));
	strcpy(servo->servo_name, servo_name);
	// servo->servo_name = servo_name;
	servo->delay_start = 0;
	servo->delay_finish = 0;
	servo->enable = enable;
	servo->enable_previous = false;
	servo->positioning = IDLE;

	// Encoder
	quadrature_encoder_program_init(pio0, sm, pio_ofset, encoder_pin, 0);
	servo->sm = sm;
	servo->scale = scale;
	
	servo->enc_old = 0;
			
	// PWM
	servo->pwm_slice = pwm_chan_init(pwm_pin);

	// PID
	servo->pid_pos = pid_create(&servo->current_pos, &servo->out_pos, &servo->set_pos, 50.0, 0.0, 5.0);
	servo->pid_vel = pid_create(&servo->current_vel, &servo->out_vel, &servo->set_vel, 5.0, 4.0, 3.0);

	// Positional controller
	servo->nominal_acc = 300.0;
	servo->nominal_speed = 30.0;
	servo->enc_old = 0;
	servo->computed_speed = 0.0;

	// Limits
	servo->pos_limit_enabled = true;
	servo->max_pos = 100.0;
	servo->min_pos = -10.0;
	servo->error = error;
	servo->error_message = message;
	strcpy(*servo->error_message, "OK");
	servo->pos_error_internal = false;
	servo->offset = 0.0;
	servo->set_zero = false;

	/**
	 * @brief Error code variable
	 * 
	 * @note 01 - following error
	 * @note 02 - max position overrun
	 * @note 03 - min position overrun
	 */

	// Feeder
	servo->no_of_stops = 0;
	
	// Buttons 
	servo->man_plus = man_plus;
	servo->man_minus = man_minus;

	// Temporary hardcoded 0 position
	servo->set_pos = 0;

	return servo;
}

void servo_compute(servo_t servo)
{ 
	// Get current position, calculate velocity
	int32_t enc_new = quadrature_encoder_get_count(pio0, servo->sm);
	servo->current_pos = ((float)enc_new / 4000.0) - servo->offset;
	if (servo->set_zero) {
		servo->offset = servo->current_pos;
		servo->current_pos = 0.0;
		pid_reset_all(servo->pid_pos);
		pid_reset_all(servo->pid_vel);
		servo->set_pos = 0.0;
		servo->set_zero = false;
	}
	servo->current_vel = enc2speed(enc_new - servo->enc_old);
	servo->enc_old = enc_new; // N eeded for velocity calculation

	if (*servo->enable) {
		// Reset All on positive edge of enable
		if (servo->enable_previous) {
			servo->enable_previous = false;
			servo_reset_all(servo);
		}

		// Evaluate following error
		if (fabs(servo->current_pos - servo->set_pos) >= FOLLOWING_ERROR)
			servo->pos_error_internal = true;

		robust_pos_compute(servo);

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

		if (!*servo->error && (servo->pid_pos->error || servo->pid_vel->error)) {
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
}  

float enc2speed(int32_t enc_diff){
	// Time difference of measured encoder tics
	// is ~1 milisecond but we want to get it in seconds,
	// so we have to multiply by 1000
    
	// 4000 - encoder tics per rev

	// So for speed we have to multiply by 1000 and divide by 4000
	// (enc_diff * 1000.0) / 4000.0
	
	// return (float)enc_diff / 4.0;
	return (float)enc_diff * (1.0 / CYCLE_TIME) / 4000.0;
}

void servo_goto(servo_t servo, float position, float speed){
	servo->next_stop = position;
	servo->nominal_speed = speed;
	if (servo->delay_start == 0) { // TODO, add some sign to not add delay, e.g. maximum number
		servo->delay_start = 500;
	}
	servo->positioning = REQUESTED;
}

void robust_pos_compute(servo_t servo)
{
	switch(servo->positioning) {
        case IDLE:
			servo->movement_done = false;
			break;
		
		case REQUESTED:
			// First occurence of movement request, save the position of movement beginning
			if (servo->next_stop >= servo->current_pos) {
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
			servo->movement_done = true;
			servo->positioning = IDLE;
			break;
	}
}

void servo_manual_handling(servo_t  servo) {
	if ((*servo->man_plus)->state_raised) {
			servo_goto(servo, servo->next_stop = servo->current_pos + 500.0, 2.5);
		}

		if ((*servo->man_minus)->state_raised) {
			servo_goto(servo, servo->next_stop = servo->current_pos - 500.0, 2.5);
		}

		if ((*servo->man_plus)->state_dropped || (*servo->man_minus)->state_dropped) {
			servo->next_stop = servo->set_pos + get_breaking_distance(servo);
			servo->braking = true;
		}
}

void add_stop(servo_t servo)
{
	// add stop
    // servo->stops[servo->no_of_stops] = servo->set_pos + CUT_OFFSET;
	servo->stops[servo->no_of_stops] = 60.0;
    servo->no_of_stops++;
}

void remove_stop(servo_t servo)
{
	for (int i = 0; i < servo->no_of_stops; i++)
    {   
        servo->stops[i] = servo->stops[i+1];
    }
    servo->no_of_stops--;
}

bool stop_ahead(servo_t servo)
{
	return get_breaking_distance(servo) >= get_dist_to_stop(servo) ? true : false;
}

float get_breaking_distance(servo_t servo)
{
	return 0.5 * (pow(servo->computed_speed, 2) / servo->current_acc);
}

float get_dist_to_stop(servo_t servo)
{
	if (servo->no_of_stops > 0)
	{
		return servo->stops[0] - servo->current_pos;
	}
	else
	{
		// return big number, far away (~inf)
		return 10000.0;
	}
}

void set_zero(servo_t servo) {
	
}

void servo_reset_all(servo_t servo) {
	pid_reset_all(servo->pid_pos);
	pid_reset_all(servo->pid_vel);
	*servo->pid_vel->output = 0.0;
	servo->positioning = IDLE;
	servo->pos_error_internal = false;
	servo->computed_speed = 0.0;
	servo->set_pos = servo->current_pos;
}
