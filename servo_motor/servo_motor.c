#include "servo_motor.h"
#include <stdlib.h>
#include <string.h>

servo_t servo_create(char (*servo_name)[10], uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin, float scale, enum mode mode, 
							button_t *man_plus, button_t *man_minus, bool *enable, bool *error, char (*message)[16])
{
	// Create servo data structure
	servo_t servo = (servo_t)malloc(sizeof(struct servo_motor));
	servo->servo_name = servo_name;
	servo->enable = enable;
	servo->enable_previous = false;

	// Encoder
	quadrature_encoder_program_init(pio0, sm, pio_ofset, encoder_pin, 0);
	servo->sm = sm;
	servo->scale = scale;
	
	servo->enc_old = 0;
			
	// PWM
	servo->pwm_slice = pwm_chan_init(pwm_pin);

	// PID
	servo->pid_pos = pid_create(&servo->current_pos, &servo->out_pos, &servo->set_pos, 50.0, 0.0, 0.0);
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
	servo->posError = error;
	servo->error_message = message;
	strcpy(*servo->error_message, "OK");
	servo->pos_error_internal = false;

	/**
	 * @brief Error code variable
	 * 
	 * @note 01 - following error
	 * @note 02 - max position overrun
	 * @note 03 - min position overrun
	 */

	// Feeder
	servo->no_of_stops = 0;
	servo->movement_in_progress = false;
	
	// Initial mode
	servo->mode = mode;

	// Buttons 
	servo->man_plus = man_plus;
	servo->man_minus = man_minus;

	// Temporary hardcoded 0 position
	servo->set_pos = 0;

	return servo;
}

void servo_compute(servo_t servo, float cycle_time)
{ 
	// Get current position, calculate velocity
	int32_t enc_new = quadrature_encoder_get_count(pio0, servo->sm);
	servo->current_pos = ((float)enc_new / 4000.0);
	servo->current_vel = enc2speed(enc_new - servo->enc_old, cycle_time);
	servo->enc_old = enc_new; // N eeded for velocity calculation

	if (*servo->enable) {
		if (servo->enable_previous) {
			servo->enable_previous = false;
			servo_reset_all(servo);
		}

		// Evaluate following error
		if (fabs(servo->current_pos - servo->set_pos) >= FOLLOWING_ERROR)
			servo->pos_error_internal = true;
		
		// TEMPORARY
		// Trigger the action by button
		if ((*servo->man_plus)->state_raised == 1) {
			servo_goto(servo, servo->next_stop = servo->current_pos + 20.0, 20.0);
		}

		if ((*servo->man_minus)->state_raised == 1) {
			servo_goto(servo, servo->next_stop = servo->current_pos - 20.0, 5.0);
		}

		// Current delta
		servo->cycle_time = cycle_time;

		robust_pos_compute(servo);

		// PID Computation
		pid_compute(servo->pid_pos);
		servo->set_vel = servo->out_pos; // Positional --> Velocity PID
		pid_compute(servo->pid_vel);
		
		// set_two_chans_pwm(servo->pwm_slice, servo->out_vel);
		if (*servo->posError == 0 && servo->pos_error_internal == 1) {
			strcpy(*servo->error_message, *servo->servo_name);
			strcat(*servo->error_message, ": Pos Error");
			*servo->posError = true;
		}

		// PWM output
		set_two_chans_pwm(servo->pwm_slice, servo->out_vel);
	}
	else {
		set_two_chans_pwm(servo->pwm_slice, 0.0);
		servo->enable_previous = true;
	}
}  

float enc2speed(int32_t enc_diff, float current_cycle){
	// Time difference of measured encoder tics
	// is ~1 milisecond but we want to get it in seconds,
	// so we have to multiply by 1000
    
	// 4000 - encoder tics per rev

	// So for speed we have to multiply by 1000 and divide by 4000
	// (enc_diff * 1000.0) / 4000.0
	
	// return (float)enc_diff / 4.0;
	return (float)enc_diff * (1.0 / current_cycle) / 4000.0;
}

void servo_goto(servo_t servo, float position, float speed){
	servo->next_stop = position;
	servo->nominal_speed = speed;
	servo->movement_request = true;
}

void robust_pos_compute(servo_t servo)
{
	// First occurence of movement request, save the position of movement beginning
	if (servo->movement_request == true) {
		if (servo->next_stop >= servo->current_pos) {
			// Positive direction
			servo->positive_direction = true;
			servo->current_acc = servo->nominal_acc;
			servo->current_speed = servo->nominal_speed;
			// servo->current_acc = 100.0;
			// servo->current_speed = 20.0;
		}
		else {
			// Negative direction
			servo->positive_direction = false;
			servo->current_acc = -servo->nominal_acc;
			servo->current_speed = -servo->nominal_speed;
			// servo->current_acc = -100.0;
			// servo->current_speed = -20.0;
		}
		servo->braking = false;
		servo->movement_request = false;
		servo->movement_in_progress = true;
	}

	// Periodic computation of new position 
	if (servo->movement_in_progress == true) {
		if (servo->braking == false) {
			// Accelerating
			servo->computed_speed += servo->current_acc * servo->cycle_time;

			// check if nominal speed has been reached
			if (fabs(servo->computed_speed) > fabs(servo->current_speed)) {
				servo->computed_speed = servo->current_speed;
			}

			// Compute position for next cycle time
			servo->set_pos += servo->computed_speed * servo->cycle_time;

			// Check if braking is needed
			if (servo->positive_direction) {
				if (servo->next_stop - servo->set_pos < get_breaking_distance(servo)) {
					servo->braking = true;
				}
			}
			else {
				if (servo->next_stop - servo->set_pos > get_breaking_distance(servo)) {
					servo->braking = true;
				}
			}
		}
		else {
			// Braking
			servo->computed_speed -= servo->current_acc * servo->cycle_time;
			servo->set_pos += servo->computed_speed * servo->cycle_time;
			
			// Check if desired position has been reached
			if (fabs(servo->set_pos - servo->current_pos) < 0.01) {
				servo->set_pos = servo->next_stop;
				servo->movement_in_progress = false;
			}
		}

		// Something like emergency stop
		if (fabs(servo->current_pos) > 200.0) {
			servo->movement_in_progress = false;
		}
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

void servo_reset_all(servo_t servo) {
	pid_reset_all(servo->pid_pos);
	pid_reset_all(servo->pid_vel);
	servo->movement_request = false;
	servo->movement_in_progress = false;
	servo->pos_error_internal = false;
	servo->computed_speed = 0.0;
	servo->set_pos = servo->current_pos;
}
