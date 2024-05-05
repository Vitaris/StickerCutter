#include "servo_motor.h"
#include <stdlib.h>
#include <string.h>

servo_t servo_create(char (*servo_name)[10], uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin, float scale, enum mode mode, 
							button_t *man_plus, button_t *man_minus, bool *error, char (*message)[16])
{
	// Create servo data structure
	servo_t servo = (servo_t)malloc(sizeof(struct servo_motor));
	servo->servo_name = servo_name;

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
	servo->nominal_acc = 20.0;
	servo->nominal_speed = 20.0;
	servo->last_speed = 0.0;
	servo->enc_position = 0.0;
	servo->enc_old = 0;

	// Limits
	servo->pos_limit_enabled = true;
	servo->max_diff = 0.5; // Max following error
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
	
	// Initial mode
	servo->mode = mode;

	// Buttons
	servo->man_plus = man_plus;
	servo->man_minus = man_minus;

	servo->a = 0;
	servo->b = 0;
	servo->edge_1 = false;
	servo->edge_2 = false;
	servo->edge_3 = false;
	// servo->movement_request = true;
	return servo;
}

void servo_compute(servo_t servo, float cycle_time)
{
	// Encoder
	// Get current position, calculate velocity
	int32_t enc_new = quadrature_encoder_get_count(pio0, servo->sm);
	servo->current_pos = ((float)enc_new / 4000.0);
	servo->current_vel = enc2speed(enc_new - servo->enc_old, cycle_time);
	servo->enc_old = enc_new; // Needed for velocity calculation

	// Evaluate following error
	if (fabs(servo->current_pos - servo->set_pos) >= FOLLOWING_ERROR)
		servo->pos_error_internal = true;

	// Get current time 
	servo->current_cycle_time = (float)(time_us_64() - servo->current_time) * 0.001;
	servo->current_time = time_us_64();

	// Current delta
	servo->cycle_time = cycle_time;

	// Temporary hardcoded 0 position
	servo->set_pos = 0;

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
	if (*servo->posError == 1) {
		set_two_chans_pwm(servo->pwm_slice, 0.0);
	}
	else {
		set_two_chans_pwm(servo->pwm_slice, servo->out_vel);
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
	servo->requested_pos = position;
	servo->nominal_speed = speed;
	servo->positioning_request = true;
}

void compute_path_params(servo_t servo)
{
	// Main path parameters computation
	servo->t_ramp = servo->nominal_speed / servo->nominal_acc;		// duration of acceleration(decceleration) 
	servo->s_ramp = 0.5 * servo->nominal_speed * servo->t_ramp;		// distance traveled during acc(decc)
	servo->s_conts = servo->requested_pos - (servo->s_ramp * 2);	// distance travelled during constant speed movement
	servo->t_const = servo->s_conts / servo->nominal_speed;			// duration of constant speed movement

	// Check if it can reach the given speed at the requested distance
	if (servo->s_ramp * 2 > servo->requested_pos)
	{
		// Movement will constist only by acceleration a decceleration movemnt
		servo->t_ramp = sqrt(servo->requested_pos / servo->nominal_acc);
		servo->s_ramp = servo->requested_pos / 2;
		servo->s_conts = 0.0;
		servo->t_const = 0.0;
	}

}

float pos_compute(servo_t servo, float current_pos)
{
	// Request for positioning received
	if (servo->positioning_request == true && servo->state == POSITIONING_DONE)
	{
		servo->start_time = time_us_64();		// save the time of beginning
		compute_path_params(servo);
		servo->positioning_request = false;
		servo->state = IN_POSITIONING;
	}

	servo->ramp_time = (float)(servo->current_time - servo->start_time) * 1.0e-6;
	
	if (servo->state == IN_POSITIONING)
	{
		//  Acceleration
		if (servo->ramp_time < servo->t_ramp)
		{
			(servo->out_pos) = (0.5 * servo->nominal_acc * pow(servo->ramp_time, 2));
		}
		// Constant speed
		else if (servo->ramp_time >= servo->t_ramp && servo->ramp_time < (servo->t_ramp + servo->t_const))
		{
			(servo->out_pos) = (servo->nominal_speed * servo->ramp_time - servo->s_ramp);
		}
		// Decceleration
		else if (servo->ramp_time >= (servo->t_ramp + servo->t_const) && servo->ramp_time < (servo->t_ramp * 2) + servo->t_const)
		{
			(servo->out_pos) = (servo->nominal_speed * servo->ramp_time - servo->s_ramp) - (0.5 * servo->nominal_acc * pow(servo->ramp_time - servo->t_ramp - servo->t_const, 2));
		}
		// Stand still
		else if (servo->ramp_time >= (servo->t_ramp * 2) + servo->t_const)
		{   
			(servo->out_pos) = servo->requested_pos;
			servo->state = POSITIONING_DONE;
		}
	}
	return servo->out_pos;
}

float pos_compute_2(servo_t servo, float delta_time, float current_pos)
{
	// Request for positioning received
	if (servo->positioning_request == true && servo->state == POSITIONING_DONE)
	{
		servo->start_time = time_us_64();		// save the time of beginning
		compute_path_params(servo);
		servo->positioning_request = false;
		servo->state = IN_POSITIONING;
	}

	servo->ramp_time = (float)(servo->current_time - servo->start_time) * 1.0e-6;
	
	if (servo->state == IN_POSITIONING)
	{
		// Check if we can deccelerate to reach the requested position
		
		
	}

//************************************************************************************************
	float delta = 0.001 + (delta_time - 0.001);
	// delta = 0.001;

	if (servo->last_speed < servo->nominal_speed && servo->speed_reached == false)
	{
		// Ramp up
		servo->last_speed += (servo->nominal_speed * delta);
		servo->a++;
	}
	else if (servo->current_pos >= 60.0)
	{
		// Ramp down
		servo->last_speed -= (servo->nominal_speed * delta);

		if (servo->last_speed <= 0.0)
			servo->last_speed = 0.0;
	}
	else
	{
		servo->last_speed = servo->nominal_speed;
		servo->speed_reached = true;
		servo->edge_1 = true;
	}

	if (servo->edge_1 == true && servo->edge_2 == false)
	{
		servo->mem_speed = servo->last_speed;
		servo->mem_pos = servo->current_pos;
		servo->b = servo->a;
		servo->edge_2 = true;
	}

	return servo->set_pos + (servo->last_speed * delta);
}

float speed_compute(servo_t servo, bool plus, bool minus)
{
	// Both buttons should not be active at the same time
	// There may be a malfunction, so it is better to stop the motor

	if (plus == true && minus == true)
	{
		// Some error handling here
		return 0.0;
	}

	if (plus == true)
	{
		return MAN_SPEED;

	}
	else if (minus == true)
	{
		return -MAN_SPEED;
	}
	else
	{
		return 0.0;
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

float feeder(servo_t servo)
{
	// servo->movement_request = *servo->man_plus->state;
	// servo->breaking_request = *servo->man_minus->state;
	servo->movement_request = false;
	servo->breaking_request = false;

	if (servo->set_pos >= 50 && servo->edge_3 == false)
	{
		add_stop(servo);
		servo->edge_3 = true;
	}

	// Check if its needed to feed or break
	if (stop_ahead(servo) || servo->breaking_in_progress == true)
	{
		// do break
		return breaking_to(servo);

	}
	else
	{
		// do feed
		return continuous_feeding(servo);
	}

}



bool stop_ahead(servo_t servo)
{
	return get_breaking_distance(servo) >= get_dist_to_stop(servo) ? true : false;
}

float get_breaking_distance(servo_t servo)
{
	return 0.5 * (pow(servo->current_vel, 2) / servo->nominal_acc);
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

float continuous_feeding(servo_t servo)
{
	// Starting point of a feeding
	if (servo->movement_request == true && servo->movement_in_progress == false && servo->current_pos < 1.0)
	{
		// Save starting time
		servo->movement_start_time = time_us_64();		// save the time of beginning

		servo->acc_time = servo->nominal_speed / servo->nominal_acc;		// duration of acceleration(decceleration)
		servo->acc_dist = 0.5 * servo->nominal_speed * servo->acc_time;		// distance traveled during acc(decc)
		servo->begin_pos = servo->current_pos;

		// Set in_progress flag
		servo->movement_request = false;
		servo->movement_in_progress = true;
	}
	servo->acc_progress_time = (float)(servo->current_time - servo->movement_start_time) * 1.0e-6;

	if (servo->movement_in_progress == true)
	{
		// Ramp or continuous speed
		if (servo->set_pos < servo->begin_pos + servo->acc_dist)
		{
			// Ramp
			// return a computed next position 
			return servo->begin_pos + ( 0.5 * servo->nominal_acc * pow(servo->acc_progress_time, 2) );
		}
		else
		{
			// continuous speed
			// return a computed next position 
			return servo->set_pos + (servo->nominal_speed * servo->cycle_time);
		}
	}
	else
	{
		return servo->set_pos;
	}
}

float breaking_to(servo_t servo)
{
	// Starting point of a breaking, save some info
	// if (servo->breaking_request == true && servo->breaking_in_progress == false)
	if (servo->breaking_in_progress == false)
	{
		servo->breaking_start_time = time_us_64();		// save the time of beginning
		servo->begin_pos = servo->set_pos;

		servo->movement_in_progress = false;
		servo->breaking_in_progress = true;
	}
	servo->breaking_progress_time = (float)(servo->current_time - servo->breaking_start_time) * 1.0e-6;
	
	// Breaking
	if (servo->breaking_in_progress == true)
	{
		// Break
		if (servo->set_pos < servo->stops[0])
		{
			return servo->begin_pos + ( 0.5 * -servo->nominal_acc * pow(servo->breaking_progress_time - servo->acc_time, 2) + servo->acc_dist);
		}
		else
		{
			servo->breaking_in_progress = false;
			float stop_pos = servo->stops[0];
			remove_stop(servo);
			return stop_pos;
		}
		
	}
	else
	{
		return servo->set_pos;
	}

}

float accelerate(servo_t servo)
{

}

void check_for_following_error(servo_t* servo)
{
	
}
