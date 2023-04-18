/*	Positional controller for Microcontrollers
	Copyright (C) 2022

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "servo_motor.h"

servo_t servo_create(servo_t servo, uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin, float scale, bool mode, 
							bool *man_plus, bool *man_minus)
{
	// Encoder
	quadrature_encoder_program_init(pio0, sm, pio_ofset, encoder_pin, 0);
	servo->sm = sm;
	servo->scale = scale;
	
	servo->enc_old = 0;
			
	// PWM
	// servo->pwm_slice = pwm_chan_init(pwm_pin);

	// PID
	// BEST!
	// float kp_speed = 5.0, ki_speed = 4.0, kd_speed = 3.0;
	// float kp_pos = 50.0, ki_pos = 0.0, kd_pos = 0.0;
	servo->pid_pos = pid_create(&servo->ctrldata_pos, &servo->current_pos, &servo->out_pos, &servo->set_pos, 50.0, 0.0, 0.0);
	servo->pid_vel = pid_create(&servo->ctrldata_vel, &servo->current_vel, &servo->out_vel, &servo->set_vel, 5.0, 4.0, 3.0);

	// Positional controller
	servo->nominal_acc = 200.0;
	servo->nominal_speed = 30.0;

	// Feeder
	servo->no_of_stops = 0;
	
	// Initial mode
	servo->mode = MAN;

	// Buttons
	servo->man_plus = man_plus;
	servo->man_minus = man_minus;

	return servo;
}

void servo_compute(servo_t servo)
{
	// Encoder
	// Get current position, velocity
	int32_t enc_new = quadrature_encoder_get_count(pio0, servo->sm);
	servo->current_pos = ((float)enc_new / 4000.0);
	servo->current_vel = enc2speed(enc_new - servo->enc_old);
	servo->enc_old = enc_new;

	// Get current time 
	servo->current_cycle_time = (float)(time_us_64() - servo->current_time) * 0.001;
	servo->current_time = time_us_64();

	switch (servo->mode)
	{
		case POSITIONER:
			// Positional controller
			servo->set_pos = pos_compute(servo, servo->current_pos);

			// PID
			pid_compute(servo->pid_pos);

			servo->set_vel = servo->out_pos;
			pid_compute(servo->pid_vel);

		case FEEDER:
			

		case MANUAL:
			// Manual mode
			servo->set_vel = speed_compute(servo, *servo->man_plus, *servo->man_minus);
			// servo->set_vel = 0.0;
			pid_compute(servo->pid_vel);
		
	}
	

	// PWM
	//set_two_chans_pwm(servo->pwm_slice,servo->out_vel);
}  

float enc2speed(int32_t enc_diff){
	// Time difference of measured encoder tics
	// is ~1 milisecond but we want to get it in seconds,
	// so we have to multiply by 1000
    
	// 4000 - encoder tics per rev

	// So for speed we have to multiply by 1000 and divide by 4000
	// (enc_diff * 1000.0) / 4000.0
	
    return (float)enc_diff / 4.0;
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

float pos_compute_2(servo_t servo, float current_pos)
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
	return servo->out_pos;
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
		return 5.0;

	}
	else if (minus == true)
	{
		return -5.0;
	}
	else
	{
		return 0.0;
	}
}

void add_stop(servo_t servo)
{
	// add stop
    servo->stops[servo->no_of_stops] = servo->current_pos + CUT_OFFSET;
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

