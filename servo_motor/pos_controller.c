#include "pos_controller.h"

posc_t pos_control_create(posc_t pos, float* out, float acc, float speed)
{
	// Controller state
	pos->positioning_request = false;
	pos->in_positioning = false;
	pos->positioning_done = false;

	// IO
	pos->acc = acc;
	pos->speed = speed;
	pos->out_pos = out;

	return pos;
}


float pos_compute(posc_t pos, float in_pos)
{
	// Request for positioning received
	if (pos->positioning_request == true && pos->in_positioning == false)
	{
		pos->start_time = time_us_64();		// save the time of beginning
		compute_path(pos, in_pos);
		pos->positioning_request = false;
		pos->in_positioning = true;
	}

	// Get current time 
	pos->current_cycle_time = (float)(time_us_64() - pos->last_start_time) * 0.001;
	pos->last_start_time = time_us_64();

	pos->ramp_time = (float)(pos->last_start_time - pos->start_time) * 1.0e-6;
	
	if (pos->in_positioning == true)
	{
		if (pos->ramp_time < pos->t_ramp)
		{
			(*pos->out_pos) = (0.5 * pos->acc * pow(pos->ramp_time, 2));
		}
		else if (pos->ramp_time >= pos->t_ramp && pos->ramp_time < (pos->t_ramp + pos->t_const))
		{
			(*pos->out_pos) = (pos->speed * pos->ramp_time - pos->s_ramp);
		}
		else if (pos->ramp_time >= (pos->t_ramp + pos->t_const) && pos->ramp_time < (pos->t_ramp * 2) + pos->t_const)
		{
			(*pos->out_pos) = (pos->speed * pos->ramp_time - pos->s_ramp) - (0.5 * pos->acc * pow(pos->ramp_time - pos->t_ramp - pos->t_const, 2));
		}
		else if (pos->ramp_time >= (pos->t_ramp * 2) + pos->t_const)
		{   
			(*pos->out_pos) = pos->requested_pos;
		}
	}
	return *pos->out_pos;
}

float speed_compute(posc_t pos, bool run, float in_speed)
{
	if (run == true)
	{
		if (in_speed >= pos->speed)
		{
			return pos->speed;
		}
		else
		{
			return in_speed + pos->acc * pos->current_cycle_time;
		}
	}
	else
	{
		if (in_speed <= 0.0)
		{
			return 0.0;
		}
		else
		{
			return in_speed - pos->acc * pos->current_cycle_time;
		}
	}
}



void compute_path(posc_t pos, float input_pos)
{
	// Input_pos is for continue movement

	// Main path parameters
	pos->t_ramp = pos->speed / pos->acc;			// duration of acceleration(decceleration) 
	pos->s_ramp = 0.5 * pos->speed * pos->t_ramp;	// distance during acc(decc)
	pos->s_conts = pos->requested_pos - (pos->s_ramp * 2);	// distance traveled during constant speed movement
	pos->t_const = pos->s_conts / pos->speed;		// duration of constant speed movement

	// Check if it can reach the given speed at the requested distance
	if (pos->s_ramp * 2 > pos->requested_pos)
	{
		// Movement will constist only by acceleration a decceleration movemnt
		pos->t_ramp = sqrt(pos->requested_pos / pos->acc);
		pos->s_ramp = pos->requested_pos / 2;
		pos->s_conts = 0.0;
		pos->t_const = 0.0;
	}

}
