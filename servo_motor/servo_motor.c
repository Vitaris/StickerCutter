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

servo_t servo_motor_create(servo_t motor, uint pio_ofset, uint sm, uint encoder_pin, uint pwm_pin)
{
	// Encoder
	quadrature_encoder_program_init(pio0, sm, pio_ofset, encoder_pin, 0);
	motor->sm = sm;
	
	motor->enc_old = 0;
			
	// PWM
	// motor->pwm_slice = pwm_chan_init(pwm_pin);

	// PID
	// BEST!
	// float kp_speed = 5.0, ki_speed = 4.0, kd_speed = 3.0;
	// float kp_pos = 50.0, ki_pos = 0.0, kd_pos = 0.0;
	motor->pid_pos = pid_create(&motor->ctrldata_pos, &motor->in_pos, &motor->out_pos, &motor->set_pos, 50.0, 0.0, 0.0);
	motor->pid_vel = pid_create(&motor->ctrldata_vel, &motor->in_vel, &motor->out_vel, &motor->set_vel, 5.0, 4.0, 3.0);

	// Positional controller
	motor->pos = pos_control_create(&motor->pos_ctrData, &motor->generated_pos, 200.0, 30.0);


	return motor;
}

void motor_compute(servo_t motor)
{
	// Encoder
	int32_t enc_new = quadrature_encoder_get_count(pio0, motor->sm);
	motor->in_pos = ((float)enc_new / 4000.0);
	motor->in_vel = enc2speed(enc_new - motor->enc_old);
	motor->enc_old = enc_new;
	

	// Positional controller
	motor->set_pos = pos_compute(motor->pos, motor->enc_position);

	// PID
	pid_compute(motor->pid_pos);

	motor->set_vel = motor->out_pos;
	pid_compute(motor->pid_vel);

	// PWM
	//set_two_chans_pwm(motor->pwm_slice,motor->out_vel);
}

float enc2speed(int32_t enc_diff){
    // 1000 - 1ms
    // 4000 - encoder tics per rev
    return ((enc_diff * 1000.0) / 4000.0);
}

void motor_goto(servo_t motor, float position, float speed){
	pos_goto(motor->pos, position, speed);
}

