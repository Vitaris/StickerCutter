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
#include "pos_controller.h"

posc_t pos_control_create(posc_t pos, float* in_speed, float* out_pos, float acc, float speed)
{
	// Controller state
	pos->positioning_request = false;
	pos->in_positioning = false;
	pos->positioning_done = false;


	return pos;
}


void pos_compute(posc_t pos)
{
	
}

void pos_goto(posc_t pos, float position)
{
	pos->requested_pos = position;
	pos->positioning_request = true;
}