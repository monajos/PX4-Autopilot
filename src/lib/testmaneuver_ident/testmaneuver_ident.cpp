/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "testmaneuver_ident.hpp"

bool
Testmaneuver_ident::init_time()
{
	_time   = 0.0f;
	return true;
}

bool
Testmaneuver_ident::ongoing_5s()
{
	if (_time> 5.0f)
	{
		return false;
	}
	else {
		return true;
	}
}

bool
Testmaneuver_ident::is_active()
{
	if (_is_active)
	{
		return true;
	}
	else {
		return false;
	}
}

bool
Testmaneuver_ident::reset_trajectory()
{
	if (_is_active) {
		return false;
	}

	_time = 0.0f;
	_spd_sp = 0.0f;
	_hgt_sp = 0.0f;

	_spd_coeff_a = 0.0f;
	_spd_coeff_b = 0.0f;
	_hgt_coeff_a = 0.0f;
	_hgt_coeff_b = 0.0f;

	_is_init = false;
	_is_active = false;
	return true;
}

float
Testmaneuver_ident::get_test_spd_sp()
{
	return _spd_sp;
}

float
Testmaneuver_ident::get_test_hgt_sp()
{
	return _hgt_sp;
}


float
Testmaneuver_ident::elevator_doublet(float dt, float scale, float man_dt)
{
	_time += dt;
	if (_time > 0.0f && _time < man_dt) {
		/*positive deflection*/
		return scale;
	} else if (_time >= man_dt && _time < 2*man_dt) {
		/*negative deflection*/
		return -scale;
	} else {
		_is_active = false;
		return 0.0f;
	}
}
float
Testmaneuver_ident::elevator_multistep(float dt, float scale, float man_dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 3*man_dt) {
		return scale;
	} else if (_time >= 3*man_dt && _time < 5*man_dt) {
		return -scale;
	} else if (_time >= 5*man_dt && _time < 6*man_dt) {
		return scale;
	} else if (_time >= 6*man_dt && _time < 7*man_dt) {
		return -scale;
	} else if (_time >= 7*man_dt && _time < 10*man_dt) {
		return 0.0f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::elevator_pulse(float dt, float scale, float man_dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < man_dt) {
		return scale;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::bank_to_bank_30(float dt, float scale)
{
	_time += dt;
	if (_time >= 0.0f && _time < 1.0f) {
		return scale;
	} else if (_time >= 1.0f && _time < 2.8f) {
		return -scale;
	} else if (_time >= 2.8f && _time < 3.5f) {
		return scale;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::bank_to_bank_50(float dt, float scale)
{
	_time += dt;
	if (_time >= 0.0f && _time < 1.6f) {
		return scale;
	} else if (_time >= 1.6f && _time < 3.8f) {
		return -scale;
	} else if (_time >= 3.8f && _time < 4.5f) {
		return scale;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::rudder_doublet(float dt, float scale, float man_dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < man_dt) {
		return scale;
	} else if (_time >= man_dt && _time < 2*man_dt) {
		return -scale;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::rudder_pulses(float dt, float scale, float man_dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < man_dt) {
		return scale;
	} else if (_time >= man_dt && _time < man_dt + 1.0f) {
		return 0.0f;
	} else if (_time >= (man_dt+1.0f) && _time < (2*man_dt+1.0f)) {
		return -scale;
	} else {
		_is_active = false;
		return 0.0f;
	}
}
float
Testmaneuver_ident::thrust_variation(float dt,float throttle_before, float man_dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < man_dt) {
		return 0.8f; // throttle_before * 0.1f;
	} else if (_time >= man_dt && _time < 2*man_dt) {
		return throttle_before * -0.1f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
