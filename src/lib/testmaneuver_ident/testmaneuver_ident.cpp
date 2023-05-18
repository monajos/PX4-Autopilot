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
Testmaneuver_ident::elevator_doublet(float dt)
{
	_time += dt;
	if (_time > 0.0f && _time < 1.0f) {
		/*positive deflection*/
		return 0.2f;
	} else if (_time >= 1.0f && _time < 2.0f) {
		/*negative deflection*/
		return -0.2f;
	//} else if (_time < 0.000001f && _time > -0.000001f){
	//	return 0.0f;
	} else { //if ( _time >= 2.0f && _time < 5.0f)
		/*initial*/
		_is_active = false;
		return 0.0f;
	}
}
float
Testmaneuver_ident::elevator_multistep(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 1.3f) {
		return 0.1f;
	} else if (_time >= 1.3f && _time < 2.3f) {
		return -0.1f;
	} else if (_time >= 2.3f && _time < 3.0f) {
		return 0.1f;
	} else if (_time >= 3.0f && _time < 3.7f) {
		return -0.1f;
	} else if (_time >= 3.7f && _time < 4.0f) {
		return 0.0f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::elevator_pulse(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 3.0f) {
		return 0.05f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::bank_to_bank_30(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 1.0f) {
		return 0.3f;
	} else if (_time >= 1.0f && _time < 2.8f) {
		return -0.3f;
	} else if (_time >= 2.8f && _time < 3.5f) {
		return 0.3f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::bank_to_bank_50(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 1.6f) {
		return 0.3f;
	} else if (_time >= 1.6f && _time < 3.8f) {
		return -0.3f;
	} else if (_time >= 3.8f && _time < 4.5f) {
		return 0.3f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::rudder_doublet(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 1.0f) {
		return 0.4f;
	} else if (_time >= 1.0f && _time < 2.0f) {
		return -0.4f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::rudder_pulses(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 5.0f) {
		return 0.1f;
	} else if (_time >= 5.0f && _time < 8.0f) {
		return 0.0f;
	} else if (_time >= 8.0f && _time < 13.0f) {
		return -0.1f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
float
Testmaneuver_ident::thrust_variation(float dt)
{
	_time += dt;
	if (_time >= 0.0f && _time < 5.0f) {
		return 0.1f;
	} else if (_time >= 5.0f && _time < 10.0f) {
		return -0.1f;
	} else {
		_is_active = false;
		return 0.0f;  // Return default value for times outside defined range
	}
}
