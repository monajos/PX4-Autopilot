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

#include "testmaneuver.hpp"

bool
Testmaneuver::init_trajectory()
{
	if(_is_active) {
		return false;
	}
	_time   = 0.0f;

	// == INITIALISE COEFFICIENTS
	_spd_coeff_a = -3.0f*_rel_spd_sp/(_spd_rise_time*_spd_rise_time*_spd_rise_time*_spd_rise_time);
	_spd_coeff_b = 4.0f*_rel_spd_sp/(_spd_rise_time*_spd_rise_time*_spd_rise_time);

	_hgt_coeff_a = -3.0f*_rel_hgt_sp/(_hgt_rise_time*_hgt_rise_time*_hgt_rise_time*_hgt_rise_time);
	_hgt_coeff_b = 4.0f*_rel_hgt_sp/(_hgt_rise_time*_hgt_rise_time*_hgt_rise_time);
	return true;
}

void
Testmaneuver::update_trajectory(float dt)
{
	_spd_sp = _base_spd_sp + update_spd();
	_hgt_sp = _base_hgt_sp + update_hgt();
	_time += dt;
}

bool
Testmaneuver::reset_trajectory()
{
	if(_is_active) {
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
Testmaneuver::get_test_spd_sp()
{
	return _spd_sp;
}

float
Testmaneuver::get_test_hgt_sp()
{
	return _hgt_sp;
}

float
Testmaneuver::update_spd()
{
	if(_time > _hgt_rise_time) {
		return _rel_hgt_sp;
	}
	else
	{
		return (_spd_coeff_a*_time*_time*_time*_time + _spd_coeff_b*_time*_time*_time);
	}

}

float
Testmaneuver::update_hgt()
{
	if(_time > _hgt_rise_time) {
		return _rel_hgt_sp;
	}
	else
	{
		return (_hgt_coeff_a*_time*_time*_time*_time + _hgt_coeff_b*_time*_time*_time);
	}
}
