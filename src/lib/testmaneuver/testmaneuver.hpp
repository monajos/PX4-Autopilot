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

/*
 * @file testmaneuver.h
 *
 * @author Christopher Ruwisch <christopher.ruwisch@gmail.com>
 */

#ifndef TESTMANEUVER_H_
#define TESTMANEUVER_H_

#include <math.h>
#include <stdint.h>

class Testmaneuver
{
public:
	Testmaneuver() = default;
	~Testmaneuver() = default;

	void set_base_spd_sp(float base_spd_sp) { _base_spd_sp = base_spd_sp; }
	void set_base_hgt_sp(float base_hgt_sp) { _base_hgt_sp = base_hgt_sp; }
	void set_rel_spd_sp(float rel_spd_sp) { _rel_spd_sp = rel_spd_sp; }
	void set_rel_hgt_sp(float rel_hgt_sp) { _rel_hgt_sp = rel_hgt_sp; }
	void set_spd_rise_time(float spd_rise_time) { _spd_rise_time = spd_rise_time; }
	void set_hgt_rise_time(float hgt_rise_time) { _hgt_rise_time = hgt_rise_time; }

	bool init_trajectory();
	void update_trajectory(float dt);
	bool reset_trajectory();

	float get_test_spd_sp();
	float get_test_hgt_sp();


private:
	bool _is_active{false};
	bool _is_init{false};

	float _base_spd_sp{0.0f};
	float _base_hgt_sp{0.0f};

	float _rel_spd_sp{0.0f};
	float _rel_hgt_sp{0.0f};

	float _spd_rise_time{0.0f};
	float _hgt_rise_time{0.0f};


	float _spd_sp{0.0f};
	float _hgt_sp{0.0f};

	float _time{0.0f};

	float _spd_coeff_a{0.0f};
	float _spd_coeff_b{0.0f};

	float _hgt_coeff_a{0.0f};
	float _hgt_coeff_b{0.0f};

	float update_spd();
	float update_hgt();
};





#endif /* TESTMANEUVER_H_ */
