/****************************************************************************
 *
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "PI_X.hpp"

#include <lib/ecl/geo/geo.h>

#include <px4_platform_common/defines.h>



using math::constrain;
using math::max;
using math::min;
using math::radians;

static constexpr float DT_MIN = 0.001f;	///< minimum allowed value of _dt (sec)
static constexpr float DT_MAX = 1.0f;	///< max value of _dt allowed before a filter state reset is performed (sec)

/**
 * @file PI_X.cpp
 *
 * @author Henrik Spark
 */


void PI_X::init_integrator_throttle(float current_throttle){

	_speed_controller.integral = current_throttle / _speed_controller.ki ;

};

void PI_X::init_integrator_pitch(float current_pitch){

	_altitude_controller.integral = current_pitch / (_altitude_controller.ki) ;

};
/*
 * This function implements a complementary filter to estimate the climb rate when
 * inertial nav data is not available. It also calculates a true airspeed derivative
 * which is used by the airspeed complimentary filter.
 */
void PI_X::update_vehicle_state_estimates(float equivalent_airspeed, const float speed_deriv_forward,
		bool altitude_lock, bool in_air, float altitude, float vz)
{
	// calculate the time lapsed since the last update
	uint64_t now = hrt_absolute_time();
	float dt = fmaxf((now - _state_update_timestamp) * 1e-6f, DT_MIN);

	bool reset_altitude = false;

	if (_state_update_timestamp == 0 || dt > DT_MAX) {
		dt = DT_DEFAULT;
		reset_altitude = true;
	}

	if (!altitude_lock || !in_air) {
		reset_altitude = true;
	}

	if (reset_altitude) {
		_states_initialized = false;
	}

	_state_update_timestamp = now;
	_EAS = equivalent_airspeed;

	_in_air = in_air;

	// Set the velocity and position state to the the INS data
	_vert_vel_state = -vz;
	_vert_pos_state = altitude;

	// Update and average speed rate of change if airspeed is being measured
	if (PX4_ISFINITE(equivalent_airspeed) && airspeed_sensor_enabled()) {
		_tas_rate_raw = speed_deriv_forward;
		// Apply some noise filtering
		_TAS_rate_filter.update(speed_deriv_forward);
		_tas_rate_filtered = _TAS_rate_filter.getState();

	} else {
		_tas_rate_raw = 0.0f;
		_tas_rate_filtered = 0.0f;
	}

	if (!_in_air) {
		_states_initialized = false;
	}

}

void PI_X::_update_speed_states(float equivalent_airspeed_setpoint, float equivalent_airspeed, float EAS2TAS)
{
	// Calculate the time in seconds since the last update and use the default time step value if out of bounds
	uint64_t now = hrt_absolute_time();
	const float dt = constrain((now - _speed_update_timestamp) * 1.0e-6f, DT_MIN, DT_MAX);

	// Convert equivalent airspeed quantities to true airspeed
	_EAS_setpoint = equivalent_airspeed_setpoint;

	/*Change made here to directly command TAS*/
	_TAS_setpoint  = _EAS_setpoint * 1; //EAS2TAS;


	// If airspeed measurements are not being used, fix the airspeed estimate to the nominal cruise airspeed
	if (!PX4_ISFINITE(equivalent_airspeed) || !airspeed_sensor_enabled()) {
		_EAS = _equivalent_airspeed_cruise;

	} else {
		_EAS = equivalent_airspeed;
	}

	// If first time through or not flying, reset airspeed states
	if (_speed_update_timestamp == 0 || !_in_air) {
		_tas_rate_state = 0.0f;
		_tas_state = (_EAS * EAS2TAS);
	}

	// Obtain a smoothed TAS estimate using a second order complementary filter

	// Update TAS rate state
	_tas_innov = (_EAS * EAS2TAS) - _tas_state;
	float tas_rate_state_input = _tas_innov * _tas_estimate_freq * _tas_estimate_freq;

	// limit integrator input to prevent windup
	if (_tas_state < 3.1f) {
		tas_rate_state_input = max(tas_rate_state_input, 0.0f);
	}

	// Update TAS state
	_tas_rate_state = _tas_rate_state + tas_rate_state_input * dt;
	float tas_state_input = _tas_rate_state + _tas_rate_raw + _tas_innov * _tas_estimate_freq * 1.4142f;
	_tas_state = _tas_state + tas_state_input * dt;

	// Limit the TAS state to a minimum of 3 m/s
	_tas_state = max(_tas_state, 3.0f);
	_speed_update_timestamp = now;

}

void PI_X::_update_speed_setpoint()
{
	_TAS_rate_setpoint = (_TAS_setpoint - _tas_state) * _airspeed_error_gain;
}

void PI_X::updateHeightRateSetpoint(float alt_sp_amsl_m, float target_climbrate_m_s, float target_sinkrate_m_s,
				    float alt_amsl)
{
	_hgt_setpoint = alt_sp_amsl_m;
	// Use a first order system to calculate a height rate setpoint from the current height error.
	_hgt_actual = alt_amsl;



}

void PI_X::_update_height_rate_setpoint(float hgt_rate_sp)
{
	// Limit the rate of change of height demand to respect vehicle performance limits
	//_hgt_rate_setpoint = math::constrain(hgt_rate_sp, -_max_sink_rate, _max_climb_rate);
	_hgt_setpoint = _vert_pos_state;
}

void PI_X::_detect_underspeed()
{

}

//MAYBE REPORT ENERGY ESTIMATES TO COMPARE TO OTHER METHOD
void PI_X::_update_energy_estimates()
{
	/*// Calculate specific energy rate demands in units of (m**2/sec**3)
	_SPE_rate_setpoint = _hgt_rate_setpoint / _tas_state; // potential energy rate of change
	_SKE_rate_setpoint = _TAS_rate_setpoint / CONSTANTS_ONE_G; // kinetic energy rate of change
	double double__hgt_rate_setpoint = double(_hgt_rate_setpoint);
	std::printf("tecsx double__hgt_rate_setpoint:\t %f\n", double__hgt_rate_setpoint);

	// Calculate specific energy rates in units of (m**2/sec**3)
	_SPE_rate = _vert_vel_state / _tas_state; // potential energy rate of change
	_SKE_rate = _tas_rate_filtered / CONSTANTS_ONE_G;// kinetic energy rate of change
	*/

}

void PI_X::_update_throttle_setpoint(const float throttle_cruise)
{	//instead off adding up throttle cruise here, set to zero
	float _throttle_setpoint = 0;

	if (airspeed_sensor_enabled()) {

		_throttle_setpoint += pid_calculate_upper_lower(&_speed_controller,  _TAS_setpoint, _tas_state, _dt);

		_last_throttle_setpoint = _throttle_setpoint;
	}


}

void PI_X::_update_pitch_setpoint()
{
	float _pitch_setpoint = radians(pid_calculate_upper_lower(&_altitude_controller, _hgt_setpoint, _hgt_actual, _dt));

	_last_pitch_setpoint = _pitch_setpoint;

	//double double_last_pitch_setpoint = double(_last_pitch_setpoint);
	//	std::printf("pi_x_double_last_pitch_setpoint:\t %f\n", double_last_pitch_setpoint);
}

void PI_X::_initialize_states(float pitch, float throttle_cruise, float baro_altitude, float pitch_min_climbout,
			      float EAS2TAS)
{
	if (_pitch_update_timestamp == 0 || _dt > DT_MAX || !_in_air || !_states_initialized) {

		_vert_vel_state = 0.0f;
		_vert_pos_state = baro_altitude;
		_tas_rate_state = 0.0f;
		_tas_state = _EAS * EAS2TAS;
		_throttle_integ_state =  0.0f;
		_pitch_integ_state = 0.0f;
		_last_throttle_setpoint = (_in_air ? throttle_cruise : 0.0f);
		//double doublethrottle_cruise = double(throttle_cruise);
		//std::printf("last throttle setpoint:\t %f\n", doublethrottle_cruise);
		_last_pitch_setpoint = constrain(pitch, _pitch_setpoint_min, _pitch_setpoint_max);
		_pitch_setpoint_unc = _last_pitch_setpoint;
		_hgt_setpoint = baro_altitude;
		_TAS_setpoint_last = _EAS * EAS2TAS;
		_TAS_setpoint_adj = _TAS_setpoint_last;
		_underspeed_detected = false;
		_uncommanded_descent_recovery = false;


		pid_init(&_speed_controller, PID_MODE_DERIVATIV_CALC, _dt);
		pid_set_parameters_upper_lower(&_speed_controller,
				   _airspeed_error_gain_pi_x,
				   _integrator_gain_throttle_pi_x,
				   0.0f, //d gain
				   10000.0f,  //integral limit -> not used
				   0.0f, //output limit low
				   1.0f); //output limit high
		//double double__airspeed_error_gain_pi_x = double(_airspeed_error_gain_pi_x);
		//std::printf("pi_x double__airspeed_error_gain_pi_x:\t %f\n", double__airspeed_error_gain_pi_x);

		pid_init(&_altitude_controller, PID_MODE_DERIVATIV_CALC, _dt);
		//use the overloaded set method which has lower and upper output limit
		pid_set_parameters_upper_lower(&_altitude_controller,
				   _height_error_gain_pi_x,
				   _integrator_gain_pitch_pi_x,
				   0.0f, //d gain
				   10000.0f,  //integral limit -> not used
				   _pitch_setpoint_min, //output limit low
				   _pitch_setpoint_max); //output limit high



		if (_dt > DT_MAX || _dt < DT_MIN) {
			_dt = DT_DEFAULT;
		}


	}


	_states_initialized = true;
}

void PI_X::_update_STE_rate_lim()
{

}

void PI_X::update_pitch_throttle(float pitch, float baro_altitude, float hgt_setpoint,
				 float EAS_setpoint, float equivalent_airspeed, float eas_to_tas, bool climb_out_setpoint, float pitch_min_climbout,
				 float throttle_min, float throttle_max, float throttle_cruise, float pitch_limit_min, float pitch_limit_max,
				 float target_climbrate, float target_sinkrate, float hgt_rate_sp)
{
	// Calculate the time since last update (seconds)
	uint64_t now = hrt_absolute_time();
	_dt = fmaxf((now - _pitch_update_timestamp) * 1e-6f, DT_MIN);

	// Set class variables from inputs
	_throttle_setpoint_max = throttle_max;
	_throttle_setpoint_min = throttle_min;
	_pitch_setpoint_max = pitch_limit_max;
	_pitch_setpoint_min = pitch_limit_min;
	_climbout_mode_active = climb_out_setpoint;

	// Initialize selected states and variables as required
	_initialize_states(pitch, throttle_cruise, baro_altitude, pitch_min_climbout, eas_to_tas);

	// Don't run TECS control algorithms when not in flight
	if (!_in_air) {
		return;
	}

	// Update the true airspeed state estimate
	_update_speed_states(EAS_setpoint, equivalent_airspeed, eas_to_tas);

	// Calculate rate limits for specific total energy
	_update_STE_rate_lim();

	// Detect an underspeed condition
	_detect_underspeed();

	_update_speed_height_weights();


	// Calculate the demanded true airspeed
	_update_speed_setpoint();

	if (PX4_ISFINITE(hgt_rate_sp)) {
		// use the provided height rate setpoint instead of the height setpoint
		_update_height_rate_setpoint(hgt_rate_sp);

	} else {
		// calculate heigh rate setpoint based on altitude demand
		updateHeightRateSetpoint(hgt_setpoint, target_climbrate, target_sinkrate, baro_altitude);
	}

	// Calculate the specific energy values required by the control loop
	_update_energy_estimates();

	// Calculate the throttle demand
	_update_throttle_setpoint(throttle_cruise);

	// Calculate the pitch demand
	_update_pitch_setpoint();



	// Update time stamps
	_pitch_update_timestamp = now;



}

void PI_X::_update_speed_height_weights()
{

}

float PI_X::get_integrator_state_speed()
{
	return _speed_controller.integral;
}

float PI_X::get_integrator_state_altitude()
{
	return _altitude_controller.integral;
}

