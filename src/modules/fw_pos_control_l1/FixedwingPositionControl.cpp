/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "FixedwingPositionControl.hpp"

#include <vtol_att_control/vtol_type.h>

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

FixedwingPositionControl::FixedwingPositionControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_launchDetector(this),
	_runway_takeoff(this)
{
	if (vtol) {
		_param_handle_airspeed_trans = param_find("VT_ARSP_TRANS");

		// VTOL parameter VTOL_TYPE
		int32_t vt_type = -1;
		param_get(param_find("VT_TYPE"), &vt_type);

		_vtol_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
	}

	// limit to 50 Hz
	_local_pos_sub.set_interval_ms(20);

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle local position callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingPositionControl::parameters_update()
{
	updateParams();

	// VTOL parameter VT_ARSP_TRANS
	if (_param_handle_airspeed_trans != PARAM_INVALID) {
		param_get(_param_handle_airspeed_trans, &_param_airspeed_trans);
	}

	// L1 control parameters
	_l1_control.set_l1_damping(_param_fw_l1_damping.get());
	_l1_control.set_l1_period(_param_fw_l1_period.get());
	_l1_control.set_l1_roll_limit(radians(_param_fw_r_lim.get()));
	_l1_control.set_roll_slew_rate(radians(_param_fw_l1_r_slew_max.get()));

	// TECS parameters
	_tecs.set_max_climb_rate(_param_fw_t_clmb_max.get());
	_tecs.set_max_sink_rate(_param_fw_t_sink_max.get());
	_tecs.set_speed_weight(_param_fw_t_spdweight.get());
	_tecs.set_equivalent_airspeed_cruise(_param_fw_airspd_trim.get());
	_tecs.set_equivalent_airspeed_min(_param_fw_airspd_min.get());
	_tecs.set_equivalent_airspeed_max(_param_fw_airspd_max.get());
	_tecs.set_min_sink_rate(_param_fw_t_sink_min.get());
	_tecs.set_throttle_damp(_param_fw_t_thr_damp.get());
	_tecs.set_integrator_gain_throttle(_param_fw_t_I_gain_thr.get());
	_tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
	_tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
	_tecs.set_vertical_accel_limit(_param_fw_t_vert_acc.get());
	_tecs.set_speed_comp_filter_omega(_param_fw_t_spd_omega.get());
	_tecs.set_roll_throttle_compensation(_param_fw_t_rll2thr.get());
	_tecs.set_pitch_damping(_param_fw_t_ptch_damp.get());
	_tecs.set_height_error_time_constant(_param_fw_t_h_error_tc.get());
	_tecs.set_heightrate_ff(_param_fw_t_hrate_ff.get());
	_tecs.set_airspeed_error_time_constant(_param_fw_t_tas_error_tc.get());
	_tecs.set_ste_rate_time_const(_param_ste_rate_time_const.get());
	_tecs.set_speed_derivative_time_constant(_param_tas_rate_time_const.get());
	_tecs.set_seb_rate_ff_gain(_param_seb_rate_ff.get());

	// TECS X parameters

	_tecs_X.set_max_climb_rate(_param_fw_tx_clmb_max.get());
	_tecs_X.set_max_sink_rate(_param_fw_tx_sink_max.get());
	_tecs_X.set_speed_weight(_param_fw_tx_spdweight.get());
	_tecs_X.set_min_sink_rate(_param_fw_tx_sink_min.get());
	_tecs_X.set_throttle_damp(_param_fw_tx_thr_damp.get());
	_tecs_X.set_integrator_gain_throttle(_param_fw_tx_I_gain_thr.get());
	_tecs_X.set_integrator_gain_pitch(_param_fw_tx_I_gain_pit.get());
	_tecs_X.set_vertical_accel_limit(_param_fw_tx_vert_acc.get());
	_tecs_X.set_speed_comp_filter_omega(_param_fw_tx_spd_omega.get());
	_tecs_X.set_roll_throttle_compensation(_param_fw_tx_rll2thr.get());
	_tecs_X.set_pitch_damping(_param_fw_tx_ptch_damp.get());
	_tecs_X.set_height_error_time_constant(_param_fw_tx_h_error_tc.get());
	_tecs_X.set_heightrate_ff(_param_fw_tx_hrate_ff.get());
	_tecs_X.set_airspeed_error_time_constant(_param_fw_tx_tas_error_tc.get());
	_tecs_X.set_ste_rate_time_const(_param_ste_x_rate_time_const.get());
	_tecs_X.set_seb_rate_ff_gain(_param_seb_x_rate_ff.get());
	_tecs_X.set_mass(_param_mass.get());
	_tecs_X.set_max_power(_param_max_power.get());


	// Based on basic TECS
	_tecs_X.set_equivalent_airspeed_cruise(_param_fw_airspd_trim.get());
	_tecs_X.set_equivalent_airspeed_min(_param_fw_airspd_min.get());
	_tecs_X.set_equivalent_airspeed_max(_param_fw_airspd_max.get());
	_tecs_X.set_throttle_slewrate(_param_fw_thr_slew_max.get());
	_tecs_X.set_speed_derivative_time_constant(_param_tas_rate_time_const.get());

	// PI X parameters

	_pi_X.set_max_climb_rate(_param_fw_tx_clmb_max.get());
	_pi_X.set_max_sink_rate(_param_fw_tx_sink_max.get());
	_pi_X.set_min_sink_rate(_param_fw_tx_sink_min.get());
	_pi_X.set_vertical_accel_limit(_param_fw_tx_vert_acc.get());
	_pi_X.set_speed_comp_filter_omega(_param_fw_tx_spd_omega.get());
	_pi_X.set_roll_throttle_compensation(_param_fw_tx_rll2thr.get());
	_pi_X.set_height_error_time_constant(_param_fw_tx_h_error_tc.get());
	_pi_X.set_error_gain_throttle(_param_fw_pi_x_tas_gain.get());
	_pi_X.set_integrator_gain_throttle(_param_fw_pi_x_tas_I_gain.get());
	_pi_X.set_error_gain_pitch(_param_fw_pi_x_h_gain.get());
	_pi_X.set_integrator_gain_pitch(_param_fw_pi_x_h_I_gain.get());




	// Based on basic TECS
	_pi_X.set_equivalent_airspeed_cruise(_param_fw_airspd_trim.get());
	_pi_X.set_equivalent_airspeed_min(_param_fw_airspd_min.get());
	_pi_X.set_equivalent_airspeed_max(_param_fw_airspd_max.get());
	_pi_X.set_throttle_slewrate(_param_fw_thr_slew_max.get());


	// Maneuver
	_maneuver.set_base_spd_sp(_param_fw_x_spd_target.get());
	_maneuver.set_base_hgt_sp(_param_fw_x_hgt_target.get());
	_maneuver.set_rel_spd_sp(_param_fw_x_rel_spd_target.get());
	_maneuver.set_rel_hgt_sp(_param_fw_x_rel_hgt_target.get());
	_maneuver.set_spd_rise_time(_param_fw_x_risetime_spd.get());
	_maneuver.set_hgt_rise_time(_param_fw_x_risetime_hgt.get());
	_maneuver.set_init_time(_param_fw_x_init_time.get());

	// Mode and Ctrl selection parameters
	_mode_sel = _param_fw_x_mode.get();
	_ctrl_sel = _param_fw_x_ctrl_sel.get();


	// Landing slope
	/* check if negative value for 2/3 of flare altitude is set for throttle cut */
	float land_thrust_lim_alt_relative = _param_fw_lnd_tlalt.get();

	if (land_thrust_lim_alt_relative < 0.0f) {
		land_thrust_lim_alt_relative = 0.66f * _param_fw_lnd_flalt.get();
	}

	_landingslope.update(radians(_param_fw_lnd_ang.get()), _param_fw_lnd_flalt.get(), land_thrust_lim_alt_relative,
			     _param_fw_lnd_hvirt.get());

	landing_status_publish();

	int check_ret = PX4_OK;

	// sanity check parameters
	if (_param_fw_airspd_max.get() < _param_fw_airspd_min.get()) {
		mavlink_log_critical(&_mavlink_log_pub, "Config invalid: Airspeed max smaller than min");
		check_ret = PX4_ERROR;
	}

	if (_param_fw_airspd_max.get() < 5.0f || _param_fw_airspd_min.get() > 100.0f) {
		mavlink_log_critical(&_mavlink_log_pub, "Config invalid: Airspeed max < 5 m/s or min > 100 m/s");
		check_ret = PX4_ERROR;
	}

	if (_param_fw_airspd_trim.get() < _param_fw_airspd_min.get() ||
	    _param_fw_airspd_trim.get() > _param_fw_airspd_max.get()) {
		mavlink_log_critical(&_mavlink_log_pub, "Config invalid: Airspeed cruise out of min or max bounds");
		check_ret = PX4_ERROR;
	}

	if (_param_fw_airspd_stall.get() > _param_fw_airspd_min.get() * 0.9f) {
		mavlink_log_critical(&_mavlink_log_pub, "Config invalid: Stall airspeed higher than 0.9 of min");
		check_ret = PX4_ERROR;
	}

	return check_ret;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		const bool was_armed = _control_mode.flag_armed;

		if (_control_mode_sub.copy(&_control_mode)) {

			// reset state when arming
			if (!was_armed && _control_mode.flag_armed) {
				reset_takeoff_state(true);
				reset_landing_state();
			}
		}
	}
}

void
FixedwingPositionControl::vehicle_command_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
			// only abort landing before point of no return (horizontal and vertical)
			if (_control_mode.flag_control_auto_enabled &&
			    _pos_sp_triplet.current.valid &&
			    (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)) {

				abort_landing(true);
			}
		}

	}
}

void
FixedwingPositionControl::airspeed_poll()
{
	bool airspeed_valid = _airspeed_valid;
	airspeed_validated_s airspeed_validated;

	if ((_param_fw_arsp_mode.get() == 0) && _airspeed_validated_sub.update(&airspeed_validated)) {

		_eas2tas = 1.0f; //this is the default value, taken in case of invalid airspeed
		_current_true_airspeed = airspeed_validated.true_airspeed_m_s;

		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(_current_true_airspeed)
		    && (airspeed_validated.calibrated_airspeed_m_s > 0.0f)) {

			airspeed_valid = true;

			_airspeed_last_valid = airspeed_validated.timestamp;
			_airspeed = airspeed_validated.calibrated_airspeed_m_s;

			_eas2tas = constrain(airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s, 0.9f, 2.0f);
		}

	} else {
		// no airspeed updates for one second
		if (airspeed_valid && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s)) {
			airspeed_valid = false;
		}
	}

	// update TECS if validity changed
	if (airspeed_valid != _airspeed_valid) {
		_tecs.enable_airspeed(airspeed_valid);
		_tecs_X.enable_airspeed(airspeed_valid);
		_pi_X.enable_airspeed(airspeed_valid);
		_airspeed_valid = airspeed_valid;
	}
	//spark hil tests
	airspeed_valid = true;
	_tecs.enable_airspeed(airspeed_valid);
	_tecs_X.enable_airspeed(airspeed_valid);
	_pi_X.enable_airspeed(airspeed_valid);
	_airspeed_valid = airspeed_valid;
	//_airspeed = hil_state.true_airspeed * 1e-2f;; //_current_groundspeed;
}

void
FixedwingPositionControl::manual_control_setpoint_poll()
{
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

	_manual_control_setpoint_altitude = _manual_control_setpoint.x;
	_manual_control_setpoint_airspeed = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);

	if (_param_fw_posctl_inv_st.get()) {
		/* Alternate stick allocation (similar concept as for multirotor systems:
		 * demanding up/down with the throttle stick, and move faster/break with the pitch one.
		 */
		_manual_control_setpoint_altitude = -(math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) * 2.f - 1.f);
		_manual_control_setpoint_airspeed = math::constrain(_manual_control_setpoint.x, -1.0f, 1.0f) / 2.f + 0.5f;
	}
}


void
FixedwingPositionControl::vehicle_attitude_poll()
{
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		const Vector3f rates{angular_velocity.xyz};

		Dcmf R{Quatf(att.q)};

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		if (_vtol_tailsitter) {
			const Dcmf R_offset{Eulerf{0.f, M_PI_2_F, 0.f}};
			R = R * R_offset;

			_yawrate = rates(0);

		} else {
			_yawrate = rates(2);
		}

		const Eulerf euler_angles(R);
		_roll = euler_angles(0);
		_pitch = euler_angles(1);
		_yaw = euler_angles(2);

		_body_acceleration = R.transpose() * Vector3f{_local_pos.ax, _local_pos.ay, _local_pos.az};
		_body_velocity = R.transpose() * Vector3f{_local_pos.vx, _local_pos.vy, _local_pos.vz};

		// update TECS load factor
		const float load_factor = 1.f / cosf(euler_angles(0));
		_tecs.set_load_factor(load_factor);
		_tecs_X.set_load_factor(load_factor);
	}
}

float
FixedwingPositionControl::get_demanded_airspeed()
{
	float altctrl_airspeed = 0;

	// neutral throttle corresponds to trim airspeed
	if (_manual_control_setpoint_airspeed < 0.5f) {
		// lower half of throttle is min to trim airspeed
		altctrl_airspeed = _param_fw_airspd_min.get() +
				   (_param_fw_airspd_trim.get() - _param_fw_airspd_min.get()) *
				   _manual_control_setpoint_airspeed * 2;

	} else {
		// upper half of throttle is trim to max airspeed
		altctrl_airspeed = _param_fw_airspd_trim.get() +
				   (_param_fw_airspd_max.get() - _param_fw_airspd_trim.get()) *
				   (_manual_control_setpoint_airspeed * 2 - 1);
	}

	// OVERWRITE
	altctrl_airspeed = _param_fw_x_spd_target.get();

	if (_man_active == true) {
		altctrl_airspeed += _maneuver.get_test_spd_sp();


	}

	_altctrl_airspeed = altctrl_airspeed;
	return altctrl_airspeed;

}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand, const Vector2f &ground_speed)
{
	/*
	 * Calculate accelerated stall airspeed factor from commanded bank angle and use it to increase minimum airspeed.
	 *
	 *  We don't know the stall speed of the aircraft, but assuming user defined
	 *  minimum airspeed (FW_AIRSPD_MIN) is slightly larger than stall speed
	 *  this is close enough.
	 *
	 * increase lift vector to balance additional weight in bank
	 *  cos(bank angle) = W/L = 1/n
	 *   n is the load factor
	 *
	 * lift is proportional to airspeed^2 so the increase in stall speed is
	 *  Vsacc = Vs * sqrt(n)
	 *
	 */
	float adjusted_min_airspeed = _param_fw_airspd_min.get();

	if (_airspeed_valid && PX4_ISFINITE(_att_sp.roll_body)) {

		adjusted_min_airspeed = constrain(_param_fw_airspd_min.get() / sqrtf(cosf(_att_sp.roll_body)),
						  _param_fw_airspd_min.get(), _param_fw_airspd_max.get());
	}

	// groundspeed undershoot
	if (!_l1_control.circle_mode()) {
		/*
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		const float ground_speed_body = _body_velocity(0);

		if (ground_speed_body < _param_fw_gnd_spd_min.get()) {
			airspeed_demand += max(_param_fw_gnd_spd_min.get() - ground_speed_body, 0.0f);
		}
	}

	// add minimum ground speed undershoot (only non-zero in presence of sufficient wind)
	// sanity check: limit to range
	return constrain(airspeed_demand, adjusted_min_airspeed, _param_fw_airspd_max.get());
}

void
FixedwingPositionControl::tecs_status_publish()
{
	tecs_status_s t{};
	testflight_tecs_status_x_s tx{};
	//testflight_tecs_status_x_s px{};
	testflight_status_x_s tfx{};
	testflight_control_params_s tfc{};
	testflight_std_tecs_params_s tfc_std{};
	testflight_pi_status_x_s pix{};

	switch (_tecs.tecs_mode()) {
	case TECS::ECL_TECS_MODE_NORMAL:
		t.mode = tecs_status_s::TECS_MODE_NORMAL;
		break;

	case TECS::ECL_TECS_MODE_UNDERSPEED:
		t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
		break;

	case TECS::ECL_TECS_MODE_BAD_DESCENT:
		t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
		break;

	case TECS::ECL_TECS_MODE_CLIMBOUT:
		t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
		break;
	}

	t.altitude_sp = _tecs.hgt_setpoint();
	t.altitude_filtered = _tecs.vert_pos_state();

	t.true_airspeed_sp = _tecs.TAS_setpoint_adj();
	t.true_airspeed_filtered = _tecs.tas_state();

	t.height_rate_setpoint = _tecs.hgt_rate_setpoint();
	t.height_rate = _tecs.vert_vel_state();

	t.equivalent_airspeed_sp = _tecs.get_EAS_setpoint();
	t.true_airspeed_derivative_sp = _tecs.TAS_rate_setpoint();
	t.true_airspeed_derivative = _tecs.speed_derivative();
	t.true_airspeed_derivative_raw = _tecs.speed_derivative_raw();
	t.true_airspeed_innovation = _tecs.getTASInnovation();

	t.total_energy_error = _tecs.STE_error();
	t.total_energy_rate_error = _tecs.STE_rate_error();

	t.energy_distribution_error = _tecs.SEB_error();
	t.energy_distribution_rate_error = _tecs.SEB_rate_error();

	t.total_energy = _tecs.STE();
	t.total_energy_rate = _tecs.STE_rate();
	t.total_energy_balance = _tecs.SEB();
	t.total_energy_balance_rate = _tecs.SEB_rate();

	t.total_energy_sp = _tecs.STE_setpoint();
	t.total_energy_rate_sp = _tecs.STE_rate_setpoint();
	t.total_energy_balance_sp = _tecs.SEB_setpoint();
	t.total_energy_balance_rate_sp = _tecs.SEB_rate_setpoint();

	t.throttle_integ = _tecs.throttle_integ_state();
	t.pitch_integ = _tecs.pitch_integ_state();

	t.throttle_sp = _tecs.get_throttle_setpoint();
	t.pitch_sp_rad = _tecs.get_pitch_setpoint();

	t.timestamp = hrt_absolute_time();
	_tecs_status_pub.publish(t);


	tx.mode_x = testflight_tecs_status_x_s::TECS_MODE_NORMAL_X;


	tx.altitude_sp_x = _tecs_X.hgt_setpoint();
	tx.altitude_filtered_x = _tecs_X.vert_pos_state();

	tx.true_airspeed_sp_x = _tecs_X.TAS_setpoint_adj();
	tx.true_airspeed_filtered_x = _tecs_X.tas_state();

	tx.height_rate_setpoint_x = _tecs_X.hgt_rate_setpoint();
	tx.height_rate_x = _tecs_X.vert_vel_state();

	tx.equivalent_airspeed_sp_x = _tecs_X.get_EAS_setpoint();
	tx.true_airspeed_derivative_sp_x = _tecs_X.TAS_rate_setpoint();
	tx.true_airspeed_derivative_x = _tecs_X.speed_derivative();
	tx.true_airspeed_derivative_raw_x = _tecs_X.speed_derivative_raw();
	tx.true_airspeed_innovation_x = _tecs_X.getTASInnovation();

	tx.total_energy_error_x = _tecs_X.STE_error();
	tx.total_energy_rate_error_x = _tecs_X.STE_rate_error();

	tx.energy_distribution_error_x = _tecs_X.SEB_error();
	tx.energy_distribution_rate_error_x = _tecs_X.SEB_rate_error();

	tx.total_energy_x = _tecs_X.STE();
	tx.total_energy_rate_x = _tecs_X.STE_rate();
	tx.total_energy_balance_x = _tecs_X.SEB();
	tx.total_energy_balance_rate_x = _tecs_X.SEB_rate();

	tx.total_energy_sp_x = _tecs_X.STE_setpoint();
	tx.total_energy_rate_sp_x = _tecs_X.STE_rate_setpoint();
	tx.total_energy_balance_sp_x = _tecs_X.SEB_setpoint();
	tx.total_energy_balance_rate_sp_x = _tecs_X.SEB_rate_setpoint();

	tx.throttle_integ_x = _tecs_X.throttle_integ_state();
	tx.pitch_integ_x = _tecs_X.pitch_integ_state();

	tx.throttle_sp_x = _tecs_X.get_throttle_setpoint();
	tx.pitch_sp_rad_x = _tecs_X.get_pitch_setpoint();

	tx.timestamp = hrt_absolute_time();



	//testflight topic

	tfx.timestamp = tx.timestamp;
	tfc.timestamp = tx.timestamp;
	tfc_std.timestamp = tx.timestamp;
	pix.timestamp = tx.timestamp;
	//vehicle_global_position_s vehicle_global_position = {};
	//_vehicle_global_position_sub gpos;
	//_global_pos_sub.copy(&gpos);
	//vehicle_global_position_s gpos;

	/*if (_global_pos_sub.update(&gpos)) {
		_current_latitude = gpos.lat;
		_current_longitude = gpos.lon;
		_current_groundlevel = gpos.terrain_alt;
	}
	*/
	tfx.terrain_alt = _local_pos.ref_alt;
	//grab maneuver parameters
	tfc.man_active = _man_active;

	if (_man_active) {
		_man_delta_alt_cmd = _maneuver.get_test_hgt_sp();
		_man_delta_spd_cmd = _maneuver.get_test_spd_sp();
	}

	tfc.man_ctrl_sel = _ctrl_sel;
	tfc.man_delta_alt_cmd = _man_delta_alt_cmd;
	tfc.man_delta_spd_cmd = _man_delta_spd_cmd;
	tfc.man_risetime_spd = _param_fw_x_risetime_spd.get();
	tfc.man_risetime_hgt = _param_fw_x_risetime_hgt.get();
	tfc.man_position_sp_alt = _position_sp_alt;
	tfc.man_position_sp_airspeed = _altctrl_airspeed;

	//grab pitch controller parameters
	tfc.pitch_cntr_param_fw_p_rmax_neg = _param_fw_p_rmax_neg.get();
	tfc.pitch_cntr_param_fw_p_rmax_pos =  _param_fw_p_rmax_pos.get();
	tfc.pitch_cntr_param_fw_p_tc =  _param_fw_p_tc.get();
	tfc.pitch_cntr_param_fw_pr_ff = _param_fw_pr_ff.get();
	tfc.pitch_cntr_param_fw_pr_i = _param_fw_pr_i.get();
	tfc.pitch_cntr_param_fw_pr_imax =  _param_fw_pr_imax.get();
	tfc.pitch_cntr_param_fw_pr_p = _param_fw_pr_p.get();
	tfc.pitch_cntr_param_fw_psp_off = _param_fw_psp_off.get();

	//grab standard tecs parameters
	//tfc.std_tecs_param_fw_airspd_max = _param_fw_airspd_max.get();
	//tfc.std_tecs_param_fw_airspd_min =_param_fw_airspd_min.get();
	tfc_std.std_tecs_fw_airspd_trim = _param_fw_airspd_trim.get();
	//tfc.std_tecs_param_fw_airspd_stall =_param_fw_airspd_stall.get();
	tfc_std.std_tecs_aram_fw_clmbout_diff = _param_fw_clmbout_diff.get();
	//tfc.std_tecs_param_fw_gnd_spd_min =_param_fw_gnd_spd_min.get();
	//tfc.std_tecs_param_fw_l1_damping =_param_fw_l1_damping.get();
	//fc.std_tecs_param_fw_l1_period =_param_fw_l1_period.get();
	//tfc.std_tecs_param_fw_l1_r_slew_max =_param_fw_l1_r_slew_max.get();
	//tfc.std_tecs_param_fw_r_lim = _param_fw_r_lim.get();
	//tfc.std_tecs_param_fw_lnd_airspd_sc =_param_fw_lnd_airspd_sc.get();
	//tfc.std_tecs_param_fw_lnd_ang =_param_fw_lnd_ang.get();
	//tfc.std_tecs_param_fw_lnd_fl_pmax =_param_fw_lnd_fl_pmax.get();
	//tfc.std_tecs_param_fw_lnd_fl_pmin = _param_fw_lnd_fl_pmin.get();
	//tfc.std_tecs_param_fw_lnd_flalt = _param_fw_lnd_flalt.get();
	//tfc.std_tecs_param_fw_lnd_hhdist = _param_fw_lnd_hhdist.get();
	//tfc.std_tecs_param_fw_lnd_hvirt =_param_fw_lnd_hvirt.get();
	//tfc.std_tecs_param_fw_thrtc_sc =_param_fw_thrtc_sc.get();
	//tfc.std_tecs_param_fw_lnd_tlalt =_param_fw_lnd_tlalt.get();
	//tfc.std_tecs_param_fw_lnd_earlycfg =_param_fw_lnd_earlycfg.get();
	//tfc.std_tecs_param_fw_lnd_useter =_param_fw_lnd_useter.get();
	tfc_std.std_tecs_param_fw_p_lim_max = _param_fw_p_lim_max.get();
	tfc_std.std_param_fw_p_lim_min = _param_fw_p_lim_min.get();
	tfc_std.std_tecs_param_fw_t_clmb_max = _param_fw_t_clmb_max.get();
	tfc_std.std_tecs_param_fw_t_hrate_ff = _param_fw_t_hrate_ff.get();
	tfc_std.std_tecs_param_fw_t_h_error_tc = _param_fw_t_h_error_tc.get();
	tfc_std.std_tecs_param_fw_t_i_gain_thr = _param_fw_t_I_gain_thr.get();
	tfc_std.std_tecs_param_fw_t_i_gain_pit = _param_fw_t_I_gain_pit.get();
	tfc_std.std_tecs_param_fw_t_ptch_damp = _param_fw_t_ptch_damp.get();
	//tfc.std_tecs_param_fw_t_rll2thr = _param_fw_t_rll2thr.get();
	tfc_std.std_tecs_param_fw_t_sink_max = _param_fw_t_sink_max.get();
	tfc_std.std_tecs_param_fw_t_sink_min = _param_fw_t_sink_min.get();
	tfc_std.std_tecs_param_fw_t_spd_omega = _param_fw_t_spd_omega.get();
	tfc_std.std_tecs_param_fw_t_spdweight = _param_fw_t_spdweight.get();
	tfc_std.std_tecs_param_fw_t_tas_error_tc = _param_fw_t_tas_error_tc.get();
	tfc_std.std_tecs_param_fw_t_thr_damp = _param_fw_t_thr_damp.get();
	tfc_std.std_tecs_param_fw_t_vert_acc = _param_fw_t_vert_acc.get();
	tfc_std.std_tecs_param_ste_rate_time_const = _param_ste_rate_time_const.get();
	tfc_std.std_tecs_param_tas_rate_time_const = _param_tas_rate_time_const.get();
	tfc_std.std_tecs_param_seb_rate_ff = _param_seb_rate_ff.get();
	tfc_std.std_tecs_param_climbrate_target = _param_climbrate_target.get();
	tfc_std.std_tecs_param_sinkrate_target = _param_sinkrate_target.get();
	tfc_std.std_tecs_param_fw_thr_alt_scl = _param_fw_thr_alt_scl.get();
	tfc_std.std_tecs_param_param_fw_thr_cruise = _param_fw_thr_cruise.get();
	tfc_std.std_tecs_param_fw_thr_idle = _param_fw_thr_idle.get();
	tfc_std.std_tecs_param_fw_thr_lnd_max = _param_fw_thr_lnd_max.get();
	tfc_std.std_tecs_param_fw_thr_max = _param_fw_thr_max.get();
	tfc_std.std_tecs_param_fw_thr_min = _param_fw_thr_min.get();
	tfc_std.std_tecs_param_fw_thr_slew_max = _param_fw_thr_slew_max.get();
	tfc_std.std_tecs_param_fw_posctl_inv_st = _param_fw_posctl_inv_st.get();
	tfc_std.std_tecs_param_fw_arsp_mode = _param_fw_arsp_mode.get();
	tfc_std.std_tecs_param_fw_psp_off = _param_fw_psp_off.get();
	tfc_std.std_tecs_param_fw_man_p_max = _param_fw_man_p_max.get();
	//tfc.std_tecs_param_fw_man_r_max = _param_fw_man_r_max.get();
	//tfc.std_tecs_param_nav_loiter_rad = _param_nav_loiter_rad.get();
	//tfc.std_tecs_param_takeoff_pitch_min = _takeoff_pitch_min.get();
	//grab experimental tecs parameters
	//tfc.experimental_tecs_param_fw_x_hgt_target = _param_fw_x_hgt_target.get();
	//tfc.experimental_tecs_param_fw_x_spd_target = _param_fw_x_spd_target.get();
	//tfc.experimental_tecs_param_fw_x_rel_hgt_target = _param_fw_x_rel_hgt_target.get();
	//tfc.experimental_tecs_param_fw_x_rel_spd_target = _param_fw_x_rel_spd_target.get();
	//tfc.experimental_tecs_param_fw_x_risetime_hgt = _param_fw_x_risetime_hgt.get();
	//tfc.experimental_tecs_param_fw_x_risetime_spd = _param_fw_x_risetime_spd.get();
	//tfc.experimental_tecs_param_fw_x_mode = _param_fw_x_mode.get();
	tfc.experimental_tecs_param_fw_x_ctrl_sel = _param_fw_x_ctrl_sel.get();
	tfc.experimental_tecs_param_fw_x_init_time = _param_fw_x_init_time.get();
	tfc.experimental_tecs_param_fw_tx_clmb_max = _param_fw_tx_clmb_max.get();
	tfc.experimental_tecs_param_fw_tx_hrate_ff = _param_fw_tx_hrate_ff.get();
	tfc.experimental_tecs_param_fw_tx_h_error_tc = _param_fw_tx_h_error_tc.get();
	tfc.experimental_tecs_param_fw_tx_i_gain_thr = _param_fw_tx_I_gain_thr.get();
	tfc.experimental_tecs_param_fw_tx_i_gain_pit = _param_fw_tx_I_gain_pit.get();
	tfc.experimental_tecs_param_fw_tx_ptch_damp = _param_fw_tx_ptch_damp.get();
	//tfc.experimental_tecs_param_fw_tx_rll2thr = _param_fw_tx_rll2thr.get();
	//tfc.experimental_tecs_param_fw_tx_sink_max = _param_fw_tx_sink_max.get();
	//tfc.experimental_tecs_param_fw_tx_sink_min = _param_fw_tx_sink_min.get();
	//tfc.experimental_tecs_param_fw_tx_sink_min = _param_fw_tx_spd_omega.get();
	tfc.experimental_tecs_param_fw_tx_spdweight = _param_fw_tx_spdweight.get();
	tfc.experimental_tecs_param_fw_tx_tas_error_tc = _param_fw_tx_tas_error_tc.get();
	//tfc.experimental_tecs_param_fw_tx_thr_damp = _param_fw_tx_thr_damp.get();
	//tfc.experimental_tecs_param_fw_tx_vert_acc = _param_fw_tx_vert_acc.get();
	tfc.experimental_tecs_param_ste_x_rate_time_const = _param_ste_x_rate_time_const.get();
	tfc.experimental_tecs_param_tas_x_rate_time_const = _param_tas_x_rate_time_const.get();
	tfc.experimental_tecs_param_seb_x_rate_ff = _param_seb_x_rate_ff.get();
	tfc.experimental_tecs_param_climbrate_x_target = _param_climbrate_x_target.get();
	tfc.experimental_tecs_param_sinkrate_x_target = _param_sinkrate_x_target.get();
	tfc.experimental_tecs_param_fw_x_stall = _param_fw_x_stall.get();

	//grab the experimental pi parameters
	tfc.experimental_pi_param_fw_pi_x_tas_gain = _param_fw_pi_x_tas_gain.get();
	tfc.experimental_pi_param_fw_pi_x_tas_i_gain = _param_fw_pi_x_tas_I_gain.get();
	tfc.experimental_pi_param_fw_pi_x_h_gain = _param_fw_pi_x_h_gain.get();
	tfc.experimental_pi_param_fw_pi_x_h_i_gain = _param_fw_pi_x_h_I_gain.get();

	//grab the angular rates
	vehicle_angular_velocity_s vav;

	if (_vehicle_angular_velocity_sub.update(&vav)) {
		current_x_rate = vav.xyz[0];
		current_y_rate = vav.xyz[1];
		current_z_rate = vav.xyz[2];
	}

	tfx.current_x_rate = current_x_rate;
	tfx.current_y_rate = current_y_rate;
	tfx.current_z_rate = current_z_rate;

	//grab the attitude
	tfx.current_att_pitch = _pitch;
	tfx.current_att_roll = _roll;
	tfx.current_att_yaw = _yaw;

	//grab the speeds
	tfx.current_true_airspeed = current_true_airspeed;
	tfx.airspeed_valid = _airspeed_valid;
	tfx.current_true_airspeed = _current_true_airspeed;

	vehicle_gps_position_s v_gps_pos;

	if (_vehicle_gps_position_sub.update(&v_gps_pos)) {
		_current_groundspeed = v_gps_pos.vel_m_s;
		_current_gps_altitude = v_gps_pos.alt;
	}

	tfx.current_groundspeed = _current_groundspeed;
	tfx.current_calibrated_airspeed = _airspeed;

	//grab thrust
	tfx.current_thrust_body = _att_sp.thrust_body[0];

	//grab altitudes
	tfx.current_altitude_baro = _current_altitude;
	tfx.current_gps_altitude = _current_gps_altitude;

	//grab position
	tfx.current_latitude = _current_latitude;
	tfx.current_longitude = _current_longitude;

	//grab PWM from pilot imput
	pwm_input_s pwm_in;

	if (_pwm_input_sub.update(&pwm_in)) {
		_pwm_err = pwm_in.error_count;
		_pwm_period = pwm_in.period;
		_pwm_pulse_width = pwm_in.pulse_width;
		_pwm_timestamp = pwm_in.timestamp;
	}

	tfx.pwm_err = _pwm_err;
	tfx.pwm_period = _pwm_period;
	tfx.pwm_pulse_width = _pwm_pulse_width;
	tfx.pwm_timestamp = _pwm_timestamp;


	//grab PWM from pilot imput
	actuator_outputs_s act_out;

	if (_actuator_outputs_sub.update(&act_out)) {
		_act_out_output0 = act_out.output[0];
		_act_out_output1 = act_out.output[1];
		_act_out_output2 = act_out.output[2];
		_act_out_output3 = act_out.output[3];
		_act_out_output4 = act_out.output[4];
		_act_out_output5 = act_out.output[5];
		_act_out_output6 = act_out.output[6];
		_act_out_output7 = act_out.output[7];
		_act_out_output8 = act_out.output[8];
		_act_out_output9 = act_out.output[9];
		_act_out_output10 = act_out.output[10];
		_act_out_output11 = act_out.output[11];
		_act_out_output12 = act_out.output[12];
		_act_out_output13 = act_out.output[13];
		_act_out_output14 = act_out.output[14];
		_act_out_output15 = act_out.output[15];
		_act_out_timestamp = act_out.timestamp;
	}

	tfx.act_out_output0 = _act_out_output0;
	tfx.act_out_output1 = _act_out_output1;
	tfx.act_out_output2 = _act_out_output2;
	tfx.act_out_output3 = _act_out_output3;
	tfx.act_out_output4 = _act_out_output4;
	tfx.act_out_output5 = _act_out_output5;
	tfx.act_out_output6 = _act_out_output6;
	tfx.act_out_output7 = _act_out_output7;
	tfx.act_out_output8 = _act_out_output8;
	tfx.act_out_output9 = _act_out_output9;
	tfx.act_out_output10 = _act_out_output10;
	tfx.act_out_output11 = _act_out_output11;
	tfx.act_out_output12 = _act_out_output12;
	tfx.act_out_output13 = _act_out_output13;
	tfx.act_out_output14 = _act_out_output14;
	tfx.act_out_output15 = _act_out_output15;

	tfx.act_out_timestamp = _act_out_timestamp;

	actuator_controls_s act_cntr;

	if (_actuator_controls_sub.update(&act_cntr)) {
		_act_cntr_output0 = act_cntr.control[0];
		_act_cntr_output1 = act_cntr.control[1];
		_act_cntr_output2 = act_cntr.control[2];
		_act_cntr_output3 = act_cntr.control[3];
		_act_cntr_output4 = act_cntr.control[4];
		_act_cntr_output5 = act_cntr.control[5];
		_act_cntr_output6 = act_cntr.control[6];
		_act_cntr_output7 = act_cntr.control[7];

		_act_out_timestamp = act_out.timestamp;
	}

	tfx.act_cntr_output0 = _act_cntr_output0;
	tfx.act_cntr_output1 = _act_cntr_output1;
	tfx.act_cntr_output2 = _act_cntr_output2;
	tfx.act_cntr_output3 = _act_cntr_output3;
	tfx.act_cntr_output4 = _act_cntr_output4;
	tfx.act_cntr_output5 = _act_cntr_output5;
	tfx.act_cntr_output6 = _act_cntr_output6;
	tfx.act_cntr_output7 = _act_cntr_output7;


	tfx.act_out_timestamp = _act_out_timestamp;

	//grab battery status
	battery_status_s bat_status;

	if (_battery_status_sub.update(&bat_status)) {
		_bat_status_timestamp = bat_status.timestamp;
		_bat_status_current_a = bat_status.current_a;
		_bat_status_discharged_mah = bat_status.discharged_mah;
		_bat_status_voltage_v = bat_status.voltage_v;
	}

	tfx.bat_status_timestamp = _bat_status_timestamp;
	tfx.bat_status_current_a = _bat_status_current_a;
	tfx.bat_status_discharged_mah = _bat_status_discharged_mah;
	tfx.bat_status_voltage_v = _bat_status_voltage_v;

	//grab setpoints
	tfx.position_sp_alt = _position_sp_alt;
	tfx.position_sp_airspeed = _altctrl_airspeed;

	//grab total energy from standard tecs
	tfx.std_tecs_kin_energy = _tecs.get_kin_energy();
	tfx.std_tecs_pot_energy = _tecs.get_pot_energy();
	tfx.std_tecs_tot_energy = _tecs.get_tot_energy();

	//grab internal PIX states
	pix.integrator_speed = _pi_X.get_integrator_state_speed();
	pix.integrator_altitude = _pi_X.get_integrator_state_altitude();

	_testflight_status_x_pub.publish(tfx);
	_testflight_control_params_pub.publish(tfc);


	_testflight_std_tecs_params_pub.publish(tfc_std);
	_testflight_tecs_status_x_pub.publish(tx);
	_testflight_pi_status_x_pub.publish(pix);



}

void
FixedwingPositionControl::status_publish()
{
	position_controller_status_s pos_ctrl_status = {};

	pos_ctrl_status.nav_roll = _att_sp.roll_body;
	pos_ctrl_status.nav_pitch = _att_sp.pitch_body;
	pos_ctrl_status.nav_bearing = _l1_control.nav_bearing();

	pos_ctrl_status.target_bearing = _l1_control.target_bearing();
	pos_ctrl_status.xtrack_error = _l1_control.crosstrack_error();

	pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_current_latitude, _current_longitude,
				  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

	pos_ctrl_status.acceptance_radius = _l1_control.switch_distance(500.0f);

	pos_ctrl_status.yaw_acceptance = NAN;

	pos_ctrl_status.timestamp = hrt_absolute_time();

	pos_ctrl_status.type = _type;

	_pos_ctrl_status_pub.publish(pos_ctrl_status);
}

void
FixedwingPositionControl::landing_status_publish()
{
	/*position_controller_landing_status_s pos_ctrl_landing_status = {};
	pos_ctrl_landing_status.slope_angle_rad = _landingslope.landing_slope_angle_rad();
	pos_ctrl_landing_status.horizontal_slope_displacement = _landingslope.horizontal_slope_displacement();
	pos_ctrl_landing_status.flare_length = _landingslope.flare_length();
	pos_ctrl_landing_status.abort_landing = _land_abort;
	pos_ctrl_landing_status.timestamp = hrt_absolute_time();
	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);*/
}

void
FixedwingPositionControl::abort_landing(bool abort)
{
	// only announce changes
	if (abort && !_land_abort) {
		mavlink_log_critical(&_mavlink_log_pub, "Landing aborted");
	}

	_land_abort = abort;
	landing_status_publish();
}

void
FixedwingPositionControl::get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
		position_setpoint_s &waypoint_next, bool flag_init)
{
	position_setpoint_s temp_prev = waypoint_prev;
	position_setpoint_s temp_next = waypoint_next;

	if (flag_init) {
		// previous waypoint: HDG_HOLD_SET_BACK_DIST meters behind us
		waypoint_from_heading_and_distance(_current_latitude, _current_longitude, heading + radians(180.0f),
						   HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

		// next waypoint: HDG_HOLD_DIST_NEXT meters in front of us
		waypoint_from_heading_and_distance(_current_latitude, _current_longitude, heading,
						   HDG_HOLD_DIST_NEXT, &temp_next.lat, &temp_next.lon);

	} else {
		// use the existing flight path from prev to next

		// previous waypoint: shifted HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

		// next waypoint: shifted -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST)
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   -(HDG_HOLD_REACHED_DIST + HDG_HOLD_DIST_NEXT), &temp_next.lat, &temp_next.lon);
	}

	waypoint_prev = temp_prev;
	waypoint_prev.alt = _hold_alt;
	waypoint_prev.valid = true;

	waypoint_next = temp_next;
	waypoint_next.alt = _hold_alt;
	waypoint_next.valid = true;
}

float
FixedwingPositionControl::get_terrain_altitude_takeoff(float takeoff_alt)
{
	float terrain_alt = _local_pos.ref_alt - (_local_pos.dist_bottom + _local_pos.z);

	if (PX4_ISFINITE(terrain_alt) && _local_pos.dist_bottom_valid) {
		return terrain_alt;
	}

	return takeoff_alt;
}

void
FixedwingPositionControl::update_desired_altitude(float dt)
{
	/*
	 * The complete range is -1..+1, so this is 6%
	 * of the up or down range or 3% of the total range.
	 */
	const float deadBand = 0.06f;

	/*
	 * The correct scaling of the complete range needs
	 * to account for the missing part of the slope
	 * due to the deadband
	 */
	const float factor = 1.0f - deadBand;

	/*
	 * Reset the hold altitude to the current altitude if the uncertainty
	 * changes significantly.
	 * This is to guard against uncommanded altitude changes
	 * when the altitude certainty increases or decreases.
	 */

	if (fabsf(_althold_epv - _local_pos.epv) > ALTHOLD_EPV_RESET_THRESH) {
		_hold_alt = _current_altitude;
		_althold_epv = _local_pos.epv;
	}

	// NASTY DISABLE
	if (1) {
		_hold_alt = _param_fw_x_hgt_target.get() + _local_pos.ref_alt;

		if (_man_active == true) {
			_hold_alt += _maneuver.get_test_hgt_sp();
		}

	} else {
		/*
		* Manual control has as convention the rotation around
		* an axis. Positive X means to rotate positively around
		* the X axis in NED frame, which is pitching down
		*/
		if (_manual_control_setpoint_altitude > deadBand) {
			/* pitching down */
			float pitch = -(_manual_control_setpoint_altitude - deadBand) / factor;

			if (pitch * _param_sinkrate_target.get() < _tecs.hgt_rate_setpoint()) {
				_hold_alt += (_param_sinkrate_target.get() * dt) * pitch;
				_manual_height_rate_setpoint_m_s = pitch * _param_sinkrate_target.get();
				_was_in_deadband = false;
			}

		} else if (_manual_control_setpoint_altitude < - deadBand) {
			/* pitching up */
			float pitch = -(_manual_control_setpoint_altitude + deadBand) / factor;

			if (pitch * _param_climbrate_target.get() > _tecs.hgt_rate_setpoint()) {
				_hold_alt += (_param_climbrate_target.get() * dt) * pitch;
				_manual_height_rate_setpoint_m_s = pitch * _param_climbrate_target.get();
				_was_in_deadband = false;
			}

		} else if (!_was_in_deadband) {
			/* store altitude at which manual.x was inside deadBand
			* The aircraft should immediately try to fly at this altitude
			* as this is what the pilot expects when he moves the stick to the center */
			_hold_alt = _current_altitude;
			_althold_epv = _local_pos.epv;
			_was_in_deadband = true;
			_manual_height_rate_setpoint_m_s = NAN;
		}
	}

	if (_vehicle_status.is_vtol) {
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || _vehicle_status.in_transition_mode) {
			_hold_alt = _current_altitude;
		}
	}

}

bool
FixedwingPositionControl::in_takeoff_situation()
{
	// a VTOL does not need special takeoff handling
	if (_vehicle_status.is_vtol) {
		return false;
	}

	// in air for < 10s
	return (hrt_elapsed_time(&_time_went_in_air) < 10_s)
	       && (_current_altitude <= _takeoff_ground_alt + _param_fw_clmbout_diff.get());
}

void
FixedwingPositionControl::do_takeoff_help(float *hold_altitude, float *pitch_limit_min)
{
	/* demand "climbout_diff" m above ground if user switched into this mode during takeoff */
	if (in_takeoff_situation()) {
		*hold_altitude = _takeoff_ground_alt + _param_fw_clmbout_diff.get();
		*pitch_limit_min = radians(10.0f);
	}
}

bool
FixedwingPositionControl::control_position(const hrt_abstime &now, const Vector2d &curr_pos,
		const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next)
{
	const float dt = math::constrain((now - _control_position_last_called) * 1e-6f, 0.01f, 0.05f);
	_control_position_last_called = now;
	_dt = dt;

	_l1_control.set_dt(dt);

	/* only run position controller in fixed-wing mode and during transitions for VTOL */
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return false;
	}

	bool setpoint = true;

	_att_sp.fw_control_yaw = false;		// by default we don't want yaw to be contoller directly with rudder
	_att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_OFF;		// by default we don't use flaps

	Vector2f nav_speed_2d{ground_speed};

	if (_airspeed_valid) {
		// l1 navigation logic breaks down when wind speed exceeds max airspeed
		// compute 2D groundspeed from airspeed-heading projection
		const Vector2f air_speed_2d{_airspeed * cosf(_yaw), _airspeed * sinf(_yaw)};

		// angle between air_speed_2d and ground_speed
		const float air_gnd_angle = acosf((air_speed_2d * ground_speed) / (air_speed_2d.length() * ground_speed.length()));

		// if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
		if ((fabsf(air_gnd_angle) > M_PI_2_F) || (ground_speed.length() < 3.0f)) {
			nav_speed_2d = air_speed_2d;
		}
	}

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* save time when airplane is in air */
	if (!_was_in_air && !_landed) {
		_was_in_air = true;
		_time_went_in_air = now;
		_takeoff_ground_alt = _current_altitude;
	}

	/* reset flag when airplane landed */
	if (_landed) {
		_was_in_air = false;
	}

	/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
	if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
		/* reset integrators */
		_tecs.reset_state();
		// ADD INTEGRATOR TECS
		_tecs_X.reset_state();

		_pi_X.reset_state();
	}

	if ((_control_mode.flag_control_auto_enabled || _control_mode.flag_control_offboard_enabled) && pos_sp_curr.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = FW_POSCTRL_MODE_AUTO;

		/* reset hold altitude */
		_hold_alt = _current_altitude;

		/* reset hold yaw */
		_hdg_hold_yaw = _yaw;

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

		/* restore TECS parameters, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_param_fw_t_spdweight.get());
		_tecs.set_height_error_time_constant(_param_fw_t_h_error_tc.get());

		Vector2d curr_wp{0, 0};
		Vector2d prev_wp{0, 0};

		if (_vehicle_status.in_transition_to_fw) {

			if (!PX4_ISFINITE(_transition_waypoint(0))) {
				double lat_transition, lon_transition;
				// create a virtual waypoint HDG_HOLD_DIST_NEXT meters in front of the vehicle which the L1 controller can track
				// during the transition
				waypoint_from_heading_and_distance(_current_latitude, _current_longitude, _yaw, HDG_HOLD_DIST_NEXT, &lat_transition,
								   &lon_transition);

				_transition_waypoint(0) = lat_transition;
				_transition_waypoint(1) = lon_transition;
			}


			curr_wp = prev_wp = _transition_waypoint;

		} else {
			/* current waypoint (the one currently heading for) */
			curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

			if (pos_sp_prev.valid) {
				prev_wp(0) = pos_sp_prev.lat;
				prev_wp(1) = pos_sp_prev.lon;

			} else {
				/*
				 * No valid previous waypoint, go for the current wp.
				 * This is automatically handled by the L1 library.
				 */
				prev_wp(0) = pos_sp_curr.lat;
				prev_wp(1) = pos_sp_curr.lon;
			}


			/* reset transition waypoint, will be set upon entering front transition */
			_transition_waypoint(0) = static_cast<double>(NAN);
			_transition_waypoint(1) = static_cast<double>(NAN);
		}

		/* Initialize attitude controller integrator reset flags to 0 */
		_att_sp.roll_reset_integral = false;
		_att_sp.pitch_reset_integral = false;
		_att_sp.yaw_reset_integral = false;

		float mission_airspeed = _param_fw_airspd_trim.get();

		if (PX4_ISFINITE(pos_sp_curr.cruising_speed) &&
		    pos_sp_curr.cruising_speed > 0.1f) {

			mission_airspeed = pos_sp_curr.cruising_speed;
		}

		float mission_throttle = _param_fw_thr_cruise.get();

		if (PX4_ISFINITE(pos_sp_curr.cruising_throttle) &&
		    pos_sp_curr.cruising_throttle >= 0.0f) {
			mission_throttle = pos_sp_curr.cruising_throttle;
		}

		float tecs_fw_thr_min;
		float tecs_fw_thr_max;
		float tecs_fw_mission_throttle;

		if (mission_throttle < _param_fw_thr_min.get()) {
			/* enable gliding with this waypoint */
			_tecs.set_speed_weight(2.0f);
			tecs_fw_thr_min = 0.0;
			tecs_fw_thr_max = 0.0;
			tecs_fw_mission_throttle = 0.0;

		} else {
			tecs_fw_thr_min = _param_fw_thr_min.get();
			tecs_fw_thr_max = _param_fw_thr_max.get();
			tecs_fw_mission_throttle = mission_throttle;
		}

		const float acc_rad = _l1_control.switch_distance(500.0f);

		uint8_t position_sp_type = pos_sp_curr.type;

		if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			// TAKEOFF: handle like a regular POSITION setpoint if already flying
			if (!in_takeoff_situation() && (_airspeed >= _param_fw_airspd_min.get() || !_airspeed_valid)) {
				// SETPOINT_TYPE_TAKEOFF -> SETPOINT_TYPE_POSITION
				position_sp_type = position_setpoint_s::SETPOINT_TYPE_POSITION;
			}

		} else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION
			   || pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			float dist_xy = -1.f;
			float dist_z = -1.f;

			const float dist = get_distance_to_point_global_wgs84(
						   (double)curr_wp(0), (double)curr_wp(1), pos_sp_curr.alt,
						   _current_latitude, _current_longitude, _current_altitude,
						   &dist_xy, &dist_z);

			if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
				// POSITION: achieve position setpoint altitude via loiter
				// close to waypoint, but altitude error greater than twice acceptance
				if ((dist >= 0.f)
				    && (dist_z > 2.f * _param_fw_clmbout_diff.get())
				    && (dist_xy < 2.f * math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)))) {
					// SETPOINT_TYPE_POSITION -> SETPOINT_TYPE_LOITER
					position_sp_type = position_setpoint_s::SETPOINT_TYPE_LOITER;
				}

			} else if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
				// LOITER: use SETPOINT_TYPE_POSITION to get to SETPOINT_TYPE_LOITER
				if ((dist >= 0.f)
				    && (dist_z > 2.f * _param_fw_clmbout_diff.get())
				    && (dist_xy > 2.f * math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)))) {
					// SETPOINT_TYPE_LOITER -> SETPOINT_TYPE_POSITION
					position_sp_type = position_setpoint_s::SETPOINT_TYPE_POSITION;
				}
			}
		}

		_type = position_sp_type;


		if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			_att_sp.thrust_body[0] = 0.0f;
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = radians(_param_fw_psp_off.get());

		} else if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			// waypoint is a plain navigation waypoint
			float position_sp_alt = pos_sp_curr.alt;

			// Altitude first order hold (FOH)
			if (pos_sp_prev.valid && PX4_ISFINITE(pos_sp_prev.alt) &&
			    ((pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_POSITION) ||
			     (pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_LOITER) ||
			     (pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF))
			   ) {
				const float d_curr_prev = get_distance_to_next_waypoint((double)curr_wp(0), (double)curr_wp(1),
							  pos_sp_prev.lat, pos_sp_prev.lon);

				// Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one
				if (d_curr_prev > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
					// Calculate distance to current waypoint
					const float d_curr = get_distance_to_next_waypoint((double)curr_wp(0), (double)curr_wp(1),
							     _current_latitude, _current_longitude);

					// Save distance to waypoint if it is the smallest ever achieved, however make sure that
					// _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp
					_min_current_sp_distance_xy = math::min(math::min(d_curr, _min_current_sp_distance_xy), d_curr_prev);

					// if the minimal distance is smaller than the acceptance radius, we should be at waypoint alt
					// navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude
					if (_min_current_sp_distance_xy > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
						// The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
						// radius around the current waypoint
						const float delta_alt = (pos_sp_curr.alt - pos_sp_prev.alt);
						const float grad = -delta_alt / (d_curr_prev - math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)));
						const float a = pos_sp_prev.alt - grad * d_curr_prev;

						position_sp_alt = a + grad * _min_current_sp_distance_xy;
					}
				}
			}

			_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, nav_speed_2d);
			_att_sp.roll_body = _l1_control.get_roll_setpoint();
			_att_sp.yaw_body = _l1_control.nav_bearing();
			_position_sp_alt = position_sp_alt;
			tecs_update_pitch_throttle(now, position_sp_alt,
						   calculate_target_airspeed(mission_airspeed, ground_speed),
						   radians(_param_fw_p_lim_min.get()),
						   radians(_param_fw_p_lim_max.get()),
						   tecs_fw_thr_min,
						   tecs_fw_thr_max,
						   tecs_fw_mission_throttle,
						   false,
						   radians(_param_fw_p_lim_min.get()));


		} else if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
			/* waypoint is a loiter waypoint */
			float loiter_radius = pos_sp_curr.loiter_radius;
			uint8_t loiter_direction = pos_sp_curr.loiter_direction;

			if (fabsf(pos_sp_curr.loiter_radius) < FLT_EPSILON) {
				loiter_radius = _param_nav_loiter_rad.get();
				loiter_direction = (loiter_radius > 0) ? 1 : -1;

			}

			_l1_control.navigate_loiter(curr_wp, curr_pos, loiter_radius, loiter_direction, nav_speed_2d);

			_att_sp.roll_body = _l1_control.get_roll_setpoint();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			float alt_sp = pos_sp_curr.alt;

			if (pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && pos_sp_next.valid
			    && _l1_control.circle_mode() && _param_fw_lnd_earlycfg.get()) {
				// We're in a loiter directly before a landing WP. Enable our landing configuration (flaps,
				// landing airspeed and potentially tighter altitude control) already such that we don't
				// have to do this switch (which can cause significant altitude errors) close to the ground.
				_tecs.set_height_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());
				mission_airspeed = _param_fw_lnd_airspd_sc.get() * _param_fw_airspd_min.get();
				_att_sp.apply_flaps = true;
			}

			if (in_takeoff_situation()) {
				alt_sp = max(alt_sp, _takeoff_ground_alt + _param_fw_clmbout_diff.get());
				_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-5.0f), radians(5.0f));
			}

			if (_land_abort) {
				if (pos_sp_curr.alt - _current_altitude  < _param_fw_clmbout_diff.get()) {
					// aborted landing complete, normal loiter over landing point
					abort_landing(false);

				} else {
					// continue straight until vehicle has sufficient altitude
					_att_sp.roll_body = 0.0f;
				}

				_tecs.set_height_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());
			}

			tecs_update_pitch_throttle(now, alt_sp,
						   calculate_target_airspeed(mission_airspeed, ground_speed),
						   radians(_param_fw_p_lim_min.get()),
						   radians(_param_fw_p_lim_max.get()),
						   tecs_fw_thr_min,
						   tecs_fw_thr_max,
						   tecs_fw_mission_throttle,
						   false,
						   radians(_param_fw_p_lim_min.get()));

		} else if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			control_landing(now, curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);

		} else if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			control_takeoff(now, curr_pos, ground_speed, pos_sp_prev, pos_sp_curr);
		}

		/* reset landing state */
		if (position_sp_type != position_setpoint_s::SETPOINT_TYPE_LAND) {
			reset_landing_state();
		}

		/* reset takeoff/launch state */
		if (position_sp_type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			reset_takeoff_state();
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	} else if (_control_mode.flag_control_velocity_enabled &&
		   _control_mode.flag_control_altitude_enabled) {
		/* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
		   heading is set to a distant waypoint */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _current_altitude;
			_hdg_hold_yaw = _yaw;
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;

			/* reset setpoints from other modes (auto) otherwise we won't
			 * level out without new manual input */
			_att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get());
			_att_sp.yaw_body = 0;
		}

		_control_mode_current = FW_POSCTRL_MODE_POSITION;

		float altctrl_airspeed = get_demanded_airspeed();
		_altctrl_airspeed = altctrl_airspeed;
		/* update desired altitude based on user pitch stick input */
		update_desired_altitude(dt);

		// if we assume that user is taking off then help by demanding altitude setpoint well above ground
		// and set limit to pitch angle to prevent steering into ground
		// this will only affect planes and not VTOL
		float pitch_limit_min = _param_fw_p_lim_min.get();
		do_takeoff_help(&_hold_alt, &pitch_limit_min);

		/* throttle limiting */
		throttle_max = 1.0f; //_param_fw_thr_max.get();

		if (_landed && (fabsf(_manual_control_setpoint_airspeed) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		tecs_update_pitch_throttle(now, _hold_alt,
					   altctrl_airspeed,
					   radians(_param_fw_p_lim_min.get()),
					   radians(_param_fw_p_lim_max.get()),
					   _param_fw_thr_min.get(),
					   throttle_max,
					   _param_fw_thr_cruise.get(),
					   false,
					   pitch_limit_min,
					   tecs_status_s::TECS_MODE_NORMAL,
					   _manual_height_rate_setpoint_m_s);

		/* heading control */
		if (fabsf(_manual_control_setpoint.y) < HDG_HOLD_MAN_INPUT_THRESH &&
		    fabsf(_manual_control_setpoint.r) < HDG_HOLD_MAN_INPUT_THRESH) {

			/* heading / roll is zero, lock onto current heading */
			if (fabsf(_yawrate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
				// little yaw movement, lock to current heading
				_yaw_lock_engaged = true;

			}

			/* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
			  to make sure the plane does not start rolling
			*/
			if (in_takeoff_situation()) {
				_hdg_hold_enabled = false;
				_yaw_lock_engaged = true;
			}

			if (_yaw_lock_engaged) {

				/* just switched back from non heading-hold to heading hold */
				if (!_hdg_hold_enabled) {
					_hdg_hold_enabled = true;
					_hdg_hold_yaw = _yaw;

					get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
				}

				/* we have a valid heading hold position, are we too close? */
				float dist = get_distance_to_next_waypoint(_current_latitude, _current_longitude, _hdg_hold_curr_wp.lat,
						_hdg_hold_curr_wp.lon);

				if (dist < HDG_HOLD_REACHED_DIST) {
					get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
				}

				Vector2d prev_wp{_hdg_hold_prev_wp.lat, _hdg_hold_prev_wp.lon};
				Vector2d curr_wp{_hdg_hold_curr_wp.lat, _hdg_hold_curr_wp.lon};

				/* populate l1 control setpoint */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);

				_att_sp.roll_body = _l1_control.get_roll_setpoint();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				if (in_takeoff_situation()) {
					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));
				}
			}
		}

		if (!_yaw_lock_engaged || fabsf(_manual_control_setpoint.y) >= HDG_HOLD_MAN_INPUT_THRESH ||
		    fabsf(_manual_control_setpoint.r) >= HDG_HOLD_MAN_INPUT_THRESH) {

			_hdg_hold_enabled = false;
			_yaw_lock_engaged = false;
			_att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get());
			_att_sp.yaw_body = 0;
		}

	} else if (_control_mode.flag_control_altitude_enabled) {
		/* ALTITUDE CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION && _control_mode_current != FW_POSCTRL_MODE_ALTITUDE) {
			/* Need to init because last loop iteration was in a different mode */
			//_hold_alt = _current_altitude; // <- former code

			_hold_alt = _param_fw_x_hgt_target.get() + _local_pos.ref_alt;

		}

		// MANEUVER SYSTEM
		man_active(dt);
		_control_mode_current = FW_POSCTRL_MODE_ALTITUDE;

		/* Get demanded airspeed */
		float altctrl_airspeed = get_demanded_airspeed();

		/* update desired altitude based on user pitch stick input */
		update_desired_altitude(dt);

		// if we assume that user is taking off then help by demanding altitude setpoint well above ground
		// and set limit to pitch angle to prevent steering into ground
		// this will only affect planes and not VTOL
		float pitch_limit_min = _param_fw_p_lim_min.get();
		do_takeoff_help(&_hold_alt, &pitch_limit_min);

		/* throttle limiting */
		throttle_max = 1.0f; //_param_fw_thr_max.get();

		if (_landed && (fabsf(_manual_control_setpoint_airspeed) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		// ---> TACKLE HERE
		_altctrl_airspeed = altctrl_airspeed;
		_position_sp_alt = _hold_alt;
		tecs_update_pitch_throttle(now, _hold_alt,
					   altctrl_airspeed,
					   radians(_param_fw_p_lim_min.get()),
					   radians(_param_fw_p_lim_max.get()),
					   _param_fw_thr_min.get(),
					   throttle_max,
					   _param_fw_thr_cruise.get(),
					   false,
					   pitch_limit_min,
					   tecs_status_s::TECS_MODE_NORMAL,
					   _manual_height_rate_setpoint_m_s);

		_att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get());
		_att_sp.yaw_body = 0;

	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;

		/* do not publish the setpoint */
		setpoint = false;

		// reset hold altitude
		_hold_alt = _current_altitude ;

		/* reset landing and takeoff state */
		if (!_last_manual) {
			reset_landing_state();
			reset_takeoff_state();
		}
	}

	/* Copy thrust output for publication */
	if (_control_mode_current == FW_POSCTRL_MODE_AUTO && // launchdetector only available in auto
	    pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
	    _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS &&
	    !_runway_takeoff.runwayTakeoffEnabled()) {

		/* making sure again that the correct thrust is used,
		 * without depending on library calls for safety reasons.
		   the pre-takeoff throttle and the idle throttle normally map to the same parameter. */
		_att_sp.thrust_body[0] = _param_fw_thr_idle.get();

	} else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
		   _runway_takeoff.runwayTakeoffEnabled()) {

		_att_sp.thrust_body[0] = _runway_takeoff.getThrottle(now, min(get_tecs_thrust(), throttle_max));

	} else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_att_sp.thrust_body[0] = 0.0f;

	} else if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
		_att_sp.thrust_body[0] = min(_att_sp.thrust_body[0], _param_fw_thr_max.get());

	} else {
		/* Copy thrust and pitch values from tecs */
		if (_landed) {
			// when we are landed state we want the motor to spin at idle speed
			_att_sp.thrust_body[0] = min(_param_fw_thr_idle.get(), throttle_max);

		} else {
			_att_sp.thrust_body[0] = min(get_tecs_thrust(), throttle_max);
		}
	}

	// decide when to use pitch setpoint from TECS because in some cases pitch
	// setpoint is generated by other means
	bool use_tecs_pitch = true;

	// auto runway takeoff
	use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_AUTO &&
			    pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
			    (_launch_detection_state == LAUNCHDETECTION_RES_NONE || _runway_takeoff.runwayTakeoffEnabled()));

	// flaring during landing
	use_tecs_pitch &= !(pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LAND && _land_noreturn_vertical);

	// manual attitude control
	use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_OTHER);

	if (use_tecs_pitch) {
		_att_sp.pitch_body = get_tecs_pitch();
	}

	if (_control_mode.flag_control_position_enabled) {
		_last_manual = false;

	} else {
		_last_manual = true;
	}

	return setpoint;
}

void
FixedwingPositionControl::control_takeoff(const hrt_abstime &now, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	/* current waypoint (the one currently heading for) */
	Vector2d curr_wp(pos_sp_curr.lat, pos_sp_curr.lon);
	Vector2d prev_wp{0, 0}; /* previous waypoint */

	if (pos_sp_prev.valid) {
		prev_wp(0) = pos_sp_prev.lat;
		prev_wp(1) = pos_sp_prev.lon;

	} else {
		/*
		 * No valid previous waypoint, go for the current wp.
		 * This is automatically handled by the L1 library.
		 */
		prev_wp(0) = pos_sp_curr.lat;
		prev_wp(1) = pos_sp_curr.lon;
	}

	// apply flaps for takeoff according to the corresponding scale factor set
	// via FW_FLAPS_TO_SCL
	_att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_TAKEOFF;

	// continuously reset launch detection and runway takeoff until armed
	if (!_control_mode.flag_armed) {
		_launchDetector.reset();
		_launch_detection_state = LAUNCHDETECTION_RES_NONE;
		_launch_detection_notify = 0;
	}

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now, _yaw, _current_latitude, _current_longitude);

			/* need this already before takeoff is detected
			 * doesn't matter if it gets reset when takeoff is detected eventually */
			_takeoff_ground_alt = _current_altitude;

			mavlink_log_info(&_mavlink_log_pub, "Takeoff on runway");
		}

		float terrain_alt = get_terrain_altitude_takeoff(_takeoff_ground_alt);

		// update runway takeoff helper
		_runway_takeoff.update(now, _airspeed, _current_altitude - terrain_alt,
				       _current_latitude, _current_longitude, &_mavlink_log_pub);

		/*
		 * Update navigation: _runway_takeoff returns the start WP according to mode and phase.
		 * If we use the navigator heading or not is decided later.
		 */
		_l1_control.navigate_waypoints(_runway_takeoff.getStartWP(), curr_wp, curr_pos, ground_speed);

		// update tecs
		const float takeoff_pitch_max_deg = _runway_takeoff.getMaxPitch(_param_fw_p_lim_max.get());

		tecs_update_pitch_throttle(now, pos_sp_curr.alt,
					   calculate_target_airspeed(_runway_takeoff.getMinAirspeedScaling() * _param_fw_airspd_min.get(), ground_speed),
					   radians(_param_fw_p_lim_min.get()),
					   radians(takeoff_pitch_max_deg),
					   _param_fw_thr_min.get(),
					   _param_fw_thr_max.get(), // XXX should we also set runway_takeoff_throttle here?
					   _param_fw_thr_cruise.get(),
					   _runway_takeoff.climbout(),
					   radians(_runway_takeoff.getMinPitch(_takeoff_pitch_min.get(), _param_fw_p_lim_min.get())),
					   tecs_status_s::TECS_MODE_TAKEOFF);

		// assign values
		_att_sp.roll_body = _runway_takeoff.getRoll(_l1_control.get_roll_setpoint());
		_att_sp.yaw_body = _runway_takeoff.getYaw(_l1_control.nav_bearing());
		_att_sp.fw_control_yaw = _runway_takeoff.controlYaw();
		_att_sp.pitch_body = _runway_takeoff.getPitch(get_tecs_pitch());

		// reset integrals except yaw (which also counts for the wheel controller)
		_att_sp.roll_reset_integral = _runway_takeoff.resetIntegrators();
		_att_sp.pitch_reset_integral = _runway_takeoff.resetIntegrators();

	} else {
		/* Perform launch detection */
		if (_launchDetector.launchDetectionEnabled() &&
		    _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {

			if (_control_mode.flag_armed) {
				/* Perform launch detection */

				/* Inform user that launchdetection is running every 4s */
				if ((now - _launch_detection_notify) > 4_s) {
					mavlink_log_critical(&_mavlink_log_pub, "Launch detection running");
					_launch_detection_notify = now;
				}

				/* Detect launch using body X (forward) acceleration */
				_launchDetector.update(now, _body_acceleration(0));

				/* update our copy of the launch detection state */
				_launch_detection_state = _launchDetector.getLaunchDetected();
			}

		} else	{
			/* no takeoff detection --> fly */
			_launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
		}

		/* Set control values depending on the detection state */
		if (_launch_detection_state != LAUNCHDETECTION_RES_NONE) {
			/* Launch has been detected, hence we have to control the plane. */

			_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
			_att_sp.roll_body = _l1_control.get_roll_setpoint();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			/* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
			 * full throttle, otherwise we use idle throttle */
			float takeoff_throttle = _param_fw_thr_max.get();

			if (_launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
				takeoff_throttle = _param_fw_thr_idle.get();
			}

			/* select maximum pitch: the launchdetector may impose another limit for the pitch
			 * depending on the state of the launch */
			const float takeoff_pitch_max_deg = _launchDetector.getPitchMax(_param_fw_p_lim_max.get());
			const float altitude_error = pos_sp_curr.alt - _current_altitude;

			/* apply minimum pitch and limit roll if target altitude is not within climbout_diff meters */
			if (_param_fw_clmbout_diff.get() > 0.0f && altitude_error > _param_fw_clmbout_diff.get()) {
				/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
				tecs_update_pitch_throttle(now, pos_sp_curr.alt,
							   _param_fw_airspd_trim.get(),
							   radians(_param_fw_p_lim_min.get()),
							   radians(takeoff_pitch_max_deg),
							   _param_fw_thr_min.get(),
							   takeoff_throttle,
							   _param_fw_thr_cruise.get(),
							   true,
							   radians(_takeoff_pitch_min.get()),
							   tecs_status_s::TECS_MODE_TAKEOFF);

				/* limit roll motion to ensure enough lift */
				_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-15.0f), radians(15.0f));

			} else {
				tecs_update_pitch_throttle(now, pos_sp_curr.alt,
							   calculate_target_airspeed(_param_fw_airspd_trim.get(), ground_speed),
							   radians(_param_fw_p_lim_min.get()),
							   radians(_param_fw_p_lim_max.get()),
							   _param_fw_thr_min.get(),
							   takeoff_throttle,
							   _param_fw_thr_cruise.get(),
							   false,
							   radians(_param_fw_p_lim_min.get()));
			}

		} else {
			/* Tell the attitude controller to stop integrating while we are waiting
			 * for the launch */
			_att_sp.roll_reset_integral = true;
			_att_sp.pitch_reset_integral = true;
			_att_sp.yaw_reset_integral = true;

			/* Set default roll and pitch setpoints during detection phase */
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = radians(_takeoff_pitch_min.get());
		}
	}
}

void
FixedwingPositionControl::control_landing(const hrt_abstime &now, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	/* current waypoint (the one currently heading for) */
	Vector2d curr_wp(pos_sp_curr.lat, pos_sp_curr.lon);
	Vector2d prev_wp{0, 0}; /* previous waypoint */

	if (pos_sp_prev.valid) {
		prev_wp(0) = pos_sp_prev.lat;
		prev_wp(1) = pos_sp_prev.lon;

	} else {
		/*
		 * No valid previous waypoint, go for the current wp.
		 * This is automatically handled by the L1 library.
		 */
		prev_wp(0) = pos_sp_curr.lat;
		prev_wp(1) = pos_sp_curr.lon;
	}

	// apply full flaps for landings. this flag will also trigger the use of flaperons
	// if they have been enabled using the corresponding parameter
	_att_sp.apply_flaps = vehicle_attitude_setpoint_s::FLAPS_LAND;

	// Enable tighter altitude control for landings
	_tecs.set_height_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());

	// save time at which we started landing and reset abort_landing
	if (_time_started_landing == 0) {
		reset_landing_state();
		_time_started_landing = now;
	}

	const float bearing_airplane_currwp = get_bearing_to_next_waypoint((double)curr_pos(0), (double)curr_pos(1),
					      (double)curr_wp(0), (double)curr_wp(1));

	float bearing_lastwp_currwp = bearing_airplane_currwp;

	if (pos_sp_prev.valid) {
		bearing_lastwp_currwp = get_bearing_to_next_waypoint((double)prev_wp(0), (double)prev_wp(1), (double)curr_wp(0),
					(double)curr_wp(1));
	}

	/* Horizontal landing control */
	/* switch to heading hold for the last meters, continue heading hold after */
	float wp_distance = get_distance_to_next_waypoint((double)curr_pos(0), (double)curr_pos(1), (double)curr_wp(0),
			    (double)curr_wp(1));

	/* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */
	float wp_distance_save = wp_distance;

	if (fabsf(wrap_pi(bearing_airplane_currwp - bearing_lastwp_currwp)) >= radians(90.0f)) {
		wp_distance_save = 0.0f;
	}

	// create virtual waypoint which is on the desired flight path but
	// some distance behind landing waypoint. This will make sure that the plane
	// will always follow the desired flight path even if we get close or past
	// the landing waypoint
	if (pos_sp_prev.valid) {
		double lat = pos_sp_curr.lat;
		double lon = pos_sp_curr.lon;

		create_waypoint_from_line_and_dist(pos_sp_curr.lat, pos_sp_curr.lon,
						   pos_sp_prev.lat, pos_sp_prev.lon, -1000.0f, &lat, &lon);

		curr_wp(0) = lat;
		curr_wp(1) = lon;
	}

	// we want the plane to keep tracking the desired flight path until we start flaring
	// if we go into heading hold mode earlier then we risk to be pushed away from the runway by cross winds
	if ((_param_fw_lnd_hhdist.get() > 0.0f) && !_land_noreturn_horizontal &&
	    ((wp_distance < _param_fw_lnd_hhdist.get()) || _land_noreturn_vertical)) {

		if (pos_sp_prev.valid) {
			/* heading hold, along the line connecting this and the last waypoint */
			_target_bearing = bearing_lastwp_currwp;

		} else {
			_target_bearing = _yaw;
		}

		_land_noreturn_horizontal = true;
		mavlink_log_info(&_mavlink_log_pub, "Landing, heading hold");
	}

	if (_land_noreturn_horizontal) {
		// heading hold
		_l1_control.navigate_heading(_target_bearing, _yaw, ground_speed);

	} else {
		// normal navigation
		_l1_control.navigate_waypoints(prev_wp, curr_wp, curr_pos, ground_speed);
	}

	_att_sp.roll_body = _l1_control.get_roll_setpoint();
	_att_sp.yaw_body = _l1_control.nav_bearing();

	if (_land_noreturn_horizontal) {
		/* limit roll motion to prevent wings from touching the ground first */
		_att_sp.roll_body = constrain(_att_sp.roll_body, radians(-10.0f), radians(10.0f));
	}

	/* Vertical landing control */
	/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */

	// default to no terrain estimation, just use landing waypoint altitude
	float terrain_alt = pos_sp_curr.alt;

	if (_param_fw_lnd_useter.get() == 1) {
		if (_local_pos.dist_bottom_valid) {
			// all good, have valid terrain altitude
			float terrain_vpos = _local_pos.dist_bottom + _local_pos.z;
			terrain_alt = (_local_pos.ref_alt - terrain_vpos);
			_t_alt_prev_valid = terrain_alt;
			_time_last_t_alt = now;

		} else if (_time_last_t_alt == 0) {
			// we have started landing phase but don't have valid terrain
			// wait for some time, maybe we will soon get a valid estimate
			// until then just use the altitude of the landing waypoint
			if ((now - _time_started_landing) < 10_s) {
				terrain_alt = pos_sp_curr.alt;

			} else {
				// still no valid terrain, abort landing
				terrain_alt = pos_sp_curr.alt;
				abort_landing(true);
			}

		} else if ((!_local_pos.dist_bottom_valid && (now - _time_last_t_alt) < T_ALT_TIMEOUT)
			   || _land_noreturn_vertical) {
			// use previous terrain estimate for some time and hope to recover
			// if we are already flaring (land_noreturn_vertical) then just
			//  go with the old estimate
			terrain_alt = _t_alt_prev_valid;

		} else {
			// terrain alt was not valid for long time, abort landing
			terrain_alt = _t_alt_prev_valid;
			abort_landing(true);
		}
	}

	/* Check if we should start flaring with a vertical and a
	 * horizontal limit (with some tolerance)
	 * The horizontal limit is only applied when we are in front of the wp
	 */
	if ((_current_altitude < terrain_alt + _landingslope.flare_relative_alt()) ||
	    _land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out

		/* land with minimal speed */

		/* force TECS to only control speed with pitch, altitude is only implicitly controlled now */
		// _tecs.set_speed_weight(2.0f);

		/* kill the throttle if param requests it */
		float throttle_max = 1.0f; //_param_fw_thr_max.get();

		/* enable direct yaw control using rudder/wheel */
		if (_land_noreturn_horizontal) {
			_att_sp.yaw_body = _target_bearing;
			_att_sp.fw_control_yaw = true;
		}

		if (((_current_altitude < terrain_alt + _landingslope.motor_lim_relative_alt()) &&
		     (wp_distance_save < _landingslope.flare_length() + 5.0f)) || // Only kill throttle when close to WP
		    _land_motor_lim) {
			throttle_max = min(throttle_max, _param_fw_thr_lnd_max.get());

			if (!_land_motor_lim) {
				_land_motor_lim  = true;
				mavlink_log_info(&_mavlink_log_pub, "Landing, limiting throttle");
			}
		}

		float flare_curve_alt_rel = _landingslope.getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp,
					    bearing_airplane_currwp);

		/* avoid climbout */
		if ((_flare_curve_alt_rel_last < flare_curve_alt_rel && _land_noreturn_vertical) || _land_stayonground) {
			flare_curve_alt_rel = 0.0f; // stay on ground
			_land_stayonground = true;
		}

		const float airspeed_land = _param_fw_lnd_airspd_sc.get() * _param_fw_airspd_min.get();
		const float throttle_land = _param_fw_thr_min.get() + (_param_fw_thr_max.get() - _param_fw_thr_min.get()) * 0.1f;

		tecs_update_pitch_throttle(now, terrain_alt + flare_curve_alt_rel,
					   calculate_target_airspeed(airspeed_land, ground_speed),
					   radians(_param_fw_lnd_fl_pmin.get()),
					   radians(_param_fw_lnd_fl_pmax.get()),
					   0.0f,
					   throttle_max,
					   throttle_land,
					   false,
					   _land_motor_lim ? radians(_param_fw_lnd_fl_pmin.get()) : radians(_param_fw_p_lim_min.get()),
					   _land_motor_lim ? tecs_status_s::TECS_MODE_LAND_THROTTLELIM : tecs_status_s::TECS_MODE_LAND);

		if (!_land_noreturn_vertical) {
			// just started with the flaring phase
			_flare_pitch_sp = radians(_param_fw_psp_off.get());
			_flare_height = _current_altitude - terrain_alt;
			mavlink_log_info(&_mavlink_log_pub, "Landing, flaring");
			_land_noreturn_vertical = true;

		} else {
			if (_local_pos.vz > 0.1f) {
				_flare_pitch_sp = radians(_param_fw_lnd_fl_pmin.get()) *
						  constrain((_flare_height - (_current_altitude - terrain_alt)) / _flare_height, 0.0f, 1.0f);
			}

			// otherwise continue using previous _flare_pitch_sp
		}

		_att_sp.pitch_body = _flare_pitch_sp;
		_flare_curve_alt_rel_last = flare_curve_alt_rel;

	} else {

		/* intersect glide slope:
		 * minimize speed to approach speed
		 * if current position is higher than the slope follow the glide slope (sink to the
		 * glide slope)
		 * also if the system captures the slope it should stay
		 * on the slope (bool land_onslope)
		 * if current position is below the slope continue at previous wp altitude
		 * until the intersection with slope
		 * */

		float altitude_desired = terrain_alt;

		const float landing_slope_alt_rel_desired = _landingslope.getLandingSlopeRelativeAltitudeSave(wp_distance,
				bearing_lastwp_currwp, bearing_airplane_currwp);

		if (_current_altitude > terrain_alt + landing_slope_alt_rel_desired || _land_onslope) {
			/* stay on slope */
			altitude_desired = terrain_alt + landing_slope_alt_rel_desired;

			if (!_land_onslope) {
				mavlink_log_info(&_mavlink_log_pub, "Landing, on slope");
				_land_onslope = true;
			}

		} else {
			/* continue horizontally */
			if (pos_sp_prev.valid) {
				altitude_desired = pos_sp_prev.alt;

			} else {
				altitude_desired = _current_altitude;
			}
		}

		const float airspeed_approach = _param_fw_lnd_airspd_sc.get() * _param_fw_airspd_min.get();

		tecs_update_pitch_throttle(now, altitude_desired,
					   calculate_target_airspeed(airspeed_approach, ground_speed),
					   radians(_param_fw_p_lim_min.get()),
					   radians(_param_fw_p_lim_max.get()),
					   _param_fw_thr_min.get(),
					   _param_fw_thr_max.get(),
					   _param_fw_thr_cruise.get(),
					   false,
					   radians(_param_fw_p_lim_min.get()));
	}
}

float
FixedwingPositionControl::get_tecs_pitch()
{
	if (_is_tecs_running) {
		//if (_man_active) {
			if (_param_fw_x_ctrl_sel.get() == 1) {
				return _tecs_X.get_pitch_setpoint() + radians(_param_fw_psp_off.get());

			} else if (_param_fw_x_ctrl_sel.get() == 2) {
				return _pi_X.get_pitch_setpoint() + radians(_param_fw_psp_off.get()); // PI OUTPUT HERE
			} else {
				return _tecs.get_pitch_setpoint() + radians(_param_fw_psp_off.get());
			}

		//} else {
			//reset the integrators to the last output
			// _tecs_X.init_integrator_throttle(_tecs.get_throttle_setpoint());
			// _tecs_X.init_integrator_pitch(_tecs.get_pitch_setpoint() + radians(_param_fw_psp_off.get()));
			// _pi_X.init_integrator_throttle(_tecs.get_throttle_setpoint()); //FixedwingPositionControl::get_tecs_thrust()
			// _pi_X.init_integrator_pitch(_tecs.get_pitch_setpoint() + radians(
			// 				    _param_fw_psp_off.get())); //FixedwingPositionControl::get_tecs_pitch()
			// return _tecs.get_pitch_setpoint() + radians(_param_fw_psp_off.get());
		//}

	}

	// return level flight pitch offset to prevent stale tecs state when it's not running
	return radians(_param_fw_psp_off.get());
}

float
FixedwingPositionControl::get_tecs_thrust()
{
	if (_is_tecs_running) {
		//if (_man_active) {
			if (_param_fw_x_ctrl_sel.get() == 1) {
				return _tecs_X.get_throttle_setpoint();

			} else if (_param_fw_x_ctrl_sel.get() == 2) {
				return _pi_X.get_throttle_setpoint(); // PI OUTPUT HERE

			} else {
				return _tecs.get_throttle_setpoint();
			}
		//}



	}

	// return 0 to prevent stale tecs state when it's not running
	return 0.0f;
}

void
FixedwingPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* only run controller if position changed */

	if (_local_pos_sub.update(&_local_pos)) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		vehicle_global_position_s gpos;

		if (_global_pos_sub.update(&gpos)) {
			_current_latitude = gpos.lat;
			_current_longitude = gpos.lon;
			_current_groundlevel = gpos.terrain_alt;
		}

		_current_altitude = -_local_pos.z + _local_pos.ref_alt; // Altitude AMSL in meters

		// handle estimator reset events. we only adjust setpoins for manual modes
		if (_control_mode.flag_control_manual_enabled) {
			if (_control_mode.flag_control_altitude_enabled && _local_pos.vz_reset_counter != _alt_reset_counter) {
				_hold_alt += -_local_pos.delta_z;
				// make TECS accept step in altitude and demanded altitude
				_tecs.handle_alt_step(-_local_pos.delta_z, _current_altitude);
				_tecs_X.handle_alt_step(-_local_pos.delta_z, _current_altitude);
				_pi_X.handle_alt_step(-_local_pos.delta_z, _current_altitude);
			}

			// adjust navigation waypoints in position control mode
			if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
			    && _local_pos.vxy_reset_counter != _pos_reset_counter) {

				// reset heading hold flag, which will re-initialise position control
				_hdg_hold_enabled = false;
			}
		}

		// update the reset counters in any case
		_alt_reset_counter = _local_pos.vz_reset_counter;
		_pos_reset_counter = _local_pos.vxy_reset_counter;


		if (_control_mode.flag_control_offboard_enabled) {
			// Convert Local setpoints to global setpoints
			if (!map_projection_initialized(&_global_local_proj_ref)
			    || (_global_local_proj_ref.timestamp != _local_pos.ref_timestamp)) {

				map_projection_init_timestamped(&_global_local_proj_ref, _local_pos.ref_lat, _local_pos.ref_lon,
								_local_pos.ref_timestamp);
				_global_local_alt0 = _local_pos.ref_alt;
			}

			vehicle_local_position_setpoint_s trajectory_setpoint;

			if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
				if (PX4_ISFINITE(trajectory_setpoint.x) && PX4_ISFINITE(trajectory_setpoint.y) && PX4_ISFINITE(trajectory_setpoint.z)) {
					double lat;
					double lon;

					if (map_projection_reproject(&_global_local_proj_ref, trajectory_setpoint.x, trajectory_setpoint.y, &lat, &lon) == 0) {
						_pos_sp_triplet = {}; // clear any existing

						_pos_sp_triplet.timestamp = trajectory_setpoint.timestamp;
						_pos_sp_triplet.current.timestamp = trajectory_setpoint.timestamp;
						_pos_sp_triplet.current.valid = true;
						_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
						_pos_sp_triplet.current.lat = lat;
						_pos_sp_triplet.current.lon = lon;
						_pos_sp_triplet.current.alt = _global_local_alt0 - trajectory_setpoint.z;
						_pos_sp_triplet.current.cruising_speed = NAN; // ignored
						_pos_sp_triplet.current.cruising_throttle = NAN; // ignored
					}

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Invalid offboard setpoint");
				}
			}

		} else {
			if (_pos_sp_triplet_sub.update(&_pos_sp_triplet)) {
				// reset the altitude foh (first order hold) logic
				_min_current_sp_distance_xy = FLT_MAX;
			}
		}

		airspeed_poll();
		manual_control_setpoint_poll();
		vehicle_attitude_poll();
		vehicle_command_poll();
		vehicle_control_mode_poll();

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		Vector2d curr_pos(_current_latitude, _current_longitude);
		Vector2f ground_speed(_local_pos.vx, _local_pos.vy);

		/*
		 * Attempt to control position, on success (= sensors present and not in manual mode),
		 * publish setpoint.
		 */
		if (control_position(_local_pos.timestamp, curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current,
				     _pos_sp_triplet.next)) {

			if (_control_mode.flag_control_manual_enabled) {
				_att_sp.roll_body = constrain(_att_sp.roll_body, -radians(_param_fw_man_r_max.get()),
							      radians(_param_fw_man_r_max.get()));
				_att_sp.pitch_body = constrain(_att_sp.pitch_body, -radians(_param_fw_man_p_max.get()),
							       radians(_param_fw_man_p_max.get()));
			}

			if (_control_mode.flag_control_position_enabled ||
			    _control_mode.flag_control_velocity_enabled ||
			    _control_mode.flag_control_acceleration_enabled ||
			    _control_mode.flag_control_altitude_enabled) {

				const Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
				q.copyTo(_att_sp.q_d);

				_att_sp.timestamp = hrt_absolute_time();
				_attitude_sp_pub.publish(_att_sp);

				// only publish status in full FW mode
				if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    || _vehicle_status.in_transition_mode) {
					status_publish();

				}
			}
		}

		perf_end(_loop_perf);
	}
}

void
FixedwingPositionControl::reset_takeoff_state(bool force)
{
	// only reset takeoff if !armed or just landed
	if (!_control_mode.flag_armed || (_was_in_air && _landed) || force) {

		_runway_takeoff.reset();

		_launchDetector.reset();
		_launch_detection_state = LAUNCHDETECTION_RES_NONE;
		_launch_detection_notify = 0;

	} else {
		_launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
	}
}

void
FixedwingPositionControl::reset_landing_state()
{
	_time_started_landing = 0;

	// reset terrain estimation relevant values
	_time_last_t_alt = 0;

	_land_noreturn_horizontal = false;
	_land_noreturn_vertical = false;
	_land_stayonground = false;
	_land_motor_lim = false;
	_land_onslope = false;

	// reset abort land, unless loitering after an abort
	if (_land_abort && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) {

		abort_landing(false);
	}
}

void
FixedwingPositionControl::tecs_update_pitch_throttle(const hrt_abstime &now, float alt_sp, float airspeed_sp,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		bool climbout_mode, float climbout_pitch_min_rad,
		uint8_t mode, float hgt_rate_sp)
{
	const float dt = math::constrain((now - _last_tecs_update) * 1e-6f, 0.01f, 0.05f);
	_last_tecs_update = now;

	// do not run TECS if we are not in air
	bool run_tecs = !_landed;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	// (it should also not run during VTOL blending because airspeed is too low still)
	if (_vehicle_status.is_vtol) {
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || _vehicle_status.in_transition_mode) {
			run_tecs = false;
		}

		if (_vehicle_status.in_transition_mode) {
			// we're in transition
			_was_in_transition = true;

			// set this to transition airspeed to init tecs correctly
			if (_param_fw_arsp_mode.get() == 1 && PX4_ISFINITE(_param_airspeed_trans)) {
				// some vtols fly without airspeed sensor
				_asp_after_transition = _param_airspeed_trans;

			} else {
				_asp_after_transition = _airspeed;
			}

			_asp_after_transition = constrain(_asp_after_transition, _param_fw_airspd_min.get(), _param_fw_airspd_max.get());

		} else if (_was_in_transition) {
			// after transition we ramp up desired airspeed from the speed we had coming out of the transition
			_asp_after_transition += dt * 2.0f; // increase 2m/s

			if (_asp_after_transition < airspeed_sp && _airspeed < airspeed_sp) {
				airspeed_sp = max(_asp_after_transition, _airspeed);

			} else {
				_was_in_transition = false;
				_asp_after_transition = 0.0f;
			}
		}
	}

	_is_tecs_running = run_tecs;

	if (!run_tecs) {
		// next time we run TECS we should reinitialize states
		_reinitialize_tecs = true;
		return;
	}

	if (_reinitialize_tecs) {
		_tecs.reset_state();
		_tecs_X.reset_state();
		_reinitialize_tecs = false;
	}

	if (_vehicle_status.engine_failure) {
		/* Force the slow downwards spiral */
		pitch_min_rad = radians(-1.0f);
		pitch_max_rad = radians(5.0f);
	}

	/* No underspeed protection in landing mode */
	_tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
					      || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

	_tecs_X.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
						|| mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

	/* tell TECS to update its state, but let it know when it cannot actually control the plane */
	bool in_air_alt_control = (!_landed &&
				   (_control_mode.flag_control_auto_enabled ||
				    _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_altitude_enabled));

	/* update TECS vehicle state estimates */
	_tecs.update_vehicle_state_estimates(_airspeed, _body_acceleration(0), (_local_pos.timestamp > 0), in_air_alt_control,
					     _current_altitude, _local_pos.vz);

	_tecs_X.update_vehicle_state_estimates(_airspeed, _body_acceleration(0), (_local_pos.timestamp > 0), in_air_alt_control,
					       _current_altitude, _local_pos.vz);

	_pi_X.update_vehicle_state_estimates(_airspeed, _body_acceleration(0), (_local_pos.timestamp > 0), in_air_alt_control,
					     _current_altitude, _local_pos.vz);

	/* scale throttle cruise by baro pressure */
	if (_param_fw_thr_alt_scl.get() > FLT_EPSILON) {
		vehicle_air_data_s air_data;

		if (_vehicle_air_data_sub.copy(&air_data)) {
			if (PX4_ISFINITE(air_data.baro_pressure_pa) && PX4_ISFINITE(_param_fw_thr_alt_scl.get())) {
				// scale throttle as a function of sqrt(p0/p) (~ EAS -> TAS at low speeds and altitudes ignoring temperature)
				const float eas2tas = sqrtf(CONSTANTS_STD_PRESSURE_PA / air_data.baro_pressure_pa);
				const float scale = constrain((eas2tas - 1.0f) * _param_fw_thr_alt_scl.get() + 1.f, 1.f, 2.f);

				throttle_max = 1.0f; //constrain(throttle_max * scale, throttle_min, 1.0f);
				throttle_cruise = constrain(throttle_cruise * scale, throttle_min + 0.01f, throttle_max - 0.01f);
			}
		}
	}

	_tecs.update_pitch_throttle(_pitch - radians(_param_fw_psp_off.get()),
				    _current_altitude, alt_sp,
				    airspeed_sp, _airspeed, _eas2tas,
				    climbout_mode,
				    climbout_pitch_min_rad - radians(_param_fw_psp_off.get()),
				    throttle_min, throttle_max, throttle_cruise,
				    pitch_min_rad - radians(_param_fw_psp_off.get()),
				    pitch_max_rad - radians(_param_fw_psp_off.get()),
				    _param_climbrate_target.get(), _param_sinkrate_target.get(), hgt_rate_sp);

	_tecs_X.update_pitch_throttle(_pitch - radians(_param_fw_psp_off.get()),
				      _current_altitude, alt_sp,
				      airspeed_sp, _airspeed, _eas2tas,
				      climbout_mode,
				      climbout_pitch_min_rad - radians(_param_fw_psp_off.get()),
				      throttle_min, throttle_max, throttle_cruise,
				      pitch_min_rad - radians(_param_fw_psp_off.get()),
				      pitch_max_rad - radians(_param_fw_psp_off.get()),
				      _param_climbrate_x_target.get(), _param_sinkrate_x_target.get(), hgt_rate_sp);

	_pi_X.update_pitch_throttle(_pitch - radians(_param_fw_psp_off.get()),
				    _current_altitude, alt_sp,
				    airspeed_sp, _airspeed, _eas2tas,
				    climbout_mode,
				    climbout_pitch_min_rad - radians(_param_fw_psp_off.get()),
				    throttle_min, throttle_max, throttle_cruise,
				    pitch_min_rad - radians(_param_fw_psp_off.get()),
				    pitch_max_rad - radians(_param_fw_psp_off.get()),
				    _param_climbrate_x_target.get(), _param_sinkrate_x_target.get(), hgt_rate_sp);
	tecs_status_publish();
}

bool
FixedwingPositionControl::man_active(float dt)
{
	/*This is the first entry of the maneuver setting and executed once.
	Here the option to reset the experimental controller states is added*/
	if ((_manual_control_setpoint.aux1 > 0.8f) && (_man_active == false)) {
		//_tecs_X.reset_state(); /*comment out if the integrators from the base px4 tecs should be used*/
		_man_active = true;
		_maneuver.init_trajectory();
		return true;

	} else if ((_manual_control_setpoint.aux1 > 0.8f) && (_man_active == true)) {
		_maneuver.update_trajectory(dt);
		return true;

	} else {
		_maneuver.reset_trajectory();
		_man_active = false;
		return false;
	}
}
// == SPAWN TASK ===============================================================

int FixedwingPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingPositionControl *instance = new FixedwingPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_pos_control_l1 is the fixed wing position controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_pos_control_l1", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[])
{
	return FixedwingPositionControl::main(argc, argv);
}
