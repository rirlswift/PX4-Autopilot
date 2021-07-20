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

/**
 * @file TargetEstimator.cpp
 *
 */

#include "TargetEstimator.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

TargetEstimator::TargetEstimator() : ModuleParams(nullptr)
{
	// initialize parameters
	parameters_update(true);

	// Initialize filter
	reset();
}

void TargetEstimator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void TargetEstimator::update()
{
	// compute deltatime between update() calls
	if (_last_iteration_timestamp == 0) {
		_last_iteration_timestamp = hrt_absolute_time();
		return;
	}

	const float deltatime = (hrt_absolute_time() - _last_iteration_timestamp) / 1000000.0f;
	_last_iteration_timestamp = hrt_absolute_time();

	parameters_update();
	update_filter_gains(&_filter_gains);


	// Get GPS reference location for NED frame, needed for projection
	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.copy(&_vehicle_local_position);
	}

	// Perform sensor fusion update if there's a new GPS message from the follow-target
	if (_follow_target_sub.updated()) {
		follow_target_s follow_target;
		_follow_target_sub.copy(&follow_target);

		measurement_update(follow_target);

	} else {
		prediction_update(deltatime);
	}

	// Keep position estimate as the last known position
	// but stop moving the estimate
	if (is_stale(GPS_MESSAGE_STALE_TIMEOUT_MS)) {
		_filter_states.v_ned_est = matrix::Vector3f{0.0f, 0.0f, 0.0f};
		_filter_states.a_ned_est = matrix::Vector3f{0.0f, 0.0f, 0.0f};
	}

	const bool states_are_finite = _filter_states.is_finite();

	if (!states_are_finite) {
		reset();
	}

	// Publish estimator message
	follow_target_estimator_s follow_target_estimator{};
	follow_target_estimator.timestamp = hrt_absolute_time();
	follow_target_estimator.valid = states_are_finite;
	follow_target_estimator.stale = is_stale(GPS_MESSAGE_STALE_TIMEOUT_MS);
	follow_target_estimator.last_filter_reset_timestamp = _last_filter_reset_timestamp;
	follow_target_estimator.lat_est = get_lat_lon_alt_est()(0);
	follow_target_estimator.lon_est = get_lat_lon_alt_est()(1);
	follow_target_estimator.alt_est = get_lat_lon_alt_est()(2);
	follow_target_estimator.x_est = _filter_states.x_ned_est(0);
	follow_target_estimator.y_est = _filter_states.x_ned_est(1);
	follow_target_estimator.z_est = _filter_states.x_ned_est(2);
	follow_target_estimator.vx_est = _filter_states.v_ned_est(0);
	follow_target_estimator.vy_est = _filter_states.v_ned_est(1);
	follow_target_estimator.vz_est = _filter_states.v_ned_est(2);
	follow_target_estimator.ax_est = _filter_states.a_ned_est(0);
	follow_target_estimator.ay_est = _filter_states.a_ned_est(1);
	follow_target_estimator.az_est = _filter_states.a_ned_est(2);
	_follow_target_estimator_pub.publish(follow_target_estimator);
}

void TargetEstimator::update_filter_gains(filter_gains_s *filter_gains) const
{
	const float responsiveness_param = math::constrain((float) _param_nav_ft_rs.get(), .1F, 1.0F);

	if (fabsf(filter_gains->responsiveness - responsiveness_param) < FLT_EPSILON) {
		// Parameter did not change since last execution. Skip calculations
		return;
	}

	filter_gains->responsiveness = responsiveness_param;

	// The "G" gain is equivalent to "(1-responsiveness)", but beta is required for H and K gains
	// From alpha-beta-gamma filter equations: G = 1-beta^3
	// Therefore: beta = (1-Gp)^(1/3) = (1-(1-responsiveness))^(1/3) = (r)^(1/3)
	const float beta_p = std::pow((filter_gains->responsiveness), 1.0f / 3.0f);
	const float beta_v = 0.9f * beta_p; // velocity fusion gain is slightly lower. TODO: individual parameter?

	// Estimator gains for horizontal position update
	filter_gains->G_p = 1.0f - beta_p * beta_p * beta_p;
	filter_gains->H_p = 1.5f * (1.0f - beta_p) * (1.0f - beta_p) * (1.0f + beta_p);
	filter_gains->K_p = 0.5f * (1.0f - beta_p) * (1.0f - beta_p) * (1.0f - beta_p);

	// Estimator gains for velocity update
	filter_gains->G_v = 1.0f - beta_v * beta_v ;
	filter_gains->H_v = (1.0f - beta_v) * (1.0f - beta_v);
}

void TargetEstimator::measurement_update(follow_target_s follow_target)
{
	// Don't perform measurement update if two follow_target messages with identical timestamps are used
	// This can happen when using the MAVSDK and more than one outgoing follow_target message is queued.
	if (follow_target.timestamp == _last_follow_target_timestamp) {
		return;
	}

	// Skip measurements that lie in the past. Can after an estimator reset
	if (_last_position_fusion_timestamp >= follow_target.timestamp
	    && _last_velocity_fusion_timestamp >= follow_target.timestamp) {
		return;
	}

	// Need at least one vehicle_local_position before estimator can work
	if (_vehicle_local_position.timestamp == 0) {
		return;
	}

	// Decompose follow_target message into the individual measurements for position and velocity
	matrix::Vector3f v_measured{follow_target.vx, follow_target.vy, follow_target.vz};
	matrix::Vector3f x_measured{NAN, NAN, -(follow_target.alt - _vehicle_local_position.ref_alt)};
	map_projection_init(&_reference_position, _vehicle_local_position.ref_lat, _vehicle_local_position.ref_lon);
	map_projection_project(&_reference_position,
			       follow_target.lat, follow_target.lon, &x_measured(0), &x_measured(1));

	// Initialize filter if necessary
	if (_last_follow_target_timestamp == 0) {
		_filter_states.x_ned_est = x_measured;
		_filter_states.v_ned_est = v_measured;
		_filter_states.a_ned_est = {0.0f, 0.0f, 0.0f};
	}

	_last_follow_target_timestamp = follow_target.timestamp;

	// Temporary copy to not mix old and new values during the fitler update
	filter_states_s filter_states_prev = _filter_states;

	// Filter duplicate GPS POS and VEL messages
	// QGC sends the same GPS coordinates multiple times per second, even though the phone's GPS
	// typically only updates at 1 Hz
	const bool target_moving = v_measured.norm() > MINIMUM_SPEED_FOR_TARGET_MOVING;
	const bool gps_pos_unchanged = matrix::Vector3f(x_measured - _x_measurement_old).norm() < 2.0f * FLT_EPSILON;
	const bool pos_fusion_old_enough = hrt_absolute_time() - _last_position_fusion_timestamp >
					   MINIMUM_TIME_BETWEEN_POS_FUSIONS_MS * 1000;

	// Fuse position measurement
	if (pos_fusion_old_enough && ((target_moving && !gps_pos_unchanged) || !target_moving)) {
		const float dt_update_pos = (follow_target.timestamp - _last_position_fusion_timestamp) / 1000000.0f; // seconds
		_last_position_fusion_timestamp = follow_target.timestamp;

		position_meas_update(dt_update_pos, x_measured, &filter_states_prev, &_filter_states);
		_x_measurement_old = x_measured;
	}

	// Fuse velocity measurement
	const float gps_vel_change = matrix::Vector3f(v_measured - _v_measurement_old).norm();
	const bool gps_vel_stale = gps_vel_change < 2 * FLT_EPSILON;
	const bool vel_fusion_old_enough = hrt_absolute_time() - _last_velocity_fusion_timestamp >
					   MINIMUM_TIME_BETWEEN_VEL_FUSIONS_MS * 1000;

	if (gps_vel_stale) {
		// Reset XY-acceleration while GPS is stale to avoid filter runoffs
		_filter_states.a_ned_est = matrix::Vector3f{0.0f, 0.0f, 0.0f};
	}

	if (vel_fusion_old_enough && ((target_moving && !gps_vel_stale) || !target_moving)) {
		// Wait with first velocity fusion until at least one position fusion has been done
		if (PX4_ISFINITE(_filter_states.x_ned_est(0)) && PX4_ISFINITE(_filter_states.x_ned_est(1))
		    && PX4_ISFINITE(_filter_states.x_ned_est(2))) {

			const float dt_update_vel = (follow_target.timestamp - _last_velocity_fusion_timestamp) / 1000000.0f; // seconds
			_last_velocity_fusion_timestamp = follow_target.timestamp;

			velocity_meas_update(dt_update_vel, v_measured, &filter_states_prev, &_filter_states);
			_v_measurement_old = v_measured;
		}
	}
}

void TargetEstimator::position_meas_update(float deltatime, matrix::Vector3f x_measured,
		filter_states_s const *const filter_states_prev, filter_states_s *const filter_states) const
{
	// Position update
	filter_states->x_ned_est = filter_states_prev->x_ned_est + _filter_gains.G_p * (x_measured -
				   filter_states_prev->x_ned_est);


	// Velocity update
	filter_states->v_ned_est = filter_states_prev->v_ned_est + _filter_gains.H_p / (deltatime) *
				   (x_measured - filter_states_prev->x_ned_est);

	// Acceleration update
	filter_states->a_ned_est = filter_states_prev->a_ned_est + 2.0f * _filter_gains.K_p / (deltatime * deltatime) *
				   (x_measured - filter_states_prev->x_ned_est);
	filter_states->saturate_acceleration(ACCELERATION_SATURATION);
}

void TargetEstimator::velocity_meas_update(float deltatime, matrix::Vector3f v_measured,
		filter_states_s const *const filter_states_prev,
		filter_states_s *const filter_states) const
{
	// Velocity update
	filter_states->v_ned_est = filter_states_prev->v_ned_est + _filter_gains.G_v * (v_measured -
				   filter_states_prev->v_ned_est);

	// Acceleration update
	filter_states->a_ned_est = filter_states_prev->a_ned_est + _filter_gains.H_v / (deltatime) *
				   (v_measured - filter_states_prev->v_ned_est);
	filter_states->saturate_acceleration(ACCELERATION_SATURATION);
}

void TargetEstimator::prediction_update(float deltatime)
{
	// Temporary copy to not mix old and new values during the update calculations
	const matrix::Vector3f v_ned_est_prev = _filter_states.v_ned_est;
	const matrix::Vector3f a_ned_est_prev = _filter_states.a_ned_est;

	if (PX4_ISFINITE(v_ned_est_prev(0)) && PX4_ISFINITE(v_ned_est_prev(1)) && PX4_ISFINITE(v_ned_est_prev(2))) {
		_filter_states.x_ned_est += deltatime * v_ned_est_prev + 0.5f * a_ned_est_prev * deltatime * deltatime;
	}

	if (PX4_ISFINITE(a_ned_est_prev(0)) && PX4_ISFINITE(a_ned_est_prev(1)) && PX4_ISFINITE(a_ned_est_prev(2))) {
		_filter_states.v_ned_est += deltatime * a_ned_est_prev;
	}
}


matrix::Vector3<double> TargetEstimator::get_lat_lon_alt_est() const
{
	matrix::Vector3<double> lat_lon_alt{NAN, NAN, NAN};

	if (PX4_ISFINITE(_filter_states.x_ned_est(0)) && PX4_ISFINITE(_filter_states.x_ned_est(0))) {
		map_projection_reproject(&_reference_position, _filter_states.x_ned_est(0), _filter_states.x_ned_est(1),
					 &lat_lon_alt(0), &lat_lon_alt(1));
		lat_lon_alt(2) = -_filter_states.x_ned_est(2) + _vehicle_local_position.ref_alt;
	}

	return lat_lon_alt;
}

bool TargetEstimator::is_stale(const float timeout_duration_ms) const
{
	const bool measurements_stale = (hrt_absolute_time() - _last_follow_target_timestamp) / 1000.0f >=
					timeout_duration_ms;
	return measurements_stale;
}


void TargetEstimator::reset()
{
	_last_filter_reset_timestamp = hrt_absolute_time();  // debug only
	_last_position_fusion_timestamp = _last_velocity_fusion_timestamp = 0;
	_last_follow_target_timestamp = 0;
	_filter_states.x_ned_est = {NAN, NAN, NAN};
	_filter_states.v_ned_est = {NAN, NAN, NAN};
	_filter_states.a_ned_est = {NAN, NAN, NAN};
	_x_measurement_old = {NAN, NAN, NAN};
	_v_measurement_old = {NAN, NAN, NAN};
}
