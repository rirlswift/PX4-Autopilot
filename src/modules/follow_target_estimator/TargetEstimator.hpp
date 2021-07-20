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
 * @file TargetEstimator.hpp
 *
 * @author Alessandro Simovic <potaito-dev@protonmail.com>
 */

#pragma once

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/follow_target_estimator.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/PublicationMulti.hpp>

#include <drivers/drv_hrt.h>
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#define GPS_MESSAGE_STALE_TIMEOUT_MS 3000  	// Duration after which the connection to the target is considered lost
#define MINIMUM_TIME_BETWEEN_POS_FUSIONS_MS 500
#define MINIMUM_TIME_BETWEEN_VEL_FUSIONS_MS 100
#define ACCELERATION_SATURATION 20.0f 		// 2*g
#define MINIMUM_SPEED_FOR_TARGET_MOVING 0.1f 	// speed threshold above which the target is considered to be moving

using namespace time_literals;

struct filter_gains_s {
	// Position fusion gains
	float G_p, H_p, K_p;

	// Velocity fusion gains
	float G_v, H_v;

	// Estimator position / velocity update gains
	float responsiveness;
};

struct filter_states_s {
	matrix::Vector3f x_ned_est{};	// target's position in NED frame
	matrix::Vector3f v_ned_est{};	// target's velocity in NED frame
	matrix::Vector3f a_ned_est{};	// target's acceleration in NED frame

	/**
	 * Check if all state are finite
	 *
	 * @return true if all state are finite, or false as soon as any state is NAN
	 */
	bool is_finite()
	{
		return  PX4_ISFINITE(x_ned_est(0)) && PX4_ISFINITE(x_ned_est(1)) && PX4_ISFINITE(x_ned_est(2)) &&
			PX4_ISFINITE(v_ned_est(0)) && PX4_ISFINITE(v_ned_est(1)) && PX4_ISFINITE(v_ned_est(2)) &&
			PX4_ISFINITE(a_ned_est(0)) && PX4_ISFINITE(a_ned_est(1)) && PX4_ISFINITE(a_ned_est(2));
	};

	/**
	 * Limits the acceleration state to some sane value to prevent unrealistic
	 * spikes in the acceleration, which could cause severely unexpected behaviour in the drone
	 * that is tracking the target
	 *
	 * @param saturation [m/s^2] limit to use for acceleration saturation
	 */
	void saturate_acceleration(float saturation)
	{
		if (a_ned_est.norm() > saturation) {
			a_ned_est = a_ned_est.unit_or_zero() * saturation;
		}
	}
};

class TargetEstimator : public ModuleBase<TargetEstimator>, ModuleParams
{
public:
	TargetEstimator();

	/**
	 * Process new measurement data and iterate filter to current time
	 */
	void update();

protected:
	/**
	 * Check for parameter changes and update them if needed.
	 *
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	/**
	 * Recompute filter gains
	 *
	 * @param filter_gains filter gains that will be updated based on current PX4 parameters
	 */
	void update_filter_gains(filter_gains_s *filter_gains) const;

	/**
	 * Perform filter update with new follow_target data
	 *
	 * @param follow_target GPS data last received from target
	 */
	void measurement_update(follow_target_s follow_target);

	/**
	 * Perform position measurement update (position fusion) with new data
	 *
	 * @param deltatime time [s] since last position measurement update
	 * @param x_measured last known target location in NED frame
	 * @param filter_state_prev read-only filter_state from previous update iteration
	 * @param filter_state current filter state that will be updated by this measurement fusion
	 */
	void position_meas_update(float deltatime, matrix::Vector3f x_measured, filter_states_s const *const filter_states_prev,
				  filter_states_s *const filter_states) const;

	/**
	 * Perform velocity measurement update (velocity fusion) with new data
	 *
	 * @param deltatime time [s] since last velocity measurement update
	 * @param x_measured last known target location in NED frame
	 * @param filter_state_prev read-only filter_state from previous update iteration
	 * @param filter_state current filter state that will be updated by this measurement fusion
	 */
	void velocity_meas_update(float deltatime, matrix::Vector3f v_measured, filter_states_s const *const filter_states_prev,
				  filter_states_s *const filter_states) const;

	/**
	 * Perform prediction step based on simple position-velocity-acceleration model of a point mass
	 * Can be called at a much higher frequency than measurement data is being received.
	 *
	 * @param deltatime time [s] since the last prediction or measurement update
	 */
	void prediction_update(float deltatime);

	/**
	 * Get current LAT/LON/ALT estimate of target
	 *
	 * @return Current position estimate of target as latitude / longitude / altitude vector
	 */
	matrix::Vector3<double> get_lat_lon_alt_est() const;

	/**
	 * Check if last received data from target is too old
	 *
	 * @param timeout_duration_ms timeout in [ms] to use for this check
	 */
	bool is_stale(const float timeout_duration_ms) const;

	/**
	 * Reset all filter states causing it to completely forget the old filter state
	 */
	void reset();

	filter_gains_s _filter_gains;
	filter_states_s _filter_states;

	map_projection_reference_s _reference_position{};
	vehicle_local_position_s _vehicle_local_position{};

	uint64_t _last_iteration_timestamp = 0;
	uint64_t _last_filter_reset_timestamp = 0;
	uint64_t _last_position_fusion_timestamp = 0;
	uint64_t _last_velocity_fusion_timestamp = 0;
	uint64_t _last_follow_target_timestamp = 0;

	// Pos/vel from previous measurement update. Required for filtering duplicate messages
	matrix::Vector3f _x_measurement_old{};
	matrix::Vector3f _v_measurement_old{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_FT_RS>) _param_nav_ft_rs
	)

	// Subscriptions
	uORB::Subscription _follow_target_sub{ORB_ID(follow_target)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::PublicationMulti<follow_target_estimator_s> _follow_target_estimator_pub{ORB_ID(follow_target_estimator)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
