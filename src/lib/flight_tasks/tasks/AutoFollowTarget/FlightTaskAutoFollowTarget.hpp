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
 * @file FlightTaskAutoFollowTarget.hpp
 *
 * Flight task for autonomous, gps driven follow-me mode.
 *
 * @author Alessandro Simovic <potaito-dev@protonmail.com>
 */

#pragma once

#include "FlightTaskAuto.hpp"

#include <parameters/param.h>
#include <mathlib/mathlib.h>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/follow_target_status.h>
#include <uORB/topics/follow_target_estimator.h>
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>

#define MINIMUM_SPEED_FOR_HEADING_CHANGE 0.1f 	// Speed above which the target heading can change. Used to prevent unpredictable jitter at low speeds.
#define MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL 1.0f  // Minimum distance between drone and target for the drone to do any yaw control.
#define MINIMUM_SAFETY_ALTITUDE 1.0f  // Minimum safety altitude above home (or bottom distance sensor) underneath which the flight task will stop moving horizontally
#define ALT_ACCEPTANCE_THRESHOLD 3.0f // [m] max vertical deviation from position setpoint, above which no horizontal control is done
#define EMERGENCY_ASCENT_SPEED 0.2f

// Filter gains for the different kinds of septoints
// that should ensure a cinematic experience
#define POSITION_FILTER_ALPHA 1.5f		// Lowpass applied to the estimated position of the target before using it as control input
#define FOLLOW_ANGLE_FILTER_ALPHA 3.0f		// Lowpass applied to the follow-me angle setting, to ensure smooth and circular transitions between settings
#define DIRECTION_FILTER_ALPHA 3.0f		// Lowpass applied to the actual NED direction how the drone is facing the target regarless of the setting. Used for dynamic tracking angles when the target makes a turn
#define VELOCITY_FF_FILTER_ALPHA 1.0f		// Lowpass applied for ramping up / down velocity feedforward term

// Angles [deg] for the different follow-me perspectives
#define FOLLOW_PERSPECTIVE_BEHIND_ANGLE_DEG 180.0
#define FOLLOW_PERSPECTIVE_FRONT_ANGLE_DEG 0.0
#define FOLLOW_PERSPECTIVE_FRONT_RIGHT_ANGLE_DEG 45.0
#define FOLLOW_PERSPECTIVE_FRONT_LEFT_ANGLE_DEG 315.0
#define FOLLOW_PERSPECTIVE_MID_RIGHT_ANGLE_DEG 90.0
#define FOLLOW_PERSPECTIVE_MID_LEFT_ANGLE_DEG 270.0
#define FOLLOW_PERSPECTIVE_BEHIND_RIGHT_ANGLE_DEG 135.0
#define FOLLOW_PERSPECTIVE_BEHIND_LEFT_ANGLE_DEG 225.0


class FlightTaskAutoFollowTarget : public FlightTask
{
public:
	FlightTaskAutoFollowTarget() = default;
	virtual ~FlightTaskAutoFollowTarget() = default;

	bool activate(vehicle_local_position_setpoint_s last_setpoint) override;
	bool update() override;

protected:
	enum {
		FOLLOW_PERSPECTIVE_NONE,
		FOLLOW_PERSPECTIVE_BEHIND,
		FOLLOW_PERSPECTIVE_FRONT,
		FOLLOW_PERSPECTIVE_FRONT_RIGHT,
		FOLLOW_PERSPECTIVE_FRONT_LEFT,
		FOLLOW_PERSPECTIVE_MID_RIGHT,
		FOLLOW_PERSPECTIVE_MID_LEFT,
		FOLLOW_PERSPECTIVE_BEHIND_RIGHT,
		FOLLOW_PERSPECTIVE_BEHIND_LEFT,
		FOLLOW_PERSPECTIVE_MIDDLE_FOLLOW,
		FOLLOW_PERSPECTIVE_INVALID  // Leave this as last!
	};

	enum {
		FOLLOW_ALTITUDE_MODE_CONSTANT,
		FOLLOW_ALTITUDE_MODE_TRACK_TARGET
	};

	/**
	 * Get the current follow-me perspective setting from PX4 parameters
	 *
	 * @param param_nav_ft_fs value of the parameter NAV_FT_FS
	 * @return Angle [deg] from which the drone should view the target while following it, with zero degrees indicating the target's 12 o'clock
	 */
	float update_follow_me_angle_setting(int param_nav_ft_fs) const;

	/**
	 * Predict target's position through forward integration of its currently estimated position, velocity and acceleration.
	 *
	 * @param deltatime [s] prediction horizon
	 * @return Future prediction of target position
	 */
	matrix::Vector3f predict_future_x_ned_est(float deltatime, matrix::Vector3f x_ned_est, matrix::Vector3f v_ned_est,
			matrix::Vector3f a_ned_est) const;

	// Follow angle is defined with 0 degrees following from front, and then clockwise rotation
	float _follow_angle_deg = 0.0f;
	AlphaFilter<float> _follow_angle_filtered;

	// Estimator for target position and velocity
	follow_target_estimator_s _follow_target_estimator;
	matrix::Vector2f _target_velocity_unit_vector;

	// Lowpass filters for smoothing out the drone setpoints / follow angles
	AlphaFilter<matrix::Vector3f>
	_target_position_filtered;  // Smoothen target position because it's used for setpoint generation
	AlphaFilter<matrix::Vector2f>
	_offset_vector_filtered;  // Smoothen the offset vector to have more dynamic shots when target changes direction
	AlphaFilter<float> _velocity_ff_scale;  // values 0-1, for avoiding big steps in velocity feedforward

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_MIN_FT_HT>) _param_nav_min_ft_ht,
		(ParamFloat<px4::params::NAV_FT_DST>) _param_nav_ft_dst,
		(ParamInt<px4::params::NAV_FT_FS>) _param_nav_ft_fs,
		(ParamInt<px4::params::NAV_FT_ALT_M>) _param_nav_ft_alt_m
	)

	uORB::Subscription _follow_target_estimator_sub{ORB_ID(follow_target_estimator)};

	uORB::PublicationMulti<follow_target_status_s> _follow_target_status_pub{ORB_ID(follow_target_status)};
};
