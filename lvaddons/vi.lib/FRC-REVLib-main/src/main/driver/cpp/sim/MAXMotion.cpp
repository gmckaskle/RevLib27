/*
 * Copyright (c) 2025 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * maxmotion.c
 *
 *  Created on: Jan 24, 2019
 *      Author: guitar24t
 *  Renamed on: May 16, 2025
 *      Author: Rylan
 */

#include "rev/sim/MAXMotion.h"

#include <cmath>

void maxmotion_profile_generate(maxmotion_profile_t* profile,
                                const maxmotion_constants_t motion_params,
                                maxmotion_state_t start,
                                maxmotion_state_t end) {
    profile->current_time = 0;
    profile->is_valid = false;
    profile->motion_params = motion_params;

    // Check if the motion parameters are valid
    if (motion_params.kMAXMotionMaxVelocity <= 0 ||
        motion_params.kMAXMotionMaxAccel <= 0) {
        return;
    }

    if (motion_params.kMAXMotionPositionMode == TRAPEZOIDAL) {
        // Trapezoidal profile generation

        float position_conversion_factor =
            motion_params.positionConversionFactor;
        float velocity_conversion_factor =
            motion_params.velocityConversionFactor;

        const float RPM_TO_RPS = 1.0f / 60.0f;
        const float SECONDS_TO_CYCLES = 50.0f;  // firmware runs at 100Hz, sim
                                                // runs at 50Hz

        // Invalidate s-curve points
        profile->points[1] = {};
        profile->points[2] = {};
        profile->points[5] = {};
        profile->points[6] = {};

        // direction from start pos to end pos
        int motion_direction = copysignf(1.0f, end.position - start.position);

        float max_velocity_rpm =
            motion_params.kMAXMotionMaxVelocity / velocity_conversion_factor;
        float max_accel_rpmps =
            motion_params.kMAXMotionMaxAccel / velocity_conversion_factor;

        float start_velocity_rpm = start.velocity / velocity_conversion_factor;

        float start_position_r = start.position / position_conversion_factor;
        float end_position_r = end.position / position_conversion_factor;

        int acceleration_direction = copysignf(
            1.0f, max_velocity_rpm * motion_direction - start_velocity_rpm);
        int deceleration_direction = -motion_direction;

        if (start_position_r == end_position_r) {
            motion_direction = -copysignf(1.0f, start.velocity);

            acceleration_direction = motion_direction;
            deceleration_direction = -motion_direction;
        }

        float start_rps = start_velocity_rpm * RPM_TO_RPS;
        float accel_rps2 = max_accel_rpmps * RPM_TO_RPS;

        // override starting acceleration and jerk
        start.acceleration =
            motion_params.kMAXMotionMaxAccel * acceleration_direction;
        start.jerk = 0;  // trapezoidal profile has no jerk

        // scrub end data
        end.velocity = 0;  // no support for nonzero end velocity :( it makes
                           // the rest of the math really intense
        end.acceleration = 0;
        end.jerk = 0;

        // Initialize starting point
        profile->points[0] = {start, 0, true};

        float max_achieved_velocity_rpm =
            sqrtf((2.0 * (end_position_r - start_position_r) +
                   ((start_rps * start_rps) /
                    (accel_rps2 * acceleration_direction))) /
                  ((1 / (accel_rps2 * acceleration_direction)) -
                   (1 / (accel_rps2 * deceleration_direction)))) /
            RPM_TO_RPS;

        if (std::isnan(max_achieved_velocity_rpm)) {
            motion_direction *= -1.0f;

            acceleration_direction = copysignf(
                1.0f, max_velocity_rpm * motion_direction - start_velocity_rpm);
            deceleration_direction = -motion_direction;

            max_achieved_velocity_rpm =
                sqrtf((2.0 * (end_position_r - start_position_r) +
                       ((start_rps * start_rps) /
                        (accel_rps2 * acceleration_direction))) /
                      ((1 / (accel_rps2 * acceleration_direction)) -
                       (1 / (accel_rps2 * deceleration_direction)))) *
                60.0f;
        }

        max_achieved_velocity_rpm =
            fminf(max_velocity_rpm, max_achieved_velocity_rpm);

        float max_achieved_velocity_rps =
            max_achieved_velocity_rpm * RPM_TO_RPS;

        float acceleration_time_s =
            fabsf(max_achieved_velocity_rpm * motion_direction -
                  start_velocity_rpm) /
            max_accel_rpmps;
        float deceleration_time_s = max_achieved_velocity_rpm / max_accel_rpmps;

        // Distance during accel/decel
        float acceleration_distance_r = start_rps * acceleration_time_s +
                                        0.5f * acceleration_direction *
                                            accel_rps2 * acceleration_time_s *
                                            acceleration_time_s;
        float deceleration_distance_r =
            max_achieved_velocity_rps * motion_direction * deceleration_time_s +
            0.5f * deceleration_direction * accel_rps2 * deceleration_time_s *
                deceleration_time_s;

        int accel_cycles =
            static_cast<int>(acceleration_time_s * SECONDS_TO_CYCLES);
        float const_velocity_distance_r =
            fabsf((end_position_r - deceleration_distance_r) -
                  (start_position_r + acceleration_distance_r));
        int const_velocity_time_cycles =
            static_cast<int>(const_velocity_distance_r /
                             max_achieved_velocity_rps * SECONDS_TO_CYCLES);

        profile->points[3] = {
            {(start_position_r + acceleration_distance_r) *
                 position_conversion_factor,
             (start_velocity_rpm + acceleration_direction * accel_rps2 *
                                       acceleration_time_s * 60.0f) *
                 velocity_conversion_factor,
             0.0f, 0.0f},
            accel_cycles,
            true};

        profile->points[4] = {{(start_position_r + acceleration_distance_r +
                                const_velocity_distance_r * motion_direction) *
                                   position_conversion_factor,
                               max_achieved_velocity_rpm * motion_direction *
                                   velocity_conversion_factor,
                               max_accel_rpmps * deceleration_direction *
                                   velocity_conversion_factor,
                               0.0f},
                              accel_cycles + const_velocity_time_cycles,
                              true};

        int decel_cycles =
            static_cast<int>(deceleration_time_s * SECONDS_TO_CYCLES);

        profile->points[7] = {
            end, accel_cycles + const_velocity_time_cycles + decel_cycles,
            true};
    } else if (motion_params.kMAXMotionPositionMode == SCURVE) {
        // S-curve profile generation
        // TODO
    } else {
        // Invalid acceleration mode
        return;
    }

    profile->is_valid = true;
}

bool maxmotion_profile_check(maxmotion_profile_t* profile,
                             const maxmotion_constants_t motion_params,
                             maxmotion_state_t current_state,
                             maxmotion_state_t end_state) {
    // check if the profile should be valid
    if (!profile->is_valid) return false;

    // time invalid
    if (profile->current_time < 0) {
        profile->is_valid = false;
        return false;
    }

    // no valid points
    bool no_valid = true;
    for (int i = 0; i < 8; i++) {
        if (profile->points[i].is_valid) {
            no_valid = false;
        }
    }
    if (no_valid) {
        profile->is_valid = false;
        return false;
    }

    // [0] is invalid
    if (!profile->points[0].is_valid) {
        profile->is_valid = false;
        return false;
    }

    // setpoint has changed
    if (profile->points[7].state.position != end_state.position) {
        profile->is_valid = false;
        return false;
    }

    // motion params has changed
    if (motion_params.kAllowedClosedLoopError !=
            profile->motion_params.kAllowedClosedLoopError ||
        motion_params.kMAXMotionMaxVelocity !=
            profile->motion_params.kMAXMotionMaxVelocity ||
        motion_params.kMAXMotionMaxAccel !=
            profile->motion_params.kMAXMotionMaxAccel ||
        motion_params.kMAXMotionMaxJerk !=
            profile->motion_params.kMAXMotionMaxJerk) {
        profile->is_valid = false;
        return false;
    }

    // more than AllowedClosedLoopError of error
    float intermediate_target =
        maxmotion_profile_get_point(profile, profile->current_time).position;
    float error = fabsf(current_state.position - intermediate_target);
    if (error > motion_params.kAllowedClosedLoopError) {
        profile->is_valid = false;
        return false;
    }

    return true;
}

maxmotion_state_t maxmotion_profile_get_point(maxmotion_profile_t* profile,
                                              int time) {
    maxmotion_profile_point_t last_point = profile->points[0];
    for (int i = 0; i < 8; i++) {
        if (profile->points[i].is_valid) {
            if (profile->points[i].time <= time &&
                profile->points[i].time >= 0) {
                last_point = profile->points[i];
            } else {
                break;
            }
        }
    }

    const float RPM_TO_RPS = 1.0f / 60.0f;
    const float SECONDS_TO_CYCLES = 50.0f;  // firmware runs at 100Hz, sim
                                            // runs at 50Hz

    int time_from_last_point_cycles = time - last_point.time;

    float time_from_last_point_seconds =
        time_from_last_point_cycles / SECONDS_TO_CYCLES;

    float last_point_velocity_rps =
        last_point.state.velocity /
        profile->motion_params.velocityConversionFactor * RPM_TO_RPS;
    float last_point_acceleration_rps2 =
        last_point.state.acceleration /
        profile->motion_params.velocityConversionFactor * RPM_TO_RPS;

    // adjusted for velocity, acceleration, and jerk
    float acceleration_rps2 =
        last_point_acceleration_rps2 +
        last_point.state.jerk /
            profile->motion_params.velocityConversionFactor *
            time_from_last_point_seconds;
    float velocity_rps = last_point_velocity_rps +
                         acceleration_rps2 * time_from_last_point_seconds;
    float position_r = last_point.state.position /
                           profile->motion_params.positionConversionFactor +
                       last_point_velocity_rps * time_from_last_point_seconds +
                       0.5f * acceleration_rps2 * time_from_last_point_seconds *
                           time_from_last_point_seconds;

    maxmotion_state_t new_point = {
        position_r * profile->motion_params.positionConversionFactor,
        velocity_rps * 60.0f * profile->motion_params.velocityConversionFactor,
        acceleration_rps2 * 60.0f *
            profile->motion_params.velocityConversionFactor,
        last_point.state.jerk *
            profile->motion_params.velocityConversionFactor};
    return new_point;
}

///////////////////////////////////////////////////////////////////////////////////

maxmotion_velocity_state_t smartervelocity_calculate_output_velocity(
    maxmotion_constants_t* motion_params, float setpoint, float prev_vel,
    float dt) {
    float convertedMaxAccel = motion_params->kMAXMotionMaxAccel;
    float convertedClosedLoopErr = motion_params->kAllowedClosedLoopError;

    float diffErr = setpoint - prev_vel;

    if (convertedMaxAccel == 0) return {};

    if (fabsf(diffErr) <= convertedClosedLoopErr) {
        return {setpoint, 0};
    } else {
        float speedNew =
            prev_vel + (convertedMaxAccel * dt * copysignf(1.0f, diffErr));
        if (diffErr > 0) {
            return {speedNew > setpoint ? setpoint : speedNew,
                    convertedMaxAccel};
        } else {
            return {speedNew < setpoint ? setpoint : speedNew,
                    -convertedMaxAccel};
        }
    }
}
