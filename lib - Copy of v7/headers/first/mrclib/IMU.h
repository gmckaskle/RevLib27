// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"

struct MRC_Quaternion {
    int64_t timestamp;
    double w;
    double x;
    double y;
    double z;
};

struct MRC_Acceleration3d {
    int64_t timestamp;
    double x;
    double y;
    double z;
};

struct MRC_GyroRate3d {
    int64_t timestamp;
    double x;
    double y;
    double z;
};

struct MRC_EulerAngles3d {
    int64_t timestamp;
    double x;
    double y;
    double z;
};

extern "C" {
/**
 * Gets the current acceleration of the robot in meters per second squared.
 *
 * @param acceleration pointer to store the acceleration
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_IMU_GetAcceleration(struct MRC_Acceleration3d* acceleration);

/**
 * Gets the current gyro rate of the robot in radians per second.
 *
 * @param gyroRate pointer to store the gyro rate
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_IMU_GetGyroRate(struct MRC_GyroRate3d* gyroRate);

/**
 * Gets the current Euler angles of the robot in radians.
 *
 * @param eulerAngles pointer to store the Euler angles
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_IMU_GetEulerAnglesFlat(struct MRC_EulerAngles3d* eulerAngles);

/**
 * Gets the current Euler angles of the robot in landscape orientation in
 * radians.
 *
 * @param eulerAngles pointer to store the Euler angles
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_IMU_GetEulerAnglesLandscape(struct MRC_EulerAngles3d* eulerAngles);

/**
 * Gets the current Euler angles of the robot in portrait orientation in
 * radians.
 *
 * @param eulerAngles pointer to store the Euler angles
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_IMU_GetEulerAnglesPortrait(struct MRC_EulerAngles3d* eulerAngles);

/**
 * Gets the current orientation of the robot as a quaternion.
 *
 * @param quaternion pointer to store the quaternion
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_IMU_GetQuaternion(struct MRC_Quaternion* quaternion);

/**
 * Gets the current yaw of the robot in flat orientation in radians.
 *
 * @param yaw pointer to store the yaw
 * @param timestamp pointer to store the timestamp of the reading
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_IMU_GetYawFlat(double* yaw, int64_t* timestamp);

/**
 * Gets the current yaw of the robot in landscape orientation in radians.
 *
 * @param yaw pointer to store the yaw
 * @param timestamp pointer to store the timestamp of the reading
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_IMU_GetYawLandscape(double* yaw,
                                                int64_t* timestamp);

/**
 * Gets the current yaw of the robot in portrait orientation in radians.
 *
 * @param yaw pointer to store the yaw
 * @param timestamp pointer to store the timestamp of the reading
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_IMU_GetYawPortrait(double* yaw, int64_t* timestamp);

/**
 * Sets the yaw trim in radians per revolution.
 *
 * The publisher is created on the first call.
 *
 * @param yawTrim yaw trim in radians per revolution
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_IMU_SetYawTrim(double yawTrim);
}  // extern "C"
