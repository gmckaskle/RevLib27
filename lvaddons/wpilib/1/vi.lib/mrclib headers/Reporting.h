// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include "mrclib/MrcLibTypes.h"
#include "mrclib/MrcString.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 * Reports usage of a resource of interest. Repeated calls for the same
 * resource replace the previous report.
 *
 * @param resource the used resource name
 * @param data arbitrary associated data string
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Reporting_ReportUsage(
    const struct MRC_String* resource, const struct MRC_String* data);

/**
 * Publishes a version for a device on a bus.
 *
 * @param busId bus containing the device
 * @param deviceId device identifier
 * @param name version name
 * @param version version string
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Reporting_PublishCanVersion(
    uint8_t busId, uint32_t deviceId, const struct MRC_String* name,
    const struct MRC_String* version);

/**
 * Publishes a version without a bus or device identifier.
 *
 * The version is published on bus 31 for device 0.
 *
 * @param name version name
 * @param version version string
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Reporting_PublishVersion(
    const struct MRC_String* name, const struct MRC_String* version);

/**
 * Publishes the WPILib version.
 *
 * @param version WPILib version string
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_Reporting_PublishWpilibVersion(const struct MRC_String* version);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus
