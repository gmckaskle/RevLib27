// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"
#include "mrclib/MrcString.h"

extern "C" {
/**
 * Initializes the system NT server. This is a null operation on Systemcore
 * itself
 *
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SimSystemServer_Initialize(void);

/**
 * Sets the team number on the simulated system server.
 *
 * @param team team number
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_SimSystemServer_SetTeam(const struct MRC_String* team);

/**
 * Sets the battery voltage level on the simulated system server.
 *
 * @param level battery voltage level
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_SimSystemServer_SetBatteryVoltage(double level);
}  // extern "C"
