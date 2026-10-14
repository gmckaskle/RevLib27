// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcString.h"
#include "mrclib/MrcLibTypes.h"

/** Minimum allowed brownout voltage, in millivolts. */
#define MRC_SYSTEMCORE_BROWNOUT_VOLTAGE_MIN_MV 5000
/** Maximum allowed brownout voltage, in millivolts. */
#define MRC_SYSTEMCORE_BROWNOUT_VOLTAGE_MAX_MV 8000
/** Maximum allowed brownout recovery voltage, in millivolts. */
#define MRC_SYSTEMCORE_BROWNOUT_RECOVERY_VOLTAGE_MAX_MV 8500
/** Minimum recovery voltage above brownout voltage, in millivolts. */
#define MRC_SYSTEMCORE_BROWNOUT_RECOVERY_VOLTAGE_MIN_DELTA_MV 500
/** Number of CAN buses represented by the cross detection bitmask. */
#define MRC_SYSTEMCORE_CAN_BUS_COUNT 5
/** Valid bits in a CAN bus cross detection mask. */
#define MRC_SYSTEMCORE_CAN_BUS_CROSS_MASK 0x1F

extern "C" {

/**
 * Gets the current battery voltage of the robot.
 *
 * @param voltage pointer to store the battery voltage in volts
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Systemcore_GetBatteryVoltage(float* voltage);

/**
 * Gets the team number of the robot.
 *
 * @param teamNumber pointer to store the team number string
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Systemcore_GetTeamNumber(MRC_String* teamNumber);

/**
 * Sets the voltages where the robot enters and recovers from brownout.
 *
 * The brownout voltage must be between
 * MRC_SYSTEMCORE_BROWNOUT_VOLTAGE_MIN_MV and
 * MRC_SYSTEMCORE_BROWNOUT_VOLTAGE_MAX_MV, inclusive. The recovery voltage
 * must be no greater than MRC_SYSTEMCORE_BROWNOUT_RECOVERY_VOLTAGE_MAX_MV and
 * at least MRC_SYSTEMCORE_BROWNOUT_RECOVERY_VOLTAGE_MIN_DELTA_MV above the
 * brownout voltage. The publishers are created on the first successful call.
 *
 * @param brownoutMillivolts brownout voltage in millivolts
 * @param recoveryMillivolts recovery voltage in millivolts
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Systemcore_SetBrownoutVoltages(
    uint16_t brownoutMillivolts, uint16_t recoveryMillivolts);

/**
 * Gets whether the robot is currently browned out.
 *
 * @param brownedOut pointer to store whether the robot is browned out
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Systemcore_GetBrownedOut(MRC_Bool* brownedOut);

/**
 * @param active pointer to store whether the watchdog is active
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Systemcore_GetWatchdogActive(MRC_Bool* active);

/**
 * Gets the current utilization of the CAN buses.
 *
 * numValues is an in/out parameter. On entry, it is the capacity of the
 * utilizations array. On return, it is the number of values written. At most
 * MRC_SYSTEMCORE_CAN_BUS_COUNT values are written. If no valid utilization
 * data has been published, the returned utilizations are zero.
 *
 * @param utilizations array to store the CAN bus utilization percentages
 * @param numValues input array capacity and output number of values written
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_Systemcore_GetCanBusUtilizations(float* utilizations, size_t* numValues);

/**
 * Sets the CAN bus cross detection mask for a given bus.
 *
 * Bus B occupies bits 5B through 5B+4 in the integer published at
 * "/sys/canbuscross". Bit N of crossMasks indicates whether bus B is crossed
 * with bus N. The publisher is created on the first successful call.
 *
 * @param bus CAN bus number in the range 0-4
 * @param crossMasks five-bit cross detection mask for the CAN bus
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_Systemcore_SetCanBusCrossDetection(uint8_t bus, uint8_t crossMasks);
}  // extern "C"
