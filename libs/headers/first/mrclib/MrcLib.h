// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"
#include "mrclib/MrcString.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 * Initializes MrcLib with the given NetworkTables client name.
 *
 * One of the MrcLib initialization functions must be called for MrcLib to
 * communicate with the system server. Other APIs may be called first; they
 * create the local NetworkTables instance and topics without starting a
 * client. Repeated calls with the same name succeed; calls that attempt to
 * change an active client's name return MRC_STATUS_INCOMPATIBLE_STATE.
 *
 * @param clientName non-empty NetworkTables client name
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_Initialize(const struct MRC_String* clientName);

/**
 * Initializes MrcLib for the main robot application.
 *
 * The NetworkTables client is named "MainRobotApp". This function also claims
 * exclusive ownership of the Driver Station control publishers. Other APIs
 * may be called before this function, but only one process may successfully
 * claim ownership at a time.
 *
 * A failed ownership attempt may be retried after the system server becomes
 * available or the existing main robot application exits.
 *
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_InitializeMainRobotApp(void);

/**
 * Eagerly creates all MrcLib NetworkTables subscribers.
 *
 * This function does not start the NetworkTables client or claim ownership of
 * the Driver Station control publishers. It may be called before or after an
 * MrcLib initialization function and may be called repeatedly.
 *
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_InitializeAllSubscribers(void);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus
