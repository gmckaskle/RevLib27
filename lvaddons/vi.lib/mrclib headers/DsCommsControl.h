// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"
#include "mrclib/MrcString.h"

#define MRCLIB_MAX_OPMODE_NAME_LENGTH 64
#define MRCLIB_MAX_OPMODE_GROUP_LENGTH 64
#define MRCLIB_MAX_OPMODE_DESCRIPTION_LENGTH 64

struct MRC_JoystickOutputs {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t leftRumble;
    uint16_t rightRumble;
    uint16_t leftTriggerRumble;
    uint16_t rightTriggerRumble;
};

struct MRC_OpMode {
    uint64_t hash;
    uint8_t nameLength;
    char name[MRCLIB_MAX_OPMODE_NAME_LENGTH];
    uint8_t groupLength;
    char group[MRCLIB_MAX_OPMODE_GROUP_LENGTH];
    uint8_t descriptionLength;
    char description[MRCLIB_MAX_OPMODE_DESCRIPTION_LENGTH];
    int32_t textColor;
    int32_t backgroundColor;
};

extern "C" {

/**
 * Writes ANSI encoded text to the Driver Station display.
 *
 * This is part of DsCommsControl so it is scoped to the controlling robot code
 * process initialized with MRC_InitializeMainRobotApp().
 *
 * @param text the ANSI text chunk to write
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_DsCommsControl_WriteAnsi(const struct MRC_String* text);

/**
 * Tells the Driver Station communications system whether user code has been
 * started.
 *
 * @param hasUserCode whether user code has been started
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsCommsControl_SetHasUserCode(MRC_Bool hasUserCode);

/**
 * Tells the Driver Station communications system whether user code is ready and
 * the robot is ready to receive enable signals.
 *
 * @param hasUserCodeReady whether user code is ready
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_DsCommsControl_SetHasUserCodeReady(MRC_Bool hasUserCodeReady);

/**
 * Sets the joystick outputs to be sent to the Driver Station for a given stick
 * index.
 *
 * @param stickIndex the index of the joystick to set outputs for (0-based)
 * @param outputs pointer to the joystick outputs to set
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsCommsControl_SetJoystickOutputs(
    int32_t stickIndex, const struct MRC_JoystickOutputs* outputs);

/**
 * Sets the opmode trace value.
 *
 * @param trace the trace value to set
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsCommsControl_SetOpModeTrace(uint64_t trace);

/**
 * Sets the opmode options for the entire robot.
 *
 * @param opModes pointer to an array of opmodes to set
 * @param opModeCount the number of opmodes in the array
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsCommsControl_SetOpModeOptions(
    struct MRC_OpMode* opModes, uint16_t opModeCount);
}  // extern "C"
