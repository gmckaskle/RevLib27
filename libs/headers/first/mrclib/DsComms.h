// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "mrclib/MrcLibTypes.h"

#define MRCLIB_MAX_GAMEDATA_LENGTH 64
#define MRCLIB_MAX_EVENTNAME_LENGTH 64
#define MRCLIB_MAX_JOYSTICKS 6
#define MRCLIB_MAX_POVS 8
#define MRCLIB_MAX_AXES 12
#define MRCLIB_MAX_TOUCHPADS 2
#define MRCLIB_MAX_TOUCHPAD_FINGERS 2
#define MRCLIB_MAX_JOYSTICK_NAME_LENGTH 255

struct MRC_ControlData {
    uint32_t controlFlags;
    uint16_t matchTime;
    uint64_t currentOpMode;
    uint8_t gameDataLength;
    char gameData[MRCLIB_MAX_GAMEDATA_LENGTH];
};

struct MRC_JoystickTouchpadFinger {
    MRC_Bool down;
    uint16_t x;
    uint16_t y;
};

struct MRC_JoystickTouchpad {
    uint8_t count;
    struct MRC_JoystickTouchpadFinger fingers[MRCLIB_MAX_TOUCHPAD_FINGERS];
};

struct MRC_Joystick {
    uint8_t availablePovs;
    uint8_t povs[MRCLIB_MAX_POVS];
    uint16_t availableAxes;
    int16_t axes[MRCLIB_MAX_AXES];
    uint64_t availableButtons;
    uint64_t buttons;
    uint8_t touchpadCount;
    struct MRC_JoystickTouchpad touchpads[MRCLIB_MAX_TOUCHPADS];
};

struct MRC_Joysticks {
    uint8_t count;
    struct MRC_Joystick joysticks[MRCLIB_MAX_JOYSTICKS];
};

struct MRC_MatchInfo {
    uint16_t matchNumber;
    uint8_t replayNumber;
    uint8_t matchType;
    uint8_t eventNameLength;
    char eventName[MRCLIB_MAX_EVENTNAME_LENGTH];
};

struct MRC_JoystickDescriptor {
    MRC_Bool isGamepad;
    uint8_t gamepadType;
    uint8_t supportedOutputs;
    uint8_t nameLength;
    char name[MRCLIB_MAX_JOYSTICK_NAME_LENGTH];
};

struct MRC_JoystickDescriptors {
    uint8_t count;
    struct MRC_JoystickDescriptor descriptors[MRCLIB_MAX_JOYSTICKS];
};

extern "C" {

/**
 * Gets the latest control data from the Driver Station, not including joystick
 * data.
 *
 * @param controlData pointer to store the control data buffer
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_DsComms_GetControlData(struct MRC_ControlData* data);

/**
 * Gets the latest control data from the Driver Station, including joystick
 * data.
 *
 * @param controlData pointer to store the control data buffer
 * @param joystickData pointer to store the joystick data buffer
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsComms_GetControlDataWithJoysticks(
    struct MRC_ControlData* data, struct MRC_Joysticks* joystickData);

/**
 * Gets the latest match info data from the Driver Station.
 *
 * Match info is only updated when the Driver Station receives new match info
 * from the Field Management System, so this data may be stale if not in a match
 * environment.
 *
 * @param matchInfo pointer to store the match info buffer
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_DsComms_GetMatchInfo(struct MRC_MatchInfo* matchInfo);

/**
 * Gets the latest joystick descriptors from the Driver Station.
 *
 * @param joystickDescriptors pointer to store the joystick descriptors buffer
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsComms_GetJoystickDescriptors(
    struct MRC_JoystickDescriptors* joystickDescriptors);

/**
 * Waits for the system server to be ready.
 *
 * @param waitTimeMs the maximum time to wait in milliseconds
 * @return true if the system server is ready, false if the wait timed out
 */
MRC_Bool MRC_CALLCONV MRC_DsComms_WaitForSystemServer(uint32_t waitTimeMs);

/**
 * Gets whether the system time is valid.
 *
 * @param valid pointer to store whether the system time is valid (output)
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV MRC_DsComms_GetSystemTimeValid(MRC_Bool* valid);

/**
 * Sets a callback function to be called when new data is received from the
 * Driver Station.
 *
 * Set to null to remove the callback. However, note that this is an atomic
 * operation and the callback could be pending while this function is called.
 *
 * @param callback function pointer to the callback function
 * @return status of the operation
 */
MRC_Status MRC_CALLCONV
MRC_DsComms_SetNewDataCallback(void(MRC_CALLCONV* callback)(void));
}  // extern "C"
