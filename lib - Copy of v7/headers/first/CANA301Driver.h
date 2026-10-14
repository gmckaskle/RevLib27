/*
 * Copyright (c) 2026 REV Robotics
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

#pragma once

#include <stdint.h>

// #include <cstddef>

// TODO
// #include "rev/CANA301Parameters.h"
// #include "rev/REVCommon.h"
#include "rev/REVLibErrors.h"
// #include "rev/REVLibVersion.h"
// #include "rev/REVUtils.h"

extern "C" {

const uint32_t kMinA301FirmwareVersion = 0x1b000000;  // v27.0.0
const uint8_t kMinA301DebugBuild = 11;

typedef struct c_A301_Obj* c_A301_handle;

typedef enum {
    c_A301_Fault_kOther = 0,
    c_A301_Fault_kMotorType = 1,
    c_A301_Fault_kSensor = 2,
    c_A301_Fault_kCan = 3,
    c_A301_Fault_kTemperature = 4,
    c_A301_Fault_kDrv = 5,
    c_A301_Fault_kEscEeprom = 6,
    c_A301_Fault_kFirmware = 7,
    c_A301_Fault_kMotorStartup = 8
} c_A301_Fault;

typedef enum {
    c_A301_Warning_kBrownout = 0,
    c_A301_Warning_kOvercurrent = 1,
    c_A301_Warning_kEscEeprom = 2,
    c_A301_Warning_kExtEeprom = 3,
    c_A301_Warning_kSensor = 4,
    c_A301_Warning_kStall = 5,
    c_A301_Warning_kHasReset = 6,
    c_A301_Warning_kOther = 7,
    c_A301_Warning_kOvervoltage = 8,
    c_A301_Warning_kMotorLoopSpeed = 9
} c_A301_Warning;

typedef enum {
    c_A301_kStatus0 = 0,
    c_A301_kStatus1 = 1,
    c_A301_kStatus2 = 2,
    c_A301_kStatus3 = 3,
} c_A301_PeriodicFrame;

typedef enum {
    c_A301_GearboxRPM_Unknown = 0,
    c_A301_GearboxRPM_215 = 1,
    c_A301_GearboxRPM_500 = 2,
} c_A301_GearboxRPM;

typedef struct {
    float appliedOutput;
    float voltage;
    float current;
    uint8_t motorTemperature;
    uint8_t primaryHeartbeatLock;
    uint8_t gearboxRPM;  // @see c_A301_GearboxRPM
    uint64_t timestamp;
} c_A301_PeriodicStatus0;

typedef struct {
    uint8_t otherFault;
    uint8_t motorTypeFault;
    uint8_t sensorFault;
    uint8_t canFault;
    uint8_t temperatureFault;
    uint8_t drvFault;
    uint8_t escEepromFault;
    uint8_t firmwareFault;
    uint8_t motorStartupFault;

    uint8_t brownoutWarning;
    uint8_t overcurrentWarning;
    uint8_t escEepromWarning;
    uint8_t extEepromWarning;
    uint8_t sensorWarning;
    uint8_t stallWarning;
    uint8_t hasResetWarning;
    uint8_t otherWarning;
    uint8_t overvoltageWarning;
    uint8_t motorLoopSpeedWarning;

    uint8_t otherStickyFault;
    uint8_t motorTypeStickyFault;
    uint8_t sensorStickyFault;
    uint8_t canStickyFault;
    uint8_t temperatureStickyFault;
    uint8_t drvStickyFault;
    uint8_t escEepromStickyFault;
    uint8_t firmwareStickyFault;
    uint8_t motorStartupStickyFault;

    uint8_t brownoutStickyWarning;
    uint8_t overcurrentStickyWarning;
    uint8_t escEepromStickyWarning;
    uint8_t extEepromStickyWarning;
    uint8_t sensorStickyWarning;
    uint8_t stallStickyWarning;
    uint8_t hasResetStickyWarning;
    uint8_t otherStickyWarning;
    uint8_t overvoltageStickyWarning;
    uint8_t motorLoopSpeedStickyWarning;

    // uint8_t isFollower;
    uint64_t timestamp;
} c_A301_PeriodicStatus1;

typedef struct {
    float encoderVelocity;
    float relativeEncoderPosition;
    uint64_t timestamp;
} c_A301_PeriodicStatus2;

typedef struct {
    float absoluteEncoderPosition;
    uint64_t timestamp;
} c_A301_PeriodicStatus3;

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint16_t patch;
    uint8_t prerelease;
    uint32_t versionRaw;
} c_A301_FirmwareVersion;

typedef enum {
    c_A301_kIdleMode_COAST,
    c_A301_kIdleMode_BRAKE,
} c_A301_IdleMode;

c_REVLib_ErrorCode c_A301_RegisterId(int busId, int deviceId,
                                     int* actualDeviceId);
c_A301_handle c_A301_Create(int busId, int deviceId,
                            c_REVLib_ErrorCode* status);
/**
 * Logically close the A301, stopping any threads it uses, and freeing the id
 * to be used again. No methods should be called after this, but methods that
 * have already started may complete until c_A301_Destroy is called.
 */
void c_A301_Close(c_A301_handle handle);
/**
 * Free any memory used by this handle. No methods can be called on it
 * afterward.
 */
void c_A301_Destroy(c_A301_handle handle);
c_REVLib_ErrorCode c_A301_GetFirmwareVersion(c_A301_handle handle,
                                             c_A301_FirmwareVersion* fwVersion);
c_REVLib_ErrorCode c_A301_GetBusId(c_A301_handle handle, int* busId);
c_REVLib_ErrorCode c_A301_GetDeviceId(c_A301_handle handle, int* deviceId);

#if 0
void c_A301_SetPeriodicFrameTimeout(c_A301_handle handle, int timeoutMs);
void c_A301_SetCANMaxRetries(c_A301_handle handle, int numRetries);

void c_A301_SetControlFramePeriod(c_A301_handle handle, int periodMs);
int c_A301_GetControlFramePeriod(c_A301_handle handle);
#endif

c_REVLib_ErrorCode c_A301_GetPeriodicStatus0(c_A301_handle handle,
                                             c_A301_PeriodicStatus0* rawframe);
c_REVLib_ErrorCode c_A301_GetPeriodicStatus1(c_A301_handle handle,
                                             c_A301_PeriodicStatus1* rawframe);
c_REVLib_ErrorCode c_A301_GetPeriodicStatus2(c_A301_handle handle,
                                             c_A301_PeriodicStatus2* rawframe);
c_REVLib_ErrorCode c_A301_GetPeriodicStatus3(c_A301_handle handle,
                                             c_A301_PeriodicStatus3* rawframe);

c_REVLib_ErrorCode c_A301_SetStatusFramePeriod(c_A301_handle handle,
                                               c_A301_PeriodicFrame frame,
                                               uint32_t period_ms);
c_REVLib_ErrorCode c_A301_GetStatusFramePeriod(c_A301_handle handle,
                                               c_A301_PeriodicFrame frame,
                                               uint32_t* period_ms);

c_REVLib_ErrorCode c_A301_SetAbsoluteEncoderPosition(c_A301_handle handle,
                                                     float position);

c_REVLib_ErrorCode c_A301_SetRelativeEncoderPosition(c_A301_handle handle,
                                                     float position);

typedef enum {
    c_A301_kControlType_DutyCycle,
    c_A301_kControlType_Velocity,
    c_A301_kControlType_Voltage,
    c_A301_kControlType_RelativePosition,
    c_A301_kControlType_AbsolutePosition,
    c_A301_kControlType_Current,
    c_A301_kNumControlTypes
} c_A301_ControlType;

c_REVLib_ErrorCode c_A301_SetpointCommand(c_A301_handle handle, float value,
                                          c_A301_ControlType ctrl,
                                          float positionSpeed);

c_REVLib_ErrorCode c_A301_SetIdleMode(c_A301_handle handle, uint8_t idleMode);
c_REVLib_ErrorCode c_A301_GetIdleMode(c_A301_handle handle, uint8_t* idleMode);

c_REVLib_ErrorCode c_A301_SetAbsolutePositionContinuousInput(
    c_A301_handle handle, uint8_t enabled);
c_REVLib_ErrorCode c_A301_GetAbsolutePositionContinuousInput(
    c_A301_handle handle, uint8_t* enabled);

c_REVLib_ErrorCode c_A301_SetAbsoluteEncoderRangeOffset(c_A301_handle handle,
                                                        float offset);
c_REVLib_ErrorCode c_A301_GetAbsoluteEncoderRangeOffset(c_A301_handle handle,
                                                        float* offset);

c_REVLib_ErrorCode c_A301_SetInverted(c_A301_handle handle, uint8_t inverted);
c_REVLib_ErrorCode c_A301_GetInverted(c_A301_handle handle, uint8_t* inverted);
#if 0
void c_A301_SetSimAppliedOutput(c_A301_handle handle, float appliedOutput);
#endif

c_REVLib_ErrorCode c_A301_ClearFaults(c_A301_handle handle);

#if 0
c_REVLib_ErrorCode c_A301_SetCANTimeout(c_A301_handle handle, int timeoutMs);
c_REVLib_ErrorCode c_A301_Identify(c_A301_handle handle);
c_REVLib_ErrorCode c_A301_IdentifyUniqueId(int busId, uint32_t uniqueId);

c_REVLib_ErrorCode c_A301_GetLastError(c_A301_handle handle);

c_REVLib_ErrorCode c_A301_StartFollowerMode(c_A301_handle handle);
c_REVLib_ErrorCode c_A301_StopFollowerMode(c_A301_handle handle);

// Sim helpers

c_REVLib_ErrorCode c_SIM_A301_GetSimPIDOutput(c_A301_handle handle,
                                               float* value, float setpoint,
                                               float pv, float dt);

#endif
}  // extern "C"
