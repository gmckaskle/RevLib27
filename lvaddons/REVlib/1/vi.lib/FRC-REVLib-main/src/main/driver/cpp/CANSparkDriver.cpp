/*
 * Copyright (c) 2018-2026 REV Robotics
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

#include "rev/CANSparkDriver.h"

#include <stdint.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>

// TODO: (dave) get rid of:
#include <rev/driver/REVLibDriver.h>

#include "rev/CANDriverPrivate.h"
#include "rev/CANSparkFrames.h"
#include "rev/REVLibDaemon.h"
#include "rev/REVLibErrors.h"
#include "rev/REVUtils.h"
#include "rev/SparkFrameManager.h"
#include "rev/sim/CANSpark.h"

namespace {

constexpr int kMaxPacketLength{8};

constexpr uint8_t kMotorInterfaceDockId{1};

bool isVersionTooNew(const c_Spark_FirmwareVersion& fwVersion) {
    return fwVersion.major > 28;
}

enum c_Spark_ParameterStatus {
    c_Spark_kParamOK = 0,
    c_Spark_kInvalidID = 1,
    c_Spark_kMismatchType = 2,
    c_Spark_kAccessMode = 3,
    c_Spark_kInvalid = 4,
    c_Spark_kNotImplementedDeprecated = 5
};

constexpr uint32_t kInvertOnControllerVersion = 0x1030002;

static_assert(sizeof(float) == sizeof(uint32_t), "float isn't 32 bits wide");

CAN_ExistingDeviceIds s_Spark_ExistingDeviceIds;

constexpr std::array<uint8_t, 0> zeroLengthDataPacket{};

bool areSideEffectsDisabled = false;

auto modelToDeviceType = [](c_Spark_SparkModel unconfirmedModel) {
    switch (unconfirmedModel) {
        case c_Spark_SparkFlex:
            return SparkFlex;
        case c_Spark_SparkMax:
            return SparkMax;
        default:
            return UnknownREVDevice;
    }
};

struct status_period_info_t {
    c_Spark_ConfigParameter periodId;
    int* handle_statusPeriod_ms;
};

}  // namespace

extern "C" {
void disableStartupSideEffects(void) { areSideEffectsDisabled = true; }
}  // extern "C"

// NOTE: This struct can't be in the unnamed namespace

struct c_Spark_Obj : public c_BaseCAN_Obj {
    c_Spark_Obj(int busId, int deviceId, c_Spark_SparkModel unconfirmedModel)
        : c_BaseCAN_Obj(modelToDeviceType(unconfirmedModel), busId, deviceId),
          m_expectedSparkModel{unconfirmedModel},
          m_simDevice{c_SIM_Spark_Create(busId, deviceId, unconfirmedModel)} {}

    c_Spark_FirmwareVersion m_firmwareVersion{};
    c_Spark_DataPortConfig m_dataPortConfig{c_Spark_kDataPortConfigDefault};
    bool m_dataPortConfigured{false};

    bool m_inverted{false};

    int m_activeSetpointApi{-1};

    c_Spark_SparkModel m_sparkModel{c_Spark_Unknown};

    uint8_t m_motorInterface{};

    c_Spark_SparkModel m_expectedSparkModel;
    c_Spark_MotorType m_expectedMotorType{};

    c_SIM_Spark_handle m_simDevice{nullptr};
};

namespace {

constexpr std::array kControlTypeFrames{
    SPARK_DUTY_CYCLE_SETPOINT_FRAME_ID,          // 0
    SPARK_VELOCITY_SETPOINT_FRAME_ID,            // 1
    SPARK_VOLTAGE_SETPOINT_FRAME_ID,             // 2
    SPARK_POSITION_SETPOINT_FRAME_ID,            // 3
    SPARK_CURRENT_SETPOINT_FRAME_ID,             // 4
    SPARK_MAXMOTION_POSITION_SETPOINT_FRAME_ID,  // 5
    SPARK_MAXMOTION_VELOCITY_SETPOINT_FRAME_ID   // 6
};

/**
 * @brief Get spark model from handle if known. Otherwise, use expected model.
 *
 * @param handle
 * @return c_Spark_SparkModel
 */
inline c_Spark_SparkModel Spark_GetBestModel(c_Spark_handle handle) {
    return handle->m_sparkModel == c_Spark_Unknown
               ? handle->m_expectedSparkModel
               : handle->m_sparkModel;
}

// c_Spark_RegisterId() must be called first
c_Spark_handle Spark_Create_Inplace(int busId, int deviceId,
                                    c_Spark_SparkModel unconfirmedModel,
                                    c_REVLib_ErrorCode* status) {
    if (!CAN_IsValidDeviceId(deviceId)) {
        c_REVLib_SendError(c_REVLibError_InvalidCANId,
                           modelToDeviceType(unconfirmedModel), busId,
                           deviceId);

        // Don't allow a nullptr to be returned, invalid deviceId will just
        // fail, error is already sent
        // return nullptr;
    }

    c_Spark_handle handle = new c_Spark_Obj(busId, deviceId, unconfirmedModel);

    if (!s_Spark_ExistingDeviceIds.ContainsDevice(busId, deviceId)) {
        // I used SendError directly instead of c_REVLib_SendError because
        // we don't want to expose a REVLib error code for this condition.
        REVLib_SendLowLevelError(
            1, "c_Spark_RegisterId() was not called before c_Spark_Create()");
    }

    REVLib_SetLastError(handle, c_REVLibError_None);

    if (!c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_REVLib_RunDaemon();
        c_Spark_RunStatusFrameManager();
    }

    *status = c_REVLibError_None;
    c_Spark_FirmwareVersion fwVersion;
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        *status = c_REVLibError_None;
    } else if (c_Spark_GetFirmwareVersion(handle, &fwVersion) !=
               c_REVLibError_None) {
        *status = c_REVLibError_CantFindFirmware;
    } else {
        handle->m_firmwareVersion = fwVersion;

        if (fwVersion.versionRaw < kMinFirmwareVersion) {
            *status = c_REVLibError_FirmwareTooOld;
        } else if (isVersionTooNew(fwVersion)) {
            *status = c_REVLibError_FirmwareTooNew;
        } else if (!kAllowDebugFirmwareBuilds && fwVersion.debugBuild != 0) {
            *status = c_REVLibError_FirmwareTooOld;
        }
    }

    if (*status != c_REVLibError_None) {
        REVLib_SendError(handle, *status);
    }

    // Make sure no frames are repeating before registering
    int halStatus{};
    for (int controlType : kControlTypeFrames) {
        halStatus = REVLib_StopCANPacketRepeating(
            handle, controlType, "Create in place: Stop Repeating");
        if (halStatus != c_REVLibError_None) {
            break;
        }
    }

    handle->m_activeSetpointApi = 0;

    c_Spark_GetSparkModel(handle, &handle->m_sparkModel);

    return handle;
}

void GetSparkStatusPeriodsFromDevice(c_Spark_handle handle) {
    const std::array<status_period_info_t, 10> statusPeriods{
        {{c_Spark_kStatus0Period, &handle->m_status0Period_ms},
         {c_Spark_kStatus1Period, &handle->m_status1Period_ms},
         {c_Spark_kStatus2Period, &handle->m_status2Period_ms},
         {c_Spark_kStatus3Period, &handle->m_status3Period_ms},
         {c_Spark_kStatus4Period, &handle->m_status4Period_ms},
         {c_Spark_kStatus5Period, &handle->m_status5Period_ms},
         {c_Spark_kStatus6Period, &handle->m_status6Period_ms},
         {c_Spark_kStatus7Period, &handle->m_status7Period_ms},
         {c_Spark_kStatus8Period, &handle->m_status8Period_ms},
         {c_Spark_kStatus9Period, &handle->m_status9Period_ms}}};

    for (const auto& sp : statusPeriods) {
        uint32_t period{100};
        c_Spark_GetParameterUint32(handle, sp.periodId, &period);
        *sp.handle_statusPeriod_ms = static_cast<int32_t>(period);
    }
}

}  // namespace

// High-level libraries should throw an exception if this
// returns c_REVLibError_DuplicateCANId
c_REVLib_ErrorCode c_Spark_RegisterId(int busId, int deviceId) {
    if (!s_Spark_ExistingDeviceIds.InsertDevice(busId, deviceId)) {
        return c_REVLibError_DuplicateCANId;
    }
    return c_REVLibError_None;
}

// c_Spark_RegisterId() must be called first
c_Spark_handle c_Spark_Create(int busId, int deviceId,
                              c_Spark_MotorType motorType,
                              c_Spark_SparkModel unconfirmedModel,
                              c_REVLib_ErrorCode* status) {
    c_Spark_handle handle =
        Spark_Create_Inplace(busId, deviceId, unconfirmedModel, status);

    handle->m_expectedMotorType = motorType;

    if (*status != c_REVLibError_None) {
        return handle;
    }

    if (areSideEffectsDisabled) {
        handle->m_deviceType = UnknownREVDevice;
        return handle;
    }

    c_Spark_GetMotorInterface(handle, &handle->m_motorInterface);

    const c_Spark_SparkModel sparkModel{Spark_GetBestModel(handle)};

    // Store current data port config for MAX
    if (sparkModel == c_Spark_SparkMax) {
        uint32_t dataPortConfig;
        if (c_Spark_GetParameterUint32(handle, c_Spark_kCompatibilityPortConfig,
                                       &dataPortConfig) == c_REVLibError_None) {
            handle->m_dataPortConfig =
                dataPortConfig == c_Spark_kDataPortConfigAltEncoder
                    ? c_Spark_kDataPortConfigAltEncoder
                    : c_Spark_kDataPortConfigDefault;
        } else {
            // Assume default data port config if param query fails
            handle->m_dataPortConfig = c_Spark_kDataPortConfigDefault;
        }
    }

    // TODO(jan): Revisit this. Realistically, firmware should be handling this
    // check and REVLib just reports the error if there is one.
    if (sparkModel != c_Spark_SparkMax &&
        handle->m_motorInterface != kMotorInterfaceDockId &&
        motorType == c_Spark_kMotorType_BRUSHED) {
        *status = c_REVLibError_SparkFlexBrushedWithoutDock;
        REVLib_SendError(handle, *status);
        return handle;
    }

    c_Spark_SetMotorType(handle, motorType);

    handle->m_deviceType = modelToDeviceType(sparkModel);

    GetSparkStatusPeriodsFromDevice(handle);

    if (sparkModel == c_Spark_SparkMax) {
        getREVLibDriver()->reportDeviceUsage(REVDevice::SparkMax, busId,
                                             deviceId);
    } else if (sparkModel == c_Spark_SparkFlex) {
        getREVLibDriver()->reportDeviceUsage(REVDevice::SparkFlex, busId,
                                             deviceId);
    }

    return handle;
}

void c_Spark_Close(c_Spark_handle handle) {
    if (handle == NULL) {
        return;
    }

    s_Spark_ExistingDeviceIds.RemoveDevice(
        static_cast<uint8_t>(handle->m_busId),
        static_cast<uint8_t>(handle->m_deviceId));

    REVLib_StopCANPacketRepeating(handle, handle->m_activeSetpointApi,
                                  "Spark destroy: Stop Repeating");

    c_REVLib_StopDaemon();
    c_Spark_StopStatusFrameManager();

    if (handle->m_simDevice != NULL) {
        c_SIM_Spark_Close(handle->m_simDevice);
    }
}

void c_Spark_Destroy(c_Spark_handle handle) {
    if (handle == NULL) {
        return;
    }

    c_SIM_Spark_Destroy(handle->m_simDevice);
    delete handle;
}

c_REVLib_ErrorCode c_Spark_GetFirmwareVersion(
    c_Spark_handle handle, c_Spark_FirmwareVersion* fwVersion) {
    // Start with invalid firmware
    fwVersion->versionRaw = 0;

    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return c_REVLibError_None;
    } else {
        std::array<uint8_t, kMaxPacketLength> packedData;
        const c_REVLib_ErrorCode status = REVLib_WriteAndReadRtrCANPacket(
            handle, packedData, SPARK_GET_FIRMWARE_VERSION_FRAME_ID,
            SPARK_GET_FIRMWARE_VERSION_LENGTH, "Get Firmware Version");

        if (status != c_REVLibError_None) {
            // Firmware version not received
            REVLib_SetLastError(handle, c_REVLibError_CantFindFirmware);
            return c_REVLibError_CantFindFirmware;
        }

        spark_get_firmware_version_t frameData;
        spark_get_firmware_version_unpack(&frameData, packedData.data(),
                                          SPARK_GET_FIRMWARE_VERSION_LENGTH);

        fwVersion->major = frameData.major;
        fwVersion->minor = frameData.minor;
        fwVersion->build = frameData.build;
        fwVersion->debugBuild = frameData.debug_build;
        fwVersion->versionRaw =
            (static_cast<uint32_t>(fwVersion->major) << 24) |
            (static_cast<uint32_t>(fwVersion->minor) << 16) |
            static_cast<uint32_t>(fwVersion->build);
    }
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_GetSerialNumber(c_Spark_handle handle,
                                           uint32_t* serialNumber[3]) {
    return c_REVLibError_NotImplemented;
}

c_REVLib_ErrorCode c_Spark_GetBusId(c_Spark_handle handle, int* busId) {
    *busId = handle->m_busId;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_GetDeviceId(c_Spark_handle handle, int* deviceId) {
    *deviceId = handle->m_deviceId;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_SetMotorType(c_Spark_handle handle,
                                        c_Spark_MotorType type) {
    return c_Spark_SetParameterUint32(handle, c_Spark_kMotorType,
                                      static_cast<uint32_t>(type));
}

void c_Spark_SetPeriodicFrameTimeout(c_Spark_handle handle, int timeoutMs) {
    if (timeoutMs < 0) {
        return;
    }
    handle->m_periodicFrameTimeout_ms = timeoutMs;
}

void c_Spark_SetCANMaxRetries(c_Spark_handle handle, int numRetries) {
    if (numRetries < 0) {
        return;
    }
    handle->m_canMaxRetryCount = numRetries;
}

void c_Spark_SetControlFramePeriod(c_Spark_handle handle, int periodMs) {
    if (periodMs < 0) {
        return;
    }
    handle->m_controlFramePeriod_ms = periodMs;
}

int c_Spark_GetControlFramePeriod(c_Spark_handle handle) {
    return handle->m_controlFramePeriod_ms;
}

static c_REVLib_ErrorCode c_Spark_ParamStatusToErrorCode(
    c_Spark_ParameterStatus status) {
    if (status == c_Spark_kParamOK) {
        return c_REVLibError_None;
    }

    return static_cast<c_REVLib_ErrorCode>(
        static_cast<int>(status) +
        static_cast<int>(c_REVLibError_ParamInvalidID) - 1);
}

static std::string getValueText(c_REVLib_ParameterType type, uint32_t value) {
    switch (type) {
        case c_REVLib_kInt32:
            return std::to_string(static_cast<uint32_t>(value));
        case c_REVLib_kUint32:
            return std::to_string(value);
        case c_REVLib_kFloat32: {
            float f;
            std::memcpy(&f, &value, 4);
            return std::to_string(f);
        }
        case c_REVLib_kBool:
            return value != 0 ? "true" : "false";
        default:
            return "unused";
    }
}

static c_REVLib_ErrorCode c_Spark_SetParameterCore(
    c_Spark_handle handle, c_Spark_ConfigParameter parameterID,
    c_REVLib_ParameterType type, uint32_t value) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return c_SIM_Spark_SetParameter(handle->m_simDevice,
                                        static_cast<uint8_t>(parameterID),
                                        static_cast<uint8_t>(type), value);
    }

    spark_parameter_write_t frameDataOut;
    uint8_t packedDataOut[SPARK_PARAMETER_WRITE_LENGTH] = {0};

    frameDataOut.value = value;
    frameDataOut.parameter_id = parameterID;

    spark_parameter_write_pack(packedDataOut, &frameDataOut,
                               SPARK_PARAMETER_WRITE_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, packedDataOut, SPARK_PARAMETER_WRITE_FRAME_ID, packedDataIn,
        SPARK_PARAMETER_WRITE_RESPONSE_FRAME_ID,
        SPARK_PARAMETER_WRITE_RESPONSE_LENGTH,
        fmt::format("Set parameter ID {}", static_cast<int>(parameterID)));

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check the status of the response
    spark_parameter_write_response_t frameDataIn;
    spark_parameter_write_response_unpack(
        &frameDataIn, packedDataIn.data(),
        SPARK_PARAMETER_WRITE_RESPONSE_LENGTH);

    c_Spark_ParameterStatus paramStatus =
        static_cast<c_Spark_ParameterStatus>(frameDataIn.result_code);

    if (frameDataIn.parameter_id == static_cast<uint8_t>(parameterID) &&
        paramStatus != c_Spark_kParamOK) {
        c_REVLib_ErrorCode errCode =
            c_Spark_ParamStatusToErrorCode(paramStatus);
        std::string errMsg{fmt::format(
            "({}) {}, invalid value is {}", static_cast<int>(parameterID),
            c_Spark_GetParameterName(parameterID), getValueText(type, value))};
        REVLib_SendErrorText(handle, errCode, errMsg);
        REVLib_SetLastError(handle, errCode);
        return errCode;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

static c_REVLib_ErrorCode c_Spark_GetParameterCore(
    c_Spark_handle handle, c_Spark_ConfigParameter parameterID,
    c_REVLib_ParameterType expectedType, uint32_t* value) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return c_SIM_Spark_GetParameter(
            handle->m_simDevice, static_cast<uint8_t>(parameterID),
            static_cast<uint8_t>(expectedType), value);
    }

    // Check that the parameter type matches before transaction
    if (c_Spark_GetParameterType(parameterID) != expectedType) {
        REVLib_SendErrorText(handle, c_REVLibError_ParamMismatchType,
                             std::to_string(static_cast<uint8_t>(parameterID)));

        REVLib_SetLastError(handle, c_REVLibError_ParamMismatchType);
        return c_REVLibError_ParamMismatchType;
    }

    // Calculate double read frame id. This math only works for parameters
    // 0-255. If more parameters are added to the spec, this will need to be
    // updated.
    const int32_t parameterReadId =
        SPARK_READ_PARAMETER_0_AND_1_FRAME_ID + ((parameterID / 2) << 6);

#if 0  // For debugging
    fmt::println("GetParamCore: id:{}, prId:{:#x}",
                 static_cast<int>(parameterID), parameterReadId);
#endif

    // Request parameter
    std::array<uint8_t, kMaxPacketLength> packedDataIn{};
    const c_REVLib_ErrorCode status = REVLib_WriteAndReadRtrCANPacket(
        handle, packedDataIn, parameterReadId,
        SPARK_READ_PARAMETER_0_AND_1_LENGTH,
        fmt::format("Get parameter ID {}, frameID {}",
                    static_cast<int>(parameterID), parameterReadId));

    if (status != c_REVLibError_None) {
        return status;
    }

    // Extract parameter from payload
    spark_read_parameter_0_and_1_t frameDataIn;
    spark_read_parameter_0_and_1_unpack(&frameDataIn, packedDataIn.data(),
                                        SPARK_READ_PARAMETER_0_AND_1_LENGTH);

    *value = parameterID % 2 == 0 ? frameDataIn.first_parameter_value
                                  : frameDataIn.second_parameter_value;

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_SetParameterFloat32(c_Spark_handle handle,
                                               c_Spark_ConfigParameter paramId,
                                               float value) {
    uint32_t tmp;
    std::memcpy(&tmp, &value, sizeof(tmp));
    return c_Spark_SetParameterCore(handle, paramId, c_REVLib_kFloat32, tmp);
}

c_REVLib_ErrorCode c_Spark_SetParameterInt32(c_Spark_handle handle,
                                             c_Spark_ConfigParameter paramId,
                                             int32_t value) {
    uint32_t tmp;
    std::memcpy(&tmp, &value, sizeof(tmp));
    return c_Spark_SetParameterCore(handle, paramId, c_REVLib_kInt32, tmp);
}

c_REVLib_ErrorCode c_Spark_SetParameterUint32(c_Spark_handle handle,
                                              c_Spark_ConfigParameter paramId,
                                              uint32_t value) {
    return c_Spark_SetParameterCore(handle, paramId, c_REVLib_kUint32, value);
}

c_REVLib_ErrorCode c_Spark_SetParameterBool(c_Spark_handle handle,
                                            c_Spark_ConfigParameter paramId,
                                            uint8_t value) {
    return c_Spark_SetParameterCore(handle, paramId, c_REVLib_kBool,
                                    value ? 1 : 0);
}

c_REVLib_ErrorCode c_Spark_GetParameterFloat32(c_Spark_handle handle,
                                               c_Spark_ConfigParameter paramId,
                                               float* value) {
    uint32_t tmp = 0;
    c_REVLib_ErrorCode canStatus =
        c_Spark_GetParameterCore(handle, paramId, c_REVLib_kFloat32, &tmp);

    std::memcpy(value, &tmp, sizeof(*value));
    return canStatus;
}

c_REVLib_ErrorCode c_Spark_GetParameterInt32(c_Spark_handle handle,
                                             c_Spark_ConfigParameter paramId,
                                             int32_t* value) {
    uint32_t tmp = 0;
    c_REVLib_ErrorCode canStatus =
        c_Spark_GetParameterCore(handle, paramId, c_REVLib_kInt32, &tmp);

    std::memcpy(value, &tmp, sizeof(*value));
    return canStatus;
}

c_REVLib_ErrorCode c_Spark_GetParameterUint32(c_Spark_handle handle,
                                              c_Spark_ConfigParameter paramId,
                                              uint32_t* value) {
    return c_Spark_GetParameterCore(handle, paramId, c_REVLib_kUint32, value);
}

c_REVLib_ErrorCode c_Spark_GetParameterBool(c_Spark_handle handle,
                                            c_Spark_ConfigParameter paramId,
                                            uint8_t* value) {
    uint32_t tmp = 0;
    c_REVLib_ErrorCode canStatus =
        c_Spark_GetParameterCore(handle, paramId, c_REVLib_kBool, &tmp);

    *value = tmp ? 1 : 0;
    return canStatus;
}

void enableFrameIfNeeded(const c_Spark_handle handle,
                         const c_REVLib_ErrorCode revlibError,
                         const int frameId) {
    if (revlibError == c_REVLibError_None) {
        c_Spark_DequeueStatusFrame(handle, frameId);
    } else {
        c_Spark_QueueStatusFrame(handle, frameId);
    }
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus0(
    c_Spark_handle handle, c_Spark_PeriodicStatus0* rawFrame) {
    rawFrame->appliedOutput = c_SIM_Spark_GetAppliedOutput(handle->m_simDevice);
    rawFrame->voltage = c_SIM_Spark_GetBusVoltage(handle->m_simDevice);
    rawFrame->current = c_SIM_Spark_GetOutputCurrent(handle->m_simDevice);
    rawFrame->motorTemperature =
        c_SIM_Spark_GetMotorTemperature(handle->m_simDevice);

    // if limit switches have not been initialized, don't make gui objects

    // forward switch
    if (c_SIM_Spark_GetSimLimitSwitchExists(handle->m_simDevice, true)) {
        c_SIM_Spark_LimitSwitch_handle simForwardLimitSwitch =
            c_SIM_Spark_GetOrCreateSimForwardLimitSwitch(handle->m_simDevice);
        rawFrame->hardForwardLimitReached =
            c_SIM_Spark_GetSimLimitSwitchIsPressed(simForwardLimitSwitch) ? 1
                                                                          : 0;

    } else {
        // if sim forward switch has not been initialized, treat switches as
        // always unpressed
        rawFrame->hardForwardLimitReached = 0;
    }

    rawFrame->softForwardLimitReached =
        c_SIM_Spark_SoftForwardLimitReached(handle->m_simDevice);

    // reverse switch
    if (c_SIM_Spark_GetSimLimitSwitchExists(handle->m_simDevice, false)) {
        c_SIM_Spark_LimitSwitch_handle simReverseLimitSwitch =
            c_SIM_Spark_GetOrCreateSimReverseLimitSwitch(handle->m_simDevice);
        rawFrame->hardReverseLimitReached =
            c_SIM_Spark_GetSimLimitSwitchIsPressed(simReverseLimitSwitch) ? 1
                                                                          : 0;
    } else {
        // if sim forward switch has not been initialized, treat switches as
        // always unpressed
        rawFrame->hardReverseLimitReached = 0;
    }

    rawFrame->softReverseLimitReached =
        c_SIM_Spark_SoftReverseLimitReached(handle->m_simDevice);

    rawFrame->inverted = c_SIM_Spark_GetInverted(handle->m_simDevice);
    rawFrame->primaryHeartbeatLock = 0;

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus0(
    c_Spark_handle handle, c_Spark_PeriodicStatus0* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus0(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_0_FRAME_ID,
        handle->m_status0Period_ms, SPARK_STATUS_0_LENGTH, "Period Status 0");

    // REVLib_ReadCANPacketTimeout will always either return the latest packet
    // data or zero the packet
    spark_status_0_t frameData;
    spark_status_0_unpack(&frameData, packedData.data(), SPARK_STATUS_0_LENGTH);

    rawFrame->appliedOutput =
        (static_cast<float>(
            spark_status_0_applied_output_decode(frameData.applied_output))) *
        (handle->m_inverted ? -1 : 1);
    rawFrame->voltage =
        static_cast<float>(spark_status_0_voltage_decode(frameData.voltage));
    rawFrame->current =
        static_cast<float>(spark_status_0_current_decode(frameData.current));
    rawFrame->motorTemperature = frameData.motor_temperature;
    rawFrame->hardForwardLimitReached = frameData.hard_forward_limit_reached;
    rawFrame->hardReverseLimitReached = frameData.hard_reverse_limit_reached;
    rawFrame->softForwardLimitReached = frameData.soft_forward_limit_reached;
    rawFrame->softReverseLimitReached = frameData.soft_reverse_limit_reached;
    rawFrame->inverted = frameData.inverted;
    rawFrame->primaryHeartbeatLock = frameData.primary_heartbeat_lock;
    rawFrame->sparkModel = frameData.spark_model;

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus1(
    c_Spark_handle handle, c_Spark_PeriodicStatus1* rawFrame) {
    if (c_SIM_Spark_GetSimFaultManagerExists(handle->m_simDevice)) {
        c_SIM_Spark_FaultManager_handle simFaultManager =
            c_SIM_Spark_GetOrCreateSimFaultManager(handle->m_simDevice);
        c_SIM_Spark_SetRawframeFromSimFaults(simFaultManager, rawFrame);
    } else {
        // if the fault manager has not been set up, all faults off
        rawFrame->otherFault = 0;
        rawFrame->motorTypeFault = 0;
        rawFrame->sensorFault = 0;
        rawFrame->canFault = 0;
        rawFrame->temperatureFault = 0;
        rawFrame->drvFault = 0;
        rawFrame->escEepromFault = 0;
        rawFrame->firmwareFault = 0;
        rawFrame->brownoutWarning = 0;
        rawFrame->overcurrentWarning = 0;
        rawFrame->escEepromWarning = 0;
        rawFrame->extEepromWarning = 0;
        rawFrame->sensorWarning = 0;
        rawFrame->stallWarning = 0;
        rawFrame->hasResetWarning = 0;
        rawFrame->otherWarning = 0;
        rawFrame->otherStickyFault = 0;
        rawFrame->motorTypeStickyFault = 0;
        rawFrame->sensorStickyFault = 0;
        rawFrame->canStickyFault = 0;
        rawFrame->temperatureStickyFault = 0;
        rawFrame->drvStickyFault = 0;
        rawFrame->escEepromStickyFault = 0;
        rawFrame->firmwareStickyFault = 0;
        rawFrame->brownoutStickyWarning = 0;
        rawFrame->overcurrentStickyWarning = 0;
        rawFrame->escEepromStickyWarning = 0;
        rawFrame->extEepromStickyWarning = 0;
        rawFrame->sensorStickyWarning = 0;
        rawFrame->stallStickyWarning = 0;
        rawFrame->hasResetStickyWarning = 0;
        rawFrame->otherStickyWarning = 0;
    }
    rawFrame->isFollower = c_SIM_Spark_IsFollower(handle->m_simDevice);
    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus1(
    c_Spark_handle handle, c_Spark_PeriodicStatus1* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus1(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_1_FRAME_ID,
        handle->m_status1Period_ms, SPARK_STATUS_1_LENGTH, "Period Status 1");

    enableFrameIfNeeded(handle, revlibError, 1);

    spark_status_1_t frameData;
    spark_status_1_unpack(&frameData, packedData.data(), SPARK_STATUS_1_LENGTH);

    rawFrame->otherFault = frameData.other_fault;
    rawFrame->motorTypeFault = frameData.motor_type_fault;
    rawFrame->sensorFault = frameData.sensor_fault;
    rawFrame->canFault = frameData.can_fault;
    rawFrame->temperatureFault = frameData.temperature_fault;
    rawFrame->drvFault = frameData.drv_fault;
    rawFrame->escEepromFault = frameData.esc_eeprom_fault;
    rawFrame->firmwareFault = frameData.firmware_fault;
    rawFrame->brownoutWarning = frameData.brownout_warning;
    rawFrame->overcurrentWarning = frameData.overcurrent_warning;
    rawFrame->escEepromWarning = frameData.esc_eeprom_warning;
    rawFrame->extEepromWarning = frameData.ext_eeprom_warning;
    rawFrame->sensorWarning = frameData.sensor_warning;
    rawFrame->stallWarning = frameData.stall_warning;
    rawFrame->hasResetWarning = frameData.has_reset_warning;
    rawFrame->otherWarning = frameData.other_warning;
    rawFrame->otherStickyFault = frameData.other_sticky_fault;
    rawFrame->motorTypeStickyFault = frameData.motor_type_sticky_fault;
    rawFrame->sensorStickyFault = frameData.sensor_sticky_fault;
    rawFrame->canStickyFault = frameData.can_sticky_fault;
    rawFrame->temperatureStickyFault = frameData.temperature_sticky_fault;
    rawFrame->drvStickyFault = frameData.drv_sticky_fault;
    rawFrame->escEepromStickyFault = frameData.esc_eeprom_sticky_fault;
    rawFrame->firmwareStickyFault = frameData.firmware_sticky_fault;
    rawFrame->brownoutStickyWarning = frameData.brownout_sticky_warning;
    rawFrame->overcurrentStickyWarning = frameData.overcurrent_sticky_warning;
    rawFrame->escEepromStickyWarning = frameData.esc_eeprom_sticky_warning;
    rawFrame->extEepromStickyWarning = frameData.ext_eeprom_sticky_warning;
    rawFrame->sensorStickyWarning = frameData.sensor_sticky_warning;
    rawFrame->stallStickyWarning = frameData.stall_sticky_warning;
    rawFrame->hasResetStickyWarning = frameData.has_reset_sticky_warning;
    rawFrame->otherStickyWarning = frameData.other_sticky_warning;
    rawFrame->isFollower = frameData.is_follower;

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus2(
    c_Spark_handle handle, c_Spark_PeriodicStatus2* rawFrame) {
    c_SIM_Spark_RelativeEncoder_handle simRelativeEncoder =
        c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle->m_simDevice);

    rawFrame->primaryEncoderPosition = static_cast<float>(
        c_SIM_Spark_GetSimRelativeEncoderPosition(simRelativeEncoder));
    rawFrame->primaryEncoderVelocity = static_cast<float>(
        c_SIM_Spark_GetSimRelativeEncoderVelocity(simRelativeEncoder));
    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus2(
    c_Spark_handle handle, c_Spark_PeriodicStatus2* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus2(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_2_FRAME_ID,
        handle->m_status2Period_ms, SPARK_STATUS_2_LENGTH, "Period Status 2");

    enableFrameIfNeeded(handle, revlibError, 2);

    spark_status_2_t frameData;
    spark_status_2_unpack(&frameData, packedData.data(), SPARK_STATUS_2_LENGTH);

    rawFrame->primaryEncoderVelocity =
        spark_status_2_primary_encoder_velocity_decode(
            frameData.primary_encoder_velocity);
    rawFrame->primaryEncoderPosition =
        spark_status_2_primary_encoder_position_decode(
            frameData.primary_encoder_position);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus3(
    c_Spark_handle handle, c_Spark_PeriodicStatus3* rawFrame) {
    c_SIM_Spark_AnalogSensor_handle simAnalogSensor =
        c_SIM_Spark_GetOrCreateSimAnalogSensor(handle->m_simDevice);
    rawFrame->analogVoltage = static_cast<float>(
        c_SIM_Spark_GetSimAnalogSensorVoltage(simAnalogSensor));
    rawFrame->analogPosition = static_cast<float>(
        c_SIM_Spark_GetSimAnalogSensorPosition(simAnalogSensor));

    rawFrame->analogVelocity = (static_cast<float>(
        c_SIM_Spark_GetSimAnalogSensorVelocity(simAnalogSensor)));
    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus3(
    c_Spark_handle handle, c_Spark_PeriodicStatus3* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus3(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_3_FRAME_ID,
        handle->m_status3Period_ms, SPARK_STATUS_3_LENGTH, "Period Status 3");

    enableFrameIfNeeded(handle, revlibError, 3);

    spark_status_3_t frameData;
    spark_status_3_unpack(&frameData, packedData.data(), SPARK_STATUS_3_LENGTH);

    rawFrame->analogVoltage =
        spark_status_3_analog_voltage_decode(frameData.analog_voltage);
    rawFrame->analogVelocity =
        spark_status_3_analog_velocity_decode(frameData.analog_velocity);
    rawFrame->analogPosition =
        spark_status_3_analog_position_decode(frameData.analog_position);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus4(
    c_Spark_handle handle, c_Spark_PeriodicStatus4* rawFrame) {
    c_SIM_Spark_ExtOrAltEncoder_handle simExtOrAltEncoder =
        c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle->m_simDevice);

    rawFrame->externalOrAltEncoderPosition = static_cast<float>(
        c_SIM_Spark_GetSimExtOrAltEncoderPosition(simExtOrAltEncoder));
    rawFrame->externalOrAltEncoderVelocity = static_cast<float>(
        c_SIM_Spark_GetSimExtOrAltEncoderVelocity(simExtOrAltEncoder));
    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus4(
    c_Spark_handle handle, c_Spark_PeriodicStatus4* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus4(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_4_FRAME_ID,
        handle->m_status4Period_ms, SPARK_STATUS_4_LENGTH, "Period Status 4");

    enableFrameIfNeeded(handle, revlibError, 4);

    spark_status_4_t frameData;
    spark_status_4_unpack(&frameData, packedData.data(), SPARK_STATUS_4_LENGTH);

    rawFrame->externalOrAltEncoderVelocity =
        spark_status_4_external_or_alt_encoder_velocity_decode(
            frameData.external_or_alt_encoder_velocity);
    rawFrame->externalOrAltEncoderPosition =
        spark_status_4_external_or_alt_encoder_velocity_decode(
            frameData.external_or_alt_encoder_position);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus5(
    c_Spark_handle handle, c_Spark_PeriodicStatus5* rawFrame) {
    c_SIM_Spark_AbsoluteEncoder_handle simAbsoluteEncoder =
        c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle->m_simDevice);

    rawFrame->dutyCycleEncoderPosition =
        c_SIM_Spark_GetSimAbsoluteEncoderPosition(simAbsoluteEncoder);
    rawFrame->dutyCycleEncoderVelocity =
        c_SIM_Spark_GetSimAbsoluteEncoderVelocity(simAbsoluteEncoder);

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus5(
    c_Spark_handle handle, c_Spark_PeriodicStatus5* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus5(handle, rawFrame);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_5_FRAME_ID,
        handle->m_status5Period_ms, SPARK_STATUS_5_LENGTH, "Period Status 5");

    enableFrameIfNeeded(handle, revlibError, 5);

    spark_status_5_t frameData;
    spark_status_5_unpack(&frameData, packedData.data(), SPARK_STATUS_5_LENGTH);

    rawFrame->dutyCycleEncoderVelocity =
        spark_status_5_duty_cycle_encoder_velocity_decode(
            frameData.duty_cycle_encoder_velocity);
    rawFrame->dutyCycleEncoderPosition =
        spark_status_5_duty_cycle_encoder_position_decode(
            frameData.duty_cycle_encoder_position);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus6(c_Spark_handle handle) {
    c_SIM_Spark_AbsoluteEncoder_handle simAbsoluteEncoder =
        c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle->m_simDevice);

    // these appear unneeded for sim, but will create the gui device
    (void)simAbsoluteEncoder;  // suppress warning

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus6(
    c_Spark_handle handle, c_Spark_PeriodicStatus6* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus6(handle);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_6_FRAME_ID,
        handle->m_status6Period_ms, SPARK_STATUS_6_LENGTH, "Period Status 6");

    enableFrameIfNeeded(handle, revlibError, 6);

    spark_status_6_t frameData;
    spark_status_6_unpack(&frameData, packedData.data(), SPARK_STATUS_6_LENGTH);

    rawFrame->unadjustedDutyCycle = spark_status_6_unadjusted_duty_cycle_decode(
        frameData.unadjusted_duty_cycle);
    rawFrame->dutyCyclePeriod =
        spark_status_6_duty_cycle_period_decode(frameData.duty_cycle_period);
    rawFrame->dutyCycleNoSignal = frameData.duty_cycle_no_signal;

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus7(
    c_Spark_PeriodicStatus7* rawFrame, c_Spark_handle handle) {
    rawFrame->iAccumulation = c_SIM_Spark_GetSimIAccum(handle->m_simDevice);

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus7(
    c_Spark_handle handle, c_Spark_PeriodicStatus7* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus7(rawFrame, handle);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_7_FRAME_ID,
        handle->m_status7Period_ms, SPARK_STATUS_7_LENGTH, "Period Status 7");

    enableFrameIfNeeded(handle, revlibError, 7);

    spark_status_7_t frameData;
    spark_status_7_unpack(&frameData, packedData.data(), SPARK_STATUS_7_LENGTH);

    rawFrame->iAccumulation =
        static_cast<float>(
            spark_status_7_i_accumulation_decode(frameData.i_accumulation)) *
        (handle->m_inverted ? -1 : 1);

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus8(
    c_Spark_PeriodicStatus8* rawFrame, c_Spark_handle handle) {
    rawFrame->setpoint = c_SIM_Spark_GetSetpoint(handle->m_simDevice);
    rawFrame->isAtSetpoint = c_SIM_Spark_IsAtSetpoint(handle->m_simDevice);
    rawFrame->selectedPidSlot =
        c_SIM_Spark_GetClosedLoopSlot(handle->m_simDevice);

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus8(
    c_Spark_handle handle, c_Spark_PeriodicStatus8* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus8(rawFrame, handle);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_8_FRAME_ID,
        handle->m_status8Period_ms, SPARK_STATUS_8_LENGTH, "Period Status 8");

    enableFrameIfNeeded(handle, revlibError, 8);

    spark_status_8_t frameData;
    spark_status_8_unpack(&frameData, packedData.data(), SPARK_STATUS_8_LENGTH);

    rawFrame->setpoint =
        static_cast<float>(spark_status_8_setpoint_decode(frameData.setpoint));
    rawFrame->isAtSetpoint = frameData.is_at_setpoint;
    rawFrame->selectedPidSlot = frameData.selected_pid_slot;

    return revlibError;
}

namespace {
c_REVLib_ErrorCode SIM_Spark_GetPeriodicStatus9(
    c_Spark_PeriodicStatus9* rawFrame, c_Spark_handle handle) {
    rawFrame->maxmotion_setpoint_position =
        c_SIM_Spark_GetMAXMotionSetpointPosition(handle->m_simDevice);
    rawFrame->maxmotion_setpoint_velocity =
        c_SIM_Spark_GetMAXMotionSetpointVelocity(handle->m_simDevice);

    return c_REVLibError_None;
}
}  // namespace

c_REVLib_ErrorCode c_Spark_GetPeriodicStatus9(
    c_Spark_handle handle, c_Spark_PeriodicStatus9* rawFrame) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return SIM_Spark_GetPeriodicStatus9(rawFrame, handle);
    }

    std::array<uint8_t, kMaxPacketLength> packedData{};

    c_REVLib_ErrorCode revlibError = REVLib_ReadCANPacketTimeout(
        handle, packedData, rawFrame->timestamp, SPARK_STATUS_9_FRAME_ID,
        handle->m_status9Period_ms, SPARK_STATUS_9_LENGTH, "Period Status 9");

    enableFrameIfNeeded(handle, revlibError, 9);

    spark_status_9_t frameData;
    spark_status_9_unpack(&frameData, packedData.data(), SPARK_STATUS_9_LENGTH);

    rawFrame->maxmotion_setpoint_position =
        static_cast<float>(spark_status_9_maxmotion_position_setpoint_decode(
            frameData.maxmotion_position_setpoint));
    rawFrame->maxmotion_setpoint_velocity =
        static_cast<float>(spark_status_9_maxmotion_velocity_setpoint_decode(
            frameData.maxmotion_velocity_setpoint));

    return revlibError;
}

c_REVLib_ErrorCode c_Spark_SetEncoderPosition(c_Spark_handle handle,
                                              float position) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_RelativeEncoder_handle simRelativeEncoder =
            c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle->m_simDevice);
        c_SIM_Spark_SetSimRelativeEncoderPosition(
            simRelativeEncoder, static_cast<double>(position));
        return c_REVLibError_None;
    }

    spark_set_primary_encoder_position_t frameData;
    uint8_t packedData[SPARK_SET_PRIMARY_ENCODER_POSITION_LENGTH];

    frameData.position = position * (handle->m_inverted ? -1 : 1);
    frameData.data_type = static_cast<uint8_t>(c_REVLib_kFloat32);
    spark_set_primary_encoder_position_pack(
        packedData, &frameData, SPARK_SET_PRIMARY_ENCODER_POSITION_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, SPARK_SET_PRIMARY_ENCODER_POSITION_FRAME_ID,
        "Set Encoder Position");
    return status;
}

c_REVLib_ErrorCode c_Spark_SetAltEncoderPosition(c_Spark_handle handle,
                                                 float position) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_ExtOrAltEncoder_handle simExtOrAltEncoder =
            c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle->m_simDevice);
        c_SIM_Spark_SetSimExtOrAltEncoderPosition(
            simExtOrAltEncoder, static_cast<double>(position));
        return c_REVLibError_None;
    }

    spark_set_ext_or_alt_encoder_position_t frameData;
    uint8_t packedData[SPARK_SET_EXT_OR_ALT_ENCODER_POSITION_LENGTH];

    frameData.position = position * (handle->m_inverted ? -1 : 1);
    frameData.data_type = static_cast<uint8_t>(c_REVLib_kFloat32);
    spark_set_ext_or_alt_encoder_position_pack(
        packedData, &frameData, SPARK_SET_EXT_OR_ALT_ENCODER_POSITION_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, SPARK_SET_EXT_OR_ALT_ENCODER_POSITION_FRAME_ID,
        "Set External or Alternate Encoder Position");
    return status;
}

c_REVLib_ErrorCode c_Spark_SetIAccum(c_Spark_handle handle, float iAccum) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_SetSimIAccum(handle->m_simDevice, iAccum);
        return c_REVLibError_None;
    }

    spark_set_i_accumulation_t frameData;
    uint8_t packedData[SPARK_SET_I_ACCUMULATION_LENGTH];

    frameData.i_accumulation = iAccum * (handle->m_inverted ? -1 : 1);
    frameData.data_type = static_cast<uint8_t>(c_REVLib_kFloat32);
    spark_set_i_accumulation_pack(packedData, &frameData,
                                  SPARK_SET_I_ACCUMULATION_LENGTH);

    c_REVLib_ErrorCode status = REVLib_WriteCANPacket(
        handle, packedData, SPARK_SET_I_ACCUMULATION_FRAME_ID,
        "Set I Accumulation");
    return status;
}

c_REVLib_ErrorCode c_Spark_ResetSafeParameters(c_Spark_handle handle,
                                               uint8_t persist) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return c_SIM_Spark_RestoreFactoryDefaults(handle->m_simDevice, persist,
                                                  false);
    }

    spark_reset_safe_parameters_t frameDataOut;
    uint8_t packedDataOut[SPARK_RESET_SAFE_PARAMETERS_LENGTH];

    frameDataOut.magic_number = 0x8DC4;
    spark_reset_safe_parameters_pack(packedDataOut, &frameDataOut,
                                     SPARK_RESET_SAFE_PARAMETERS_LENGTH);

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, packedDataOut, SPARK_RESET_SAFE_PARAMETERS_FRAME_ID,
        packedDataIn, SPARK_RESET_SAFE_PARAMETERS_RESPONSE_FRAME_ID,
        SPARK_RESET_SAFE_PARAMETERS_RESPONSE_LENGTH, "Reset Safe Parameters");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check result
    spark_reset_safe_parameters_response_t frameDataIn;
    spark_reset_safe_parameters_response_unpack(
        &frameDataIn, packedDataIn.data(),
        SPARK_RESET_SAFE_PARAMETERS_RESPONSE_LENGTH);

    if (frameDataIn.result_code != 0) {
        // TODO(jan): Update with corresponding error code once
        // implemented in firmware
        status = c_REVLibError_Invalid;
    }

    return REVLib_HALErrorCheck(handle, status, "Reset Safe Parameters");
}

float c_Spark_SafeFloat(float f) {
    if (std::isinf(f) || std::isnan(f)) return 0;
    return f;
}

c_REVLib_ErrorCode c_Spark_SetpointCommand(c_Spark_handle handle, float value,
                                           c_Spark_ControlType ctrl,
                                           int pidSlot, float arbFeedforward,
                                           int arbFFUnits) {
    // TODO(jan): Re-look into feedbackSensorRange. This has since been removed
    // but can be found in the git history.
    int apiId{};
    size_t cntrlIdx = static_cast<size_t>(static_cast<int>(ctrl));

    if (cntrlIdx < kControlTypeFrames.size()) {
        apiId = kControlTypeFrames[cntrlIdx];
    } else {
        REVLib_SetLastError(handle, c_REVLibError_Invalid);
        return c_REVLibError_Invalid;
    }

    const float shiftedArbFF = (arbFeedforward * 1024);
    int16_t packedFF;

    if (shiftedArbFF > 32767) {
        packedFF = 32767;
    } else if (shiftedArbFF < -32767) {
        packedFF = -32767;
    } else {
        packedFF = static_cast<int16_t>(shiftedArbFF);
    }

    spark_position_setpoint_t frameData{};
    frameData.setpoint = value;
    frameData.arbitrary_feedforward = packedFF;
    frameData.pid_slot = static_cast<uint8_t>(pidSlot);
    frameData.arbitrary_feedforward_units =
        arbFFUnits ? uint8_t{1} : uint8_t{0};

    if (handle->m_inverted) {
        frameData.arbitrary_feedforward *= -1;
        frameData.setpoint *= -1;
    }

    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        handle->m_activeSetpointApi = apiId;
        return c_SIM_Spark_SetSetpoint(handle->m_simDevice, value,
                                       static_cast<uint8_t>(ctrl), pidSlot,
                                       arbFeedforward, arbFFUnits);
    }

    // Stop sending previous setpoint command if it is different
    if (handle->m_activeSetpointApi != apiId) {
        REVLib_StopCANPacketRepeating(handle, handle->m_activeSetpointApi,
                                      "Setpoint command: Stop Repeating");
    }

    handle->m_activeSetpointApi = apiId;

    uint8_t packedData[SPARK_POSITION_SETPOINT_LENGTH];
    spark_position_setpoint_pack(packedData, &frameData,
                                 SPARK_POSITION_SETPOINT_LENGTH);

    const c_REVLib_ErrorCode status = REVLib_WriteCANPacketRepeating(
        handle, packedData, handle->m_activeSetpointApi, "Setpoint command");

    if (status != c_REVLibError_None) {
        return status;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_SetInverted(c_Spark_handle handle,
                                       uint8_t inverted) {
    if (handle->m_firmwareVersion.versionRaw < kInvertOnControllerVersion &&
        handle->m_firmwareVersion.versionRaw != 0) {
        handle->m_inverted = inverted;
        REVLib_SetLastError(handle, c_REVLibError_None);
        return c_REVLibError_None;
    }

    handle->m_inverted = false;
    return c_Spark_SetParameterBool(handle, c_Spark_kInverted, inverted);
}

c_REVLib_ErrorCode c_Spark_GetInverted(c_Spark_handle handle,
                                       uint8_t* inverted) {
    if (handle->m_firmwareVersion.versionRaw < kInvertOnControllerVersion &&
        handle->m_firmwareVersion.versionRaw != 0) {
        *inverted = handle->m_inverted;
        REVLib_SetLastError(handle, c_REVLibError_None);
        return c_REVLibError_None;
    }
    return c_Spark_GetParameterBool(handle, c_Spark_kInverted, inverted);
}

namespace {

c_Spark_SparkModel GetSparkModelFromProductId(c_Spark_handle handle) {
    uint32_t productId{};
    c_REVLib_ErrorCode code =
        c_Spark_GetParameterUint32(handle, c_Spark_kProductId, &productId);
    if (code != c_REVLibError_None) {
        REVLib_SendErrorText(
            handle, code,
            "Getting Product ID parameter failed. Unable to account for "
            "device-specific behavior differences.\n");
        return c_Spark_Unknown;
    }

    if (productId == 0x2158) {
        return c_Spark_SparkMax;
    } else if (productId == 0x2159) {
        return c_Spark_SparkFlex;
    } else {
        const auto details = fmt::format(
            "Unknown Product ID {}. Unable to account for device-specific "
            "behavior differences.\n",
            productId);
        REVLib_SendLowLevelWarning(0, details, true);
        return c_Spark_Unknown;
    }
}

}  // namespace

c_REVLib_ErrorCode c_Spark_GetSparkModel(c_Spark_handle handle,
                                         c_Spark_SparkModel* model) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        uint8_t value = c_SIM_Spark_GetSparkModel(handle->m_simDevice);
        *model = static_cast<c_Spark_SparkModel>(value);
        return c_REVLibError_None;
    }

    c_Spark_PeriodicStatus0 statusFrame;
    const c_REVLib_ErrorCode status =
        c_Spark_GetPeriodicStatus0(handle, &statusFrame);

    if (status != c_REVLibError_None) {
        const auto details = fmt::format(
            "Unable to get periodic status 0 for bus {} device ID {}",
            handle->m_busId, handle->m_deviceId);
        REVLib_SendLowLevelWarning(0, details, true);

        *model = GetSparkModelFromProductId(handle);
        if (*model == c_Spark_Unknown) {
            const auto details = fmt::format(
                "Getting SPARK Model failed for bus {} device ID {}. Unable to "
                "account for device-specific behavior differences.\n",
                handle->m_busId, handle->m_deviceId);
            REVLib_SendErrorText(handle, status, details);
        }
        return status;
    }

    auto receivedModel =
        static_cast<c_Spark_SparkModel>(statusFrame.sparkModel);

    if ((receivedModel == c_Spark_Unknown) ||
        (receivedModel > c_Spark_SparkMax) ||
        (handle->m_expectedSparkModel != receivedModel)) {
        const auto details = fmt::format(
            "Received invalid model {} in status 0; expected {} for bus {} "
            "device ID {}.\n",
            static_cast<int>(receivedModel),
            static_cast<int>(handle->m_expectedSparkModel), handle->m_busId,
            handle->m_deviceId);
        REVLib_SendLowLevelWarning(0, details, true);

        *model = GetSparkModelFromProductId(handle);
    } else {
        *model = receivedModel;
    }

    return status;
}

c_REVLib_ErrorCode c_Spark_ClearFaults(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        if (c_SIM_Spark_GetSimFaultManagerExists(handle->m_simDevice)) {
            c_SIM_Spark_FaultManager_handle simFaultManager =
                c_SIM_Spark_GetOrCreateSimFaultManager(handle->m_simDevice);
            c_SIM_Spark_ClearSimFaults(simFaultManager);
        }
    }

    c_REVLib_ErrorCode status =
        REVLib_WriteCANPacket(handle, zeroLengthDataPacket,
                              SPARK_CLEAR_FAULTS_FRAME_ID, "Clear Faults");
    return status;
}

c_REVLib_ErrorCode c_Spark_PersistParameters(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        return c_REVLibError_None;
    }

    spark_persist_parameters_t frameDataOut{.magic_number = 0x3AA3};
    uint8_t packedDataOut[SPARK_PERSIST_PARAMETERS_LENGTH];

    spark_persist_parameters_pack(packedDataOut, &frameDataOut,
                                  SPARK_PERSIST_PARAMETERS_LENGTH);

    // Don't use configured m_canTimeoutMs, burn flash can take a while
    constexpr uint32_t timeoutMs = 1000;

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacketWithReadTimeout(
        handle, packedDataOut, SPARK_PERSIST_PARAMETERS_FRAME_ID, packedDataIn,
        SPARK_PERSIST_PARAMETERS_RESPONSE_FRAME_ID,
        SPARK_PERSIST_PARAMETERS_RESPONSE_LENGTH, timeoutMs,
        "Persist Parameters");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check result
    spark_persist_parameters_response_t frameDataIn;
    spark_persist_parameters_response_unpack(
        &frameDataIn, packedDataIn.data(),
        SPARK_PERSIST_PARAMETERS_RESPONSE_LENGTH);

    if (frameDataIn.result_code != 0) {
        c_REVLib_ErrorCode error;
        switch (frameDataIn.result_code) {
            // 255 either means device is enabled or magic number is
            // wrong, but since we send the magic number correctly, we
            // assume it is the former case
            case 255:
                error = c_REVLibError_CannotPersistParametersWhileEnabled;
                break;
            default:
                error = c_REVLibError_Unknown;
        }

        REVLib_SendError(handle, error);
        REVLib_SetLastError(handle, error);
        return error;
    }

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_SetCANTimeout(c_Spark_handle handle, int timeoutMs) {
    handle->m_canTimeout_ms = timeoutMs;
    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_GetMotorInterface(c_Spark_handle handle,
                                             uint8_t* motorInterface) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        *motorInterface = c_SIM_Spark_GetMotorInterface(handle->m_simDevice);
        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedData;
    const c_REVLib_ErrorCode status = REVLib_WriteAndReadRtrCANPacket(
        handle, packedData, SPARK_GET_MOTOR_INTERFACE_FRAME_ID,
        SPARK_GET_MOTOR_INTERFACE_LENGTH, "Get Motor Interface");

    if (status != c_REVLibError_None) {
        return status;
    }

    spark_get_motor_interface_t frameData;
    spark_get_motor_interface_unpack(&frameData, packedData.data(),
                                     SPARK_GET_MOTOR_INTERFACE_LENGTH);

    *motorInterface = frameData.motor_interface;

    return c_REVLibError_None;
}

// ID Query calls mostly use raw CAN since they are not yet tied to a
// specific device
// TODO: (dave) get rid of "goto cleanup" and use RAII
c_REVLib_ErrorCode c_Spark_IDQuery(int busId, uint32_t* uniqueIdArray,
                                   size_t uniqueIdArraySize,
                                   size_t* numberOfDevices) {
    static const uint32_t MAX_STREAM_MESSAGES = 64;
    uint32_t numMessages = 0;
    int32_t status = 0;
    c_REVLib_ErrorCode errCode;  // NOTE: This needs to be declared before the
                                 // first goto to avoid a compilation error
    CanMessage streamMessages[MAX_STREAM_MESSAGES];

    int32_t stream = getREVLibDriver()->createCanStream(
        busId, SPARK_UNIQUE_ID_BROADCAST_FRAME_ID, 0x1FFFFFFF,
        MAX_STREAM_MESSAGES, &status);
    if (status != 0 || stream == 0) {
        goto cleanup;
    }

    errCode = REVLib_WriteBroadcastCANPacket(zeroLengthDataPacket, busId,
                                             SPARK_UNIQUE_ID_BROADCAST_FRAME_ID,
                                             "ID Query");
    if (errCode != c_REVLibError_None) {
        goto cleanup;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    getREVLibDriver()->receiveCanMessages(
        stream, streamMessages, MAX_STREAM_MESSAGES, &numMessages, &status);
    if (status != 0) {
        goto cleanup;
    }

    *numberOfDevices = 0;
    for (unsigned int i = 0;
         i < std::min(numMessages, static_cast<uint32_t>(uniqueIdArraySize));
         i++) {
        if (streamMessages[i].dataSize >= sizeof(uniqueIdArray[0])) {
            std::memcpy(uniqueIdArray++, streamMessages[i].data,
                        sizeof(uniqueIdArray[0]));
            (*numberOfDevices)++;
        }
    }

cleanup:
    if (stream != 0) {
        getREVLibDriver()->closeStream(stream);
    }
    if (status != 0) {
        c_REVLib_SendErrorText(c_REVLibError_HAL, UnknownREVDevice, busId, 0,
                               getREVLibDriver()->getErrorMessage(status));
        return c_REVLibError_HAL;
    }
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_IDAssign(uint32_t uniqueId, uint8_t busId,
                                    uint8_t deviceId) {
    if (!CAN_IsValidDeviceId(deviceId)) {
        c_REVLib_SendErrorText(c_REVLibError_Invalid, UnknownREVDevice, busId,
                               deviceId,
                               "IDAssign must provide a valid CAN ID");
        return c_REVLibError_Invalid;
    }

    constexpr size_t ID_ASSIGN_MSG_SIZE{5};
    uint8_t data[ID_ASSIGN_MSG_SIZE];

    std::memcpy(data, &uniqueId, sizeof(uniqueId));
    data[sizeof(uniqueId)] = deviceId;

    const c_REVLib_ErrorCode status = REVLib_WriteBroadcastCANPacket(
        data, busId, SPARK_SET_CAN_ID_FRAME_ID, "ID Assign");

    return status;
}

c_REVLib_ErrorCode c_Spark_Identify(c_Spark_handle handle) {
    const c_REVLib_ErrorCode status = REVLib_WriteBroadcastCANPacket(
        zeroLengthDataPacket, handle->m_busId,
        SPARK_IDENTIFY_FRAME_ID | handle->m_deviceId, "Identify");

    return status;
}

c_REVLib_ErrorCode c_Spark_IdentifyUniqueId(int busId, uint32_t uniqueId) {
    constexpr size_t IDENFITY_MSG_SIZE{sizeof(uniqueId)};

    std::span<uint8_t> packet{reinterpret_cast<uint8_t*>(&uniqueId),
                              IDENFITY_MSG_SIZE};

    const c_REVLib_ErrorCode status = REVLib_WriteBroadcastCANPacket(
        packet, busId, SPARK_IDENTIFY_FRAME_ID, "Identify Unique ID");

    return status;
}

c_REVLib_ErrorCode c_Spark_GetSoftLimit(c_Spark_handle handle,
                                        c_Spark_LimitDirection sw,
                                        uint8_t* limit) {
    c_Spark_PeriodicStatus0 statusFrame;
    c_REVLib_ErrorCode status =
        c_Spark_GetPeriodicStatus0(handle, &statusFrame);
    *limit = sw == c_Spark_kReverse ? statusFrame.softReverseLimitReached
                                    : statusFrame.softForwardLimitReached;
    return status;
}

void c_Spark_SetSimAppliedOutput(c_Spark_handle handle, float appliedOutput) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_SetAppliedOutput(handle->m_simDevice, appliedOutput);
    }
}

c_REVLib_ErrorCode c_Spark_GetDataPortConfig(c_Spark_handle handle,
                                             c_Spark_DataPortConfig* config) {
    *config = handle->m_dataPortConfig;
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_IsDataPortConfigured(c_Spark_handle handle,
                                                uint8_t* configured) {
    *configured = handle->m_dataPortConfigured;
    return c_REVLibError_None;
}

c_REVLib_APIVersion c_Spark_GetAPIVersion(void) {
    return c_REVLib_APIVersion({.Major = c_Spark_kAPIMajorVersion,
                                .Minor = c_Spark_kAPIMinorVersion,
                                .Build = c_Spark_kAPIBuildVersion,
                                .Version = c_Spark_kAPIVersion});
}

void c_Spark_SetLastError(c_Spark_handle handle, c_REVLib_ErrorCode error) {
    REVLib_SetLastError(handle, error);
}

c_REVLib_ErrorCode c_Spark_GetLastError(c_Spark_handle handle) {
    return REVLib_GetLastError(handle);
}

/**
 * Helper function to validate parameters before processing them.
 */
c_REVLib_ErrorCode c_Spark_ValidateParameters(
    c_Spark_handle handle,
    std::unordered_map<c_Spark_ConfigParameter, uint32_t>& parameters) {
    // MAX-only checks
    if (Spark_GetBestModel(handle) == c_Spark_SparkMax) {
        // Check if data port was set to invalid in the higher level APIs
        if (parameters.contains(c_Spark_kCompatibilityPortConfig) &&
            parameters[c_Spark_kCompatibilityPortConfig] ==
                static_cast<uint32_t>(c_Spark_kDataPortConfigInvalid)) {
            return c_REVLibError_SparkMaxDataPortAlreadyConfiguredDifferently;
        }

        // Check if feedback sensor is compatible with the data port's config
        if (parameters.contains(c_Spark_kClosedLoopControlSensor)) {
            // Get data port config from parameters or fall back to handle's
            c_Spark_DataPortConfig dataPortConfig =
                parameters.contains(c_Spark_kCompatibilityPortConfig)
                    ? static_cast<c_Spark_DataPortConfig>(
                          parameters[c_Spark_kCompatibilityPortConfig])
                    : handle->m_dataPortConfig;

            if ((dataPortConfig == c_Spark_kDataPortConfigDefault &&
                 parameters[c_Spark_kClosedLoopControlSensor] ==
                     c_Spark_kSensor_ALT_ENCODER) ||
                (dataPortConfig == c_Spark_kDataPortConfigAltEncoder &&
                 parameters[c_Spark_kClosedLoopControlSensor] ==
                     c_Spark_kSensor_DUTY_CYCLE)) {
                return c_REVLibError_FeedbackSensorIncompatibleWithDataPortConfig;
            }
        }
    }

    if (handle->m_expectedMotorType == c_Spark_kMotorType_BRUSHLESS &&
        (parameters.contains(c_Spark_kEncoderCountsPerRev) ||
         parameters.contains(c_Spark_kEncoderInverted))) {
        return c_REVLibError_InvalidBrushlessEncoderConfiguration;
    }

    return c_REVLibError_None;
}

/**
 * Helper function to handle additional actions with parameters before sending
 * them to the device. Parameters are passed by reference and can be modified by
 * this function which can be useful for any kind of sanitization.
 */
c_REVLib_ErrorCode c_Spark_PreProcessParameters(
    c_Spark_handle handle,
    std::unordered_map<c_Spark_ConfigParameter, uint32_t>& parameters) {
    if (parameters.contains(c_Spark_kCompatibilityPortConfig)) {
        if (Spark_GetBestModel(handle) != c_Spark_SparkMax) {
            // Only MAX needs this parameter to be set (and it's only validated
            // for MAX, so this may contain an invalid value for other device
            // types)
            parameters.erase(c_Spark_kCompatibilityPortConfig);
        }
    }

    return c_REVLibError_None;
}

/**
 * Helper function to handle additional actions with parameters after sending
 * them to the device.
 */
c_REVLib_ErrorCode c_Spark_PostProcessParameters(
    c_Spark_handle handle,
    std::unordered_map<c_Spark_ConfigParameter, uint32_t>& parameters,
    bool resetSafeParameters) {
    // Handle follower mode if follower mode ID is configured
    if (parameters.contains(c_Spark_kFollowerModeLeaderId)) {
        c_REVLib_ErrorCode status;
        if (parameters[c_Spark_kFollowerModeLeaderId] != 0) {
            status = c_Spark_StartFollowerMode(handle);
        } else {
            status = c_Spark_StopFollowerMode(handle);
        }

        if (status != c_REVLibError_None) {
            return status;
        }
    }

    const std::array<status_period_info_t, 10> statusPeriods{
        {{c_Spark_kStatus0Period, &handle->m_status0Period_ms},
         {c_Spark_kStatus1Period, &handle->m_status1Period_ms},
         {c_Spark_kStatus2Period, &handle->m_status2Period_ms},
         {c_Spark_kStatus3Period, &handle->m_status3Period_ms},
         {c_Spark_kStatus4Period, &handle->m_status4Period_ms},
         {c_Spark_kStatus5Period, &handle->m_status5Period_ms},
         {c_Spark_kStatus6Period, &handle->m_status6Period_ms},
         {c_Spark_kStatus7Period, &handle->m_status7Period_ms},
         {c_Spark_kStatus8Period, &handle->m_status8Period_ms},
         {c_Spark_kStatus9Period, &handle->m_status9Period_ms}}};

    for (auto sp : statusPeriods) {
        if (parameters.contains(sp.periodId)) {
            const int periodMs = parameters[sp.periodId];
            *sp.handle_statusPeriod_ms = periodMs;
        }
    }

    // Update data port config if MAX
    if (Spark_GetBestModel(handle) == c_Spark_SparkMax) {
        if (parameters.contains(c_Spark_kCompatibilityPortConfig)) {
            // Use specified data port config if there is one
            handle->m_dataPortConfig = static_cast<c_Spark_DataPortConfig>(
                parameters[c_Spark_kCompatibilityPortConfig]);

            handle->m_dataPortConfigured = true;
        } else if (resetSafeParameters) {
            // Use default data port config if none is specified and parameters
            // were reset
            handle->m_dataPortConfig = c_Spark_kDataPortConfigDefault;

            handle->m_dataPortConfigured = true;
        }
    }

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_Configure(c_Spark_handle handle,
                                     const char* flattenedConfig,
                                     uint8_t resetSafeParameters,
                                     uint8_t persistParameters) {
    c_REVLib_ErrorCode status{c_REVLibError_None};

    // Unflatten string into processable data
    std::istringstream iss(flattenedConfig);
    char buffer[16];

    std::unordered_map<c_Spark_ConfigParameter, uint32_t> parameters;
    while (iss.getline(buffer, sizeof(buffer))) {
        std::string line(buffer);
        uint8_t delimiterIndex = line.find(',');
        std::string paramIdStr = line.substr(0, delimiterIndex);
        std::string paramValueStr = line.substr(delimiterIndex + 1);

        uint32_t paramId = static_cast<uint32_t>(std::stoul(paramIdStr));
        uint32_t paramValue;
        std::stringstream ss;
        ss << std::hex << paramValueStr;
        ss >> paramValue;

        parameters[static_cast<c_Spark_ConfigParameter>(paramId)] = paramValue;
    }

    status = c_Spark_ValidateParameters(handle, parameters);
    if (status != c_REVLibError_None) return status;

    status = c_Spark_PreProcessParameters(handle, parameters);
    if (status != c_REVLibError_None) return status;

    // Restore defaults if specified by user
    if (resetSafeParameters) {
        status = c_Spark_ResetSafeParameters(handle, false);
        if (status != c_REVLibError_None) return status;
    }

    // Iterate through parameters and write each one
    for (const auto& [key, value] : parameters) {
        status = c_Spark_SetParameterCore(handle, key,
                                          c_Spark_GetParameterType(key), value);

        // Only return if param-specific error return by device
        if (status == c_REVLibError_ParamInvalidID ||
            status == c_REVLibError_ParamMismatchType ||
            status == c_REVLibError_ParamAccessMode ||
            status == c_REVLibError_ParamInvalid ||
            status == c_REVLibError_ParamNotImplementedDeprecated) {
            c_REVLib_FlushErrors();
            return status;
        }
    }

    status =
        c_Spark_PostProcessParameters(handle, parameters, resetSafeParameters);
    if (status != c_REVLibError_None) return status;

    // Burn flash if specified by user
    if (persistParameters) {
        status = c_Spark_PersistParameters(handle);
        if (status != c_REVLibError_None) return status;
    }

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_StartFollowerMode(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_StartFollowerMode(handle->m_simDevice);

        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedDataIn{};

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, zeroLengthDataPacket, SPARK_START_FOLLOWER_MODE_FRAME_ID,
        packedDataIn, SPARK_START_FOLLOWER_MODE_RESPONSE_FRAME_ID,
        SPARK_START_FOLLOWER_MODE_RESPONSE_LENGTH, "Start Follower Mode");

    if (status != c_REVLibError_None) {
        return status;
    }

    // Check response status
    spark_start_follower_mode_response_t frameIn;
    spark_start_follower_mode_response_unpack(
        &frameIn, packedDataIn.data(),
        SPARK_START_FOLLOWER_MODE_RESPONSE_LENGTH);
    if (frameIn.status != 0) {
        REVLib_SendErrorText(handle, c_REVLibError_FollowConfigMismatch,
                             fmt::format("Start Follower Mode"));
        REVLib_SetLastError(handle, c_REVLibError_FollowConfigMismatch);
        return c_REVLibError_FollowConfigMismatch;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_StopFollowerMode(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_StopFollowerMode(handle->m_simDevice);

        return c_REVLibError_None;
    }

    std::array<uint8_t, kMaxPacketLength> packedDataIn;

    c_REVLib_ErrorCode status = REVLib_WriteAndReadCANPacket(
        handle, zeroLengthDataPacket, SPARK_STOP_FOLLOWER_MODE_FRAME_ID,
        packedDataIn, SPARK_STOP_FOLLOWER_MODE_RESPONSE_FRAME_ID,
        SPARK_STOP_FOLLOWER_MODE_RESPONSE_LENGTH, "Stop Follower Mode");

    if (status != c_REVLibError_None) {
        return status;
    }

    REVLib_SetLastError(handle, c_REVLibError_None);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_GetSimPIDOutput(c_Spark_handle handle,
                                               float* value, float setpoint,
                                               float pv, float dt) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        feedforward_constants_t ffConstants =
            c_SIM_Spark_GetFeedforwardConstants(handle->m_simDevice);
        feedforward_state_t ffState;
        feedforward_signals_t ffSignals;
        float feedforward = 0;

        switch (c_SIM_Spark_GetControlMode(handle->m_simDevice)) {
            case CTRL_VELOCITY:
                ffState.velocity = setpoint;
                ffState.acceleration = 0;
                ffState.position = 0;
                ffSignals = {1, 1, 0, 0, 0};  // s, v; no a, g, cos
                feedforward = c_SIM_Spark_CalculateFeedforward(
                    handle->m_simDevice, &ffConstants, ffState, ffSignals,
                    c_SIM_Spark_GetBusVoltage(handle->m_simDevice));
                break;
            case CTRL_POSITION:
                ffState.velocity = setpoint - pv;
                ffState.acceleration = 0;
                ffState.position = setpoint;
                ffSignals = {1, 0, 0, 1, 1};  // s, g, cos; no v, a
                feedforward = c_SIM_Spark_CalculateFeedforward(
                    handle->m_simDevice, &ffConstants, ffState, ffSignals,
                    c_SIM_Spark_GetBusVoltage(handle->m_simDevice));
                break;
            default:
                break;
        }

        *value = c_SIM_Spark_CalculatePID(handle->m_simDevice, setpoint, pv, dt,
                                          feedforward);
    }
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_GetSimMAXMotionPositionControlOutput(
    c_Spark_handle handle, float* value, float dt) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        *value = c_SIM_Spark_SimulateMaxMotionPositionControl(
            handle->m_simDevice, dt);
    }
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_GetSimMAXMotionVelocityControlOutput(
    c_Spark_handle handle, float* value, float dt) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        *value = c_SIM_Spark_SimulateMaxMotionVelocityControl(
            handle->m_simDevice, dt);
    }
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_GetSimCurrentLimitOutput(c_Spark_handle handle,
                                                        float* value,
                                                        float appliedOutput,
                                                        float current) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        *value = c_SIM_Spark_SimulateCurrentLimit(handle->m_simDevice,
                                                  appliedOutput, current);
    }
    return c_REVLibError_None;
}

void c_SIM_Spark_CreateSimExtOrAltEncoder(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle->m_simDevice);
    }
}
void c_SIM_Spark_CreateSimAbsoluteEncoder(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle->m_simDevice);
    }
}
void c_SIM_Spark_CreateSimAnalogSensor(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimAnalogSensor(handle->m_simDevice);
    }
}
void c_SIM_Spark_CreateSimForwardLimitSwitch(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimForwardLimitSwitch(handle->m_simDevice);
    }
}
void c_SIM_Spark_CreateSimReverseLimitSwitch(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimReverseLimitSwitch(handle->m_simDevice);
    }
}

void c_SIM_Spark_CreateSimFaultManager(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimFaultManager(handle->m_simDevice);
    }
}
void c_SIM_Spark_CreateSimRelativeEncoder(c_Spark_handle handle) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle->m_simDevice);
    }
}

c_REVLib_ErrorCode c_Spark_ConfigureAsync(c_Spark_handle handle,
                                          const char* flattenedConfig,
                                          uint8_t resetSafeParameters,
                                          uint8_t persistParameters) {
    if (c_SIM_Spark_IsSim(handle->m_simDevice)) {
        c_Spark_Configure(handle, flattenedConfig, resetSafeParameters,
                          persistParameters);
    } else {
        // Convert the C-string to a std::string so the lambda will copy
        // the actual string. Otherwise, it just copies the pointer, which
        // may not exist by the time the async call triggers.
        const std::string configCopy{flattenedConfig};
        c_REVLib_RegisterAsyncCall(
            [handle, configCopy, resetSafeParameters, persistParameters]() {
                c_Spark_Configure(handle, configCopy.c_str(),
                                  resetSafeParameters, persistParameters);
            });
    }
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_StartFollowerModeAsync(c_Spark_handle handle) {
    c_REVLib_RegisterAsyncCall([=]() { c_Spark_StartFollowerMode(handle); });
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_Spark_StopFollowerModeAsync(c_Spark_handle handle) {
    c_REVLib_RegisterAsyncCall([=]() { c_Spark_StopFollowerMode(handle); });
    return c_REVLibError_None;
}
