/*
 * Copyright (c) 2025-2026 REV Robotics
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

#include "rev/sim/CANDetachedEncoder.h"

#include <rev/CANDetachedEncoderDriver.h>
#include <rev/CANDetachedEncoderFrames.h>
#include <rev/REVLibValue.h>
#include <rev/REVUtils.h>
#include <rev/driver/REVLibDriver.h>

#include <array>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <fmt/format.h>

// Build with -D to disable SIM device
// #define REV_CAN_DETACHEDENCODER_NO_SIM

namespace {

enum SIM_Detached_Signals {
    kSIM_Detached_Signal_position,
    kSIM_Detached_Signal_velocity,
    kSIM_Detached_Signal_angle,
    kSIM_Detached_Signal_rawAngle,
    kSIM_Detached_Signal_zeroOffset,
    kSIM_Detached_Signal_isInverted,
    kSIM_Detached_Signal_positionFactor,
    kSIM_Detached_Signal_velocityFactor,
    kSIM_Detached_Signal_NExternalSignals
};

struct SIM_Detached_Signals_TableDef {
    const char* name;
    bool readOnly;
    REVLibType type;
    double initialValue;
};

constexpr SIM_Detached_Signals_TableDef
    c_SIM_Detached_Signals_Table[kSIM_Detached_Signal_NExternalSignals] = {
        {"Position", false, TYPE_DOUBLE, 0.0},
        {"Velocity", false, TYPE_DOUBLE, 0.0},
        {"Angle", false, TYPE_DOUBLE, 0.0},
        {"Raw Angle", false, TYPE_DOUBLE, 0.0},
        {"Zero Offset", true, TYPE_DOUBLE, 0.0},
        {"Is Inverted", true, TYPE_BOOLEAN, 0.0},
        {"Position Conversion Factor", true, TYPE_DOUBLE, 1.0},
        {"Velocity Conversion Factor", true, TYPE_DOUBLE, 1.0},
};

REVLibValue SIM_Detached_CreateHALValue(double v, enum REVLibType type) {
    struct REVLibValue result;
    result.type = type;

    switch (type) {
        case TYPE_BOOLEAN:
            result.data.b = static_cast<bool>(v);
            break;
        case TYPE_DOUBLE:
            result.data.d = v;
            break;
        case TYPE_ENUM:
            result.data.e = static_cast<int32_t>(v);
            break;
        case TYPE_INT:
            result.data.i = static_cast<int32_t>(v);
            break;
        case TYPE_LONG:
            result.data.l = static_cast<int64_t>(v);
            break;
        default:
            result.data.i = static_cast<int32_t>(v);
            break;
    }

    return result;
}

struct SIM_Detached_Base_Obj {
    int32_t m_simDevice;

    std::vector<int32_t> m_signals;

    using Signals_TableDef = std::span<const SIM_Detached_Signals_TableDef>;

    void close(void) { getREVLibDriver()->freeSimDevice(m_simDevice); }

protected:
    SIM_Detached_Base_Obj(int32_t simDevice, Signals_TableDef tableDef)
        : m_simDevice{simDevice} {
        for (const auto& def : tableDef) {
            const REVLibValue tmp =
                SIM_Detached_CreateHALValue(def.initialValue, def.type);
            m_signals.push_back(getREVLibDriver()->createSimValue(
                simDevice, def.name, def.readOnly, tmp));
        }
    }

    virtual ~SIM_Detached_Base_Obj() = default;
};

enum SIM_Detached_FaultManagerSignal {
    kSIM_Detached_FaultManagerSignal_unexpectedFault,
    kSIM_Detached_FaultManagerSignal_hasResetWarning,
    kSIM_Detached_FaultManagerSignal_canTxFault,
    kSIM_Detached_FaultManagerSignal_canRxFault,
    kSIM_Detached_FaultManagerSignal_eepromFault,
    kSIM_Detached_FaultManagerSignal_unexpectedStickyFault,
    kSIM_Detached_FaultManagerSignal_hasResetStickyWarning,
    kSIM_Detached_FaultManagerSignal_canTxStickyFault,
    kSIM_Detached_FaultManagerSignal_canRxStickyFault,
    kSIM_Detached_FaultManagerSignal_eepromStickyFault,
    kSIM_Detached_FaultManagerSignal_NExternalSignals
};

static constexpr SIM_Detached_Signals_TableDef
    SIM_Detached_FaultManagerSignals_Table
        [kSIM_Detached_FaultManagerSignal_NExternalSignals] = {
            {"Unexpected Fault", false, TYPE_BOOLEAN, 0.0},
            {"Has Reset Warning", false, TYPE_BOOLEAN, 0.0},
            {"CAN Transmit Fault", false, TYPE_BOOLEAN, 0.0},
            {"CAN Receive Fault", false, TYPE_BOOLEAN, 0.0},
            {"Eeprom Fault", false, TYPE_BOOLEAN, 0.0},
            {"Unexpected Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Has Reset Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"CAN Transmit Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"CAN Receive Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Eeprom Sticky Fault", false, TYPE_BOOLEAN, 0.0},
};

}  // namespace

struct c_SIM_Detached_FaultManager_Obj : public SIM_Detached_Base_Obj {
    explicit c_SIM_Detached_FaultManager_Obj(int32_t simDevice)
        : SIM_Detached_Base_Obj(simDevice,
                                SIM_Detached_FaultManagerSignals_Table) {}
};

namespace {

struct SIM_Detached_InternalSignals {
    int busId;
    int deviceId;
    std::string deviceName;

    uint8_t encoderModel;

    c_SIM_Detached_FaultManager_handle m_simFaultManager{nullptr};

    ~SIM_Detached_InternalSignals() { delete m_simFaultManager; }
};

}  // namespace

struct c_SIM_Detached_Obj : public SIM_Detached_Base_Obj {
    uint32_t
        vParameterTable[c_Detached_NumParameters];  // NOLINT(runtime/arrays)

    SIM_Detached_InternalSignals m_internalSignals{};

    explicit c_SIM_Detached_Obj(int32_t simDevice)
        : SIM_Detached_Base_Obj(simDevice, c_SIM_Detached_Signals_Table) {}
};

namespace {

static void SIM_Detached_SetSignalValue(c_SIM_Detached_handle handle,
                                        SIM_Detached_Signals signal, double v) {
    struct REVLibValue hval;
    getREVLibDriver()->getSimValue(handle->m_signals[signal], &hval);

    switch (hval.type) {
        case TYPE_BOOLEAN:
            hval.data.b = static_cast<bool>(v);
            break;
        case TYPE_DOUBLE:
            hval.data.d = v;
            break;
        case TYPE_ENUM:
            hval.data.e = static_cast<int32_t>(v);
            break;
        case TYPE_INT:
            hval.data.i = static_cast<int32_t>(v);
            break;
        case TYPE_LONG:
            hval.data.l = static_cast<int64_t>(v);
            break;
        default:
            hval.data.i = static_cast<int32_t>(v);
            break;
    }

    getREVLibDriver()->setSimValue(handle->m_signals[signal], &hval);
}

static void SIM_Detached_InitializeParameterTable(uint32_t* vParameterTable) {
    std::memset(vParameterTable, 0,
                sizeof(uint32_t) * c_Detached_NumParameters);

    for (uint8_t i = 0u; i < c_Detached_NumParameters; i++) {
        vParameterTable[i] = c_Detached_GetParameterDefaultValue(
            static_cast<c_Detached_ConfigParameter>(i));
    }
}

}  // namespace

c_SIM_Detached_handle c_SIM_Detached_Create(
    int busId, int deviceId, c_Detached_EncoderModel encoderModel) {
#ifdef REV_CAN_DETACHEDENCODER_NO_SIM
    return nullptr;
#endif

    auto deviceName = fmt::format("Detached {} [{},{}]",
                                  encoderModel == c_Detached_kMAXSplineEncoder
                                      ? "MAXSpline Encoder"
                                      : "UNKNOWN",
                                  busId, deviceId);

    int32_t simHandle = getREVLibDriver()->createSimDevice(deviceName.c_str());
    if (simHandle == REVLIB_INVALID_HANDLE) {
        return nullptr;
    }

    c_SIM_Detached_handle handle = new c_SIM_Detached_Obj(simHandle);

    if (handle == nullptr) {
        getREVLibDriver()->freeSimDevice(simHandle);
        return nullptr;
    }

    handle->m_simDevice = simHandle;

    SIM_Detached_InitializeParameterTable(handle->vParameterTable);

    handle->m_internalSignals.busId = busId;
    handle->m_internalSignals.deviceId = deviceId;
    handle->m_internalSignals.deviceName = deviceName;

    handle->m_internalSignals.encoderModel = encoderModel;

    return handle;
}

void c_SIM_Detached_Close(c_SIM_Detached_handle handle) { handle->close(); }

void c_SIM_Detached_Destroy(c_SIM_Detached_handle handle) { delete handle; }

bool c_SIM_Detached_IsSim(c_SIM_Detached_handle handle) {
    if (handle == nullptr) {
        return false;
    }

    return (handle->m_simDevice != REVLIB_INVALID_HANDLE);
}

c_REVLib_ErrorCode c_SIM_Detached_ResetParameters(
    c_SIM_Detached_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    SIM_Detached_InitializeParameterTable(handle->vParameterTable);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Detached_SetParameter(
    c_SIM_Detached_handle handle, c_Detached_ConfigParameter parameter,
    uint32_t value) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((parameter < 0) ||
        (static_cast<uint32_t>(parameter) >= c_Detached_NumParameters)) {
        return c_REVLibError_ParamInvalidID;
    }

    handle->vParameterTable[parameter] = value;
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Detached_GetParameter(
    c_SIM_Detached_handle handle, c_Detached_ConfigParameter parameter,
    uint32_t* value) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((parameter < 0) ||
        (static_cast<uint32_t>(parameter) >= c_Detached_NumParameters)) {
        return c_REVLibError_ParamInvalidID;
    }
    *value = handle->vParameterTable[parameter];
    return c_REVLibError_None;
}

uint8_t c_SIM_Detached_GetEncoderModel(c_SIM_Detached_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    return handle->m_internalSignals.encoderModel;
}

c_SIM_Detached_FaultManager_handle c_SIM_Detached_GetOrCreateSimFaultManager(
    c_SIM_Detached_handle handle) {
    if (!c_SIM_Detached_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simFaultManager == NULL) {
        auto deviceName = fmt::format("{} FAULT MANAGER",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simFaultManager =
            new c_SIM_Detached_FaultManager_Obj(simHandle);
    }
    return handle->m_internalSignals.m_simFaultManager;
}

// Relative-specific
void c_SIM_Detached_SetEncoderPosition(c_SIM_Detached_handle handle,
                                       double position) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_position,
                                position);
}
double c_SIM_Detached_GetEncoderPosition(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_position], &hval);
    return hval.data.d;
}

void c_SIM_Detached_SetEncoderVelocity(c_SIM_Detached_handle handle,
                                       double velocity) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_velocity,
                                velocity);
}
double c_SIM_Detached_GetEncoderVelocity(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_velocity], &hval);
    return hval.data.d;
}

// Absolute-specific
void c_SIM_Detached_SetEncoderAngle(c_SIM_Detached_handle handle,
                                    double angle) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_angle, angle);
}
double c_SIM_Detached_GetEncoderAngle(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_angle], &hval);
    return hval.data.d;
}

void c_SIM_Detached_SetEncoderRawAngle(c_SIM_Detached_handle handle,
                                       double rawAngle) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_rawAngle,
                                rawAngle);
}
double c_SIM_Detached_GetEncoderRawAngle(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_rawAngle], &hval);
    return hval.data.d;
}

void c_SIM_Detached_SetEncoderZeroOffset(c_SIM_Detached_handle handle,
                                         double offset) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_zeroOffset,
                                offset);
}
double c_SIM_Detached_GetEncoderZeroOffset(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_zeroOffset], &hval);
    return hval.data.d;
}

// Common
void c_SIM_Detached_SetEncoderInverted(c_SIM_Detached_handle handle,
                                       bool inverted) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_isInverted,
                                inverted);
}
bool c_SIM_Detached_GetEncoderInverted(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_isInverted], &hval);
    return static_cast<bool>(hval.data.b);
}

void c_SIM_Detached_SetEncoderPositionFactor(c_SIM_Detached_handle handle,
                                             double posFactor) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_positionFactor,
                                posFactor);
}
double c_SIM_Detached_GetEncoderPositionFactor(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_positionFactor], &hval);
    return hval.data.d;
}

void c_SIM_Detached_SetEncoderVelocityFactor(c_SIM_Detached_handle handle,
                                             double velFactor) {
    if (handle == nullptr) return;
    SIM_Detached_SetSignalValue(handle, kSIM_Detached_Signal_velocityFactor,
                                velFactor);
}
double c_SIM_Detached_GetEncoderVelocityFactor(c_SIM_Detached_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_Detached_Signal_velocityFactor], &hval);
    return hval.data.d;
}

// Fault Manager

void c_SIM_Detached_SetRawframeFromSimFaults(
    c_SIM_Detached_FaultManager_handle handle,
    c_Detached_PeriodicStatus1* rawframe) {
    if (handle == nullptr) return;
    struct REVLibValue hval;

    const std::array<std::tuple<SIM_Detached_FaultManagerSignal, uint8_t*>,
                     kSIM_Detached_FaultManagerSignal_NExternalSignals>
        signalToFrameMemberMap = {{
            std::make_tuple(kSIM_Detached_FaultManagerSignal_unexpectedFault,
                            &rawframe->unexpectedFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_hasResetWarning,
                            &rawframe->hasResetFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_canTxFault,
                            &rawframe->canTxFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_canRxFault,
                            &rawframe->canRxFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_eepromFault,
                            &rawframe->eepromFault),

            std::make_tuple(
                kSIM_Detached_FaultManagerSignal_unexpectedStickyFault,
                &rawframe->unexpectedStickyFault),
            std::make_tuple(
                kSIM_Detached_FaultManagerSignal_hasResetStickyWarning,
                &rawframe->hasResetStickyFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_canTxStickyFault,
                            &rawframe->canTxStickyFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_canRxStickyFault,
                            &rawframe->canRxStickyFault),
            std::make_tuple(kSIM_Detached_FaultManagerSignal_eepromStickyFault,
                            &rawframe->eepromStickyFault),
        }};

    for (const auto& [signal, member] : signalToFrameMemberMap) {
        getREVLibDriver()->getSimValue(handle->m_signals[signal], &hval);
        *member = static_cast<bool>(hval.data.b) ? 1 : 0;
    }
}

void c_SIM_Detached_ClearSimFaults(c_SIM_Detached_FaultManager_handle handle) {
    for (int i = 0; i < kSIM_Detached_FaultManagerSignal_NExternalSignals;
         i++) {
        struct REVLibValue tmp = SIM_Detached_CreateHALValue(
            SIM_Detached_FaultManagerSignals_Table[i].initialValue,
            SIM_Detached_FaultManagerSignals_Table[i].type);
        getREVLibDriver()->setSimValue(handle->m_signals[i], &tmp);
    }
}

bool c_SIM_Detached_GetSimFaultManagerExists(c_SIM_Detached_handle handle) {
    return handle->m_internalSignals.m_simFaultManager != nullptr;
}
