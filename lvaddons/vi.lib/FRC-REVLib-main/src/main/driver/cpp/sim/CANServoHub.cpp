/*
 * Copyright (c) 2024-2026 REV Robotics
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

#include "rev/sim/CANServoHub.h"

#include <rev/CANServoHubDriver.h>
#include <rev/CANServoHubFrames.h>
#include <rev/CANServoHubParameters.h>
#include <rev/REVLibValue.h>
#include <rev/driver/REVLibDriver.h>

#include <array>
#include <string>

#include <fmt/format.h>

// Build with -D to disable SIM device
// #define REV_CAN_SERVOHUB_NO_SIM

namespace {

enum SIM_ServoHub_ServoHubSignals {
    kSIM_ServoHub_ServoHubSignal_voltage,
    kSIM_ServoHub_ServoHubSignal_servoVoltage,
    kSIM_ServoHub_ServoHubSignal_deviceCurrent,
    kSIM_ServoHub_ServoHubSignal_channel0PulseWidth,
    kSIM_ServoHub_ServoHubSignal_channel1PulseWidth,
    kSIM_ServoHub_ServoHubSignal_channel2PulseWidth,
    kSIM_ServoHub_ServoHubSignal_channel3PulseWidth,
    kSIM_ServoHub_ServoHubSignal_channel4PulseWidth,
    kSIM_ServoHub_ServoHubSignal_channel5PulseWidth,
    kSIM_ServoHub_ServoHubSignal_channel0Enabled,
    kSIM_ServoHub_ServoHubSignal_channel1Enabled,
    kSIM_ServoHub_ServoHubSignal_channel2Enabled,
    kSIM_ServoHub_ServoHubSignal_channel3Enabled,
    kSIM_ServoHub_ServoHubSignal_channel4Enabled,
    kSIM_ServoHub_ServoHubSignal_channel5Enabled,
    kSIM_ServoHub_ServoHubSignal_channel0Powered,
    kSIM_ServoHub_ServoHubSignal_channel1Powered,
    kSIM_ServoHub_ServoHubSignal_channel2Powered,
    kSIM_ServoHub_ServoHubSignal_channel3Powered,
    kSIM_ServoHub_ServoHubSignal_channel4Powered,
    kSIM_ServoHub_ServoHubSignal_channel5Powered,
    kSIM_ServoHub_ServoHubSignal_bank02PulsePeriod,
    kSIM_ServoHub_ServoHubSignal_bank35PulsePeriod,
    kSIM_ServoHub_ServoHubSignal_NumSignals
};

struct SIM_ServoHub_ServoHubSignals_TableDef {
    const char* name;
    bool readOnly;
    REVLibType type;
    double initialValue;
};

const std::array<SIM_ServoHub_ServoHubSignals_TableDef,
                 kSIM_ServoHub_ServoHubSignal_NumSignals>
    s_SIM_ServoHub_ServoHubSignals_Table{{
        {"Voltage", true, TYPE_DOUBLE, 12.0},
        {"Servo Voltage", true, TYPE_DOUBLE, 7.0},
        {"Device Current", true, TYPE_DOUBLE, 0.25},
        {"Channel 0 Pulse Width", true, TYPE_INT, 1500},
        {"Channel 1 Pulse Width", true, TYPE_INT, 1500},
        {"Channel 2 Pulse Width", true, TYPE_INT, 1500},
        {"Channel 3 Pulse Width", true, TYPE_INT, 1500},
        {"Channel 4 Pulse Width", true, TYPE_INT, 1500},
        {"Channel 5 Pulse Width", true, TYPE_INT, 1500},
        {"Channel 0 Enabled", true, TYPE_BOOLEAN, false},
        {"Channel 1 Enabled", true, TYPE_BOOLEAN, false},
        {"Channel 2 Enabled", true, TYPE_BOOLEAN, false},
        {"Channel 3 Enabled", true, TYPE_BOOLEAN, false},
        {"Channel 4 Enabled", true, TYPE_BOOLEAN, false},
        {"Channel 5 Enabled", true, TYPE_BOOLEAN, false},
        {"Channel 0 Powered", true, TYPE_BOOLEAN, false},
        {"Channel 1 Powered", true, TYPE_BOOLEAN, false},
        {"Channel 2 Powered", true, TYPE_BOOLEAN, false},
        {"Channel 3 Powered", true, TYPE_BOOLEAN, false},
        {"Channel 4 Powered", true, TYPE_BOOLEAN, false},
        {"Channel 5 Powered", true, TYPE_BOOLEAN, false},
        {"Bank 0-2 Pulse Period", true, TYPE_INT, 5000},
        {"Bank 3-5 Pulse Period", true, TYPE_INT, 5000},
    }};

REVLibValue SIM_ServoHub_CreateHALValue(double v, REVLibType type) {
    REVLibValue result;
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

enum SIM_ServoHub_FaultManagerSignals {
    kSIM_ServoHub_FaultManagerSignal_regulatorPowerGoodFault,
    kSIM_ServoHub_FaultManagerSignal_brownout,
    kSIM_ServoHub_FaultManagerSignal_canWarning,
    kSIM_ServoHub_FaultManagerSignal_canBusOff,
    kSIM_ServoHub_FaultManagerSignal_hardwareFault,
    kSIM_ServoHub_FaultManagerSignal_firmwareFault,
    kSIM_ServoHub_FaultManagerSignal_hasReset,
    kSIM_ServoHub_FaultManagerSignal_channel0Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_channel1Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_channel2Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_channel3Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_channel4Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_channel5Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_lowBatteryFault,
    kSIM_ServoHub_FaultManagerSignal_stickyRegulatorPowerGoodFault,
    kSIM_ServoHub_FaultManagerSignal_stickyBrownout,
    kSIM_ServoHub_FaultManagerSignal_stickyCanWarning,
    kSIM_ServoHub_FaultManagerSignal_stickyCanBusOff,
    kSIM_ServoHub_FaultManagerSignal_stickyHardwareFault,
    kSIM_ServoHub_FaultManagerSignal_stickyFirmwareFault,
    kSIM_ServoHub_FaultManagerSignal_stickyHasReset,
    kSIM_ServoHub_FaultManagerSignal_stickyChannel0Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_stickyChannel1Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_stickyChannel2Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_stickyChannel3Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_stickyChannel4Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_stickyChannel5Overcurrent,
    kSIM_ServoHub_FaultManagerSignal_stickyLowBatteryFault,
    kSIM_ServoHub_FaultManagerSignal_NumSignals
};

const std::array<SIM_ServoHub_ServoHubSignals_TableDef,
                 kSIM_ServoHub_FaultManagerSignal_NumSignals>
    s_SIM_ServoHub_FaultManagerSignals_Table{
        {{"Regulator Power Good Fault", false, TYPE_BOOLEAN, 0.0},
         {"Brownout", false, TYPE_BOOLEAN, 0.0},
         {"CAN Warning", false, TYPE_BOOLEAN, 0.0},
         {"CAN Bus Off", false, TYPE_BOOLEAN, 0.0},
         {"Hardware Fault", false, TYPE_BOOLEAN, 0.0},
         {"Firmware Fault", false, TYPE_BOOLEAN, 0.0},
         {"Has Reset", false, TYPE_BOOLEAN, 0.0},
         {"Channel 0 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Channel 1 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Channel 2 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Channel 3 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Channel 4 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Channel 5 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Low Battery Fault", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Regulator Power Good Fault", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Brownout", false, TYPE_BOOLEAN, 0.0},
         {"Sticky CAN Warning", false, TYPE_BOOLEAN, 0.0},
         {"Sticky CAN Bus Off", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Hardware Fault", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Firmware Fault", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Has Reset", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Channel 0 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Channel 1 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Channel 2 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Channel 3 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Channel 4 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Channel 5 Over Current", false, TYPE_BOOLEAN, 0.0},
         {"Sticky Low Battery Fault", false, TYPE_BOOLEAN, 0.0}}};

}  // namespace

struct c_SIM_ServoHub_FaultManager_Obj {
    int32_t m_simDevice;

    int32_t m_signals
        [kSIM_ServoHub_FaultManagerSignal_NumSignals];  // NOLINT(runtime/arrays)

    explicit c_SIM_ServoHub_FaultManager_Obj(int32_t simDevice)
        : m_simDevice(simDevice) {
        for (int i = 0; i < kSIM_ServoHub_FaultManagerSignal_NumSignals; ++i) {
            REVLibValue tmp = SIM_ServoHub_CreateHALValue(
                s_SIM_ServoHub_FaultManagerSignals_Table[i].initialValue,
                s_SIM_ServoHub_FaultManagerSignals_Table[i].type);
            m_signals[i] = getREVLibDriver()->createSimValue(
                simDevice, s_SIM_ServoHub_FaultManagerSignals_Table[i].name,
                s_SIM_ServoHub_FaultManagerSignals_Table[i].readOnly, tmp);
        }
    }

    c_SIM_ServoHub_FaultManager_Obj() = delete;
};

namespace {

struct SIM_ServoHub_InternalSignals {
    int busId;
    int deviceId;
    std::string deviceName;

    c_SIM_ServoHub_FaultManager_handle m_simFaultManager{nullptr};

    ~SIM_ServoHub_InternalSignals() { delete m_simFaultManager; }
};

}  // namespace

struct c_SIM_ServoHub_Obj {
    int32_t m_simDevice;
    uint32_t vParameterTable[c_ServoHub_kNumConfigParameters]{};
    int32_t m_signals[kSIM_ServoHub_ServoHubSignal_NumSignals];

    SIM_ServoHub_InternalSignals m_internalSignals{};

    explicit c_SIM_ServoHub_Obj(int32_t simDevice) {
        for (int i = 0; i < kSIM_ServoHub_ServoHubSignal_NumSignals; ++i) {
            REVLibValue tmp = SIM_ServoHub_CreateHALValue(
                s_SIM_ServoHub_ServoHubSignals_Table[i].initialValue,
                s_SIM_ServoHub_ServoHubSignals_Table[i].type);
            m_signals[i] = getREVLibDriver()->createSimValue(
                simDevice, s_SIM_ServoHub_ServoHubSignals_Table[i].name,
                s_SIM_ServoHub_ServoHubSignals_Table[i].readOnly, tmp);
        }
    }

    c_SIM_ServoHub_Obj() = delete;
};

namespace {

void SIM_ServoHub_SetServoHubSignalValue(c_SIM_ServoHub_handle handle,
                                         SIM_ServoHub_ServoHubSignals signal,
                                         double v) {
    REVLibValue hval;
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

void SIM_ServoHub_InitializeParameterTable(uint32_t* vParameterTable) {
    std::memset(vParameterTable, 0,
                sizeof(uint32_t) * c_ServoHub_kNumConfigParameters);

    for (size_t i = 0u; i < c_ServoHub_kNumConfigParameters; ++i) {
        vParameterTable[i] = s_ServoHub_ParameterTable[i].defaultValue;
    }
}

c_REVLib_ErrorCode SIM_ServoHub_GetParameter(
    c_SIM_ServoHub_handle handle, c_ServoHub_ConfigParameter parameter,
    uint32_t* value) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((parameter < 0) ||
        (parameter >= static_cast<int>(c_ServoHub_kNumConfigParameters))) {
        return c_REVLibError_ParamInvalidID;
    }
    *value = handle->vParameterTable[parameter];
    return c_REVLibError_None;
}

}  // namespace

c_SIM_ServoHub_handle c_SIM_ServoHub_Create(int busId, int deviceId) {
#ifdef REV_CAN_SERVOHUB_NO_SIM
    return nullptr;
#endif

    const std::string deviceName{
        fmt::format("Servo Hub [{},{}]", busId, deviceId)};

    int32_t simHandle = getREVLibDriver()->createSimDevice(deviceName.c_str());
    if (simHandle == REVLIB_INVALID_HANDLE) {
        return nullptr;
    }

    c_SIM_ServoHub_handle handle = new struct c_SIM_ServoHub_Obj(simHandle);

    if (handle == nullptr) {
        getREVLibDriver()->freeSimDevice(simHandle);
        return nullptr;
    }

    handle->m_simDevice = simHandle;

    SIM_ServoHub_InitializeParameterTable(handle->vParameterTable);

    handle->m_internalSignals.busId = busId;
    handle->m_internalSignals.deviceId = deviceId;
    handle->m_internalSignals.deviceName = deviceName;

    return handle;
}

void c_SIM_ServoHub_Destroy(c_SIM_ServoHub_handle handle) {
    if (handle == nullptr) {
        return;
    }

    getREVLibDriver()->freeSimDevice(
        c_SIM_ServoHub_GetOrCreateSimFaultManager(handle)->m_simDevice);

    getREVLibDriver()->freeSimDevice(handle->m_simDevice);

    delete handle;
}

bool c_SIM_ServoHub_IsSim(c_SIM_ServoHub_handle handle) {
    if (handle == nullptr) {
        return false;
    }

    return (handle->m_simDevice != REVLIB_INVALID_HANDLE);
}

c_REVLib_ErrorCode c_SIM_ServoHub_ResetParameters(
    c_SIM_ServoHub_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    SIM_ServoHub_InitializeParameterTable(handle->vParameterTable);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_ServoHub_SetParameter(
    c_SIM_ServoHub_handle handle, c_ServoHub_ConfigParameter parameter,
    uint32_t value) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((parameter < 0) || (parameter >= c_ServoHub_kNumConfigParameters)) {
        return c_REVLibError_ParamInvalidID;
    }

    handle->vParameterTable[parameter] = value;
    return c_REVLibError_None;
}

// New sim functions, telemetry 'get' functions only, SimDevice Values
// handle setting

float c_SIM_ServoHub_GetDeviceVoltage(c_SIM_ServoHub_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_ServoHubSignal_voltage], &value);
    return static_cast<float>(value.data.d);
}

float c_SIM_ServoHub_GetServoVoltage(c_SIM_ServoHub_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_ServoHubSignal_servoVoltage], &value);
    return static_cast<float>(value.data.d);
}

float c_SIM_ServoHub_GetDeviceCurrent(c_SIM_ServoHub_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_ServoHubSignal_deviceCurrent], &value);
    return static_cast<float>(value.data.d);
}

int c_SIM_ServoHub_GetChannelPulseWidth(c_SIM_ServoHub_handle handle,
                                        c_ServoHub_Channel channel) {
    if (handle == nullptr) {
        return 0;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return 0;
    }

    SIM_ServoHub_ServoHubSignals signal =
        static_cast<SIM_ServoHub_ServoHubSignals>(
            static_cast<int>(kSIM_ServoHub_ServoHubSignal_channel0PulseWidth) +
            channel);

    struct REVLibValue pulseWidth_us;
    getREVLibDriver()->getSimValue(handle->m_signals[signal], &pulseWidth_us);
    return pulseWidth_us.data.i;
}

c_REVLib_ErrorCode c_SIM_ServoHub_SetChannelPulseWidth(
    c_SIM_ServoHub_handle handle, c_ServoHub_Channel channel,
    int pulseWidth_us) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return c_REVLibError_ParamInvalidChannel;
    }

    SIM_ServoHub_ServoHubSignals signal =
        static_cast<SIM_ServoHub_ServoHubSignals>(
            static_cast<int>(kSIM_ServoHub_ServoHubSignal_channel0PulseWidth) +
            channel);

    SIM_ServoHub_SetServoHubSignalValue(handle, signal, pulseWidth_us);

    return c_REVLibError_None;
}

bool c_SIM_ServoHub_GetChannelEnabled(c_SIM_ServoHub_handle handle,
                                      c_ServoHub_Channel channel) {
    if (handle == nullptr) {
        return 0;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return 0;
    }

    SIM_ServoHub_ServoHubSignals signal =
        static_cast<SIM_ServoHub_ServoHubSignals>(
            static_cast<int>(kSIM_ServoHub_ServoHubSignal_channel0Enabled) +
            channel);

    struct REVLibValue enabled;
    getREVLibDriver()->getSimValue(handle->m_signals[signal], &enabled);
    return static_cast<bool>(enabled.data.b);
}

c_REVLib_ErrorCode c_SIM_ServoHub_SetChannelEnabled(
    c_SIM_ServoHub_handle handle, c_ServoHub_Channel channel, bool enabled) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return c_REVLibError_ParamInvalidChannel;
    }

    SIM_ServoHub_ServoHubSignals signal =
        static_cast<SIM_ServoHub_ServoHubSignals>(
            static_cast<int>(kSIM_ServoHub_ServoHubSignal_channel0Enabled) +
            channel);

    SIM_ServoHub_SetServoHubSignalValue(handle, signal, enabled);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_ServoHub_SetChannelPowered(
    c_SIM_ServoHub_handle handle, c_ServoHub_Channel channel, bool powered) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return c_REVLibError_ParamInvalidChannel;
    }

    SIM_ServoHub_ServoHubSignals signal =
        static_cast<SIM_ServoHub_ServoHubSignals>(
            static_cast<int>(kSIM_ServoHub_ServoHubSignal_channel0Powered) +
            channel);

    SIM_ServoHub_SetServoHubSignalValue(handle, signal, powered);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_ServoHub_SetBankPulsePeriod(
    c_SIM_ServoHub_handle handle, c_ServoHub_Bank bank, int pulsePeriod_us) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((bank < c_ServoHub_kBank0_2) || (bank > c_ServoHub_kBank3_5)) {
        return c_REVLibError_ParamInvalidChannel;
    }

    SIM_ServoHub_ServoHubSignals signal =
        static_cast<SIM_ServoHub_ServoHubSignals>(
            static_cast<int>(kSIM_ServoHub_ServoHubSignal_bank02PulsePeriod) +
            bank);

    SIM_ServoHub_SetServoHubSignalValue(handle, signal, pulsePeriod_us);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_ServoHub_GetChannelPulseRange(
    c_SIM_ServoHub_handle handle, c_ServoHub_Channel channel,
    c_ServoHub_ChannelPulseRange* pulseRange) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return c_REVLibError_ParamInvalidChannel;
    }

    static constexpr int kPulseRangeOffset{3};
    const c_ServoHub_ConfigParameter minParam =
        static_cast<c_ServoHub_ConfigParameter>(
            (channel * kPulseRangeOffset) + c_ServoHub_kChannel0_MinPulseWidth);
    const c_ServoHub_ConfigParameter centerParam =
        static_cast<c_ServoHub_ConfigParameter>(minParam + 1);
    const c_ServoHub_ConfigParameter maxParam =
        static_cast<c_ServoHub_ConfigParameter>(centerParam + 1);

    uint32_t value;
    SIM_ServoHub_GetParameter(handle, minParam, &value);
    pulseRange->minPulse_us = static_cast<uint16_t>(value);
    SIM_ServoHub_GetParameter(handle, centerParam, &value);
    pulseRange->centerPulse_us = static_cast<uint16_t>(value);
    SIM_ServoHub_GetParameter(handle, maxParam, &value);
    pulseRange->maxPulse_us = static_cast<uint16_t>(value);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_ServoHub_GetChannelDisableBehavior(
    c_SIM_ServoHub_handle handle, c_ServoHub_Channel channel,
    bool* disableBehavior) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if ((channel < c_ServoHub_kChannel0) || (channel > c_ServoHub_kChannel5)) {
        return c_REVLibError_ParamInvalidChannel;
    }

    const c_ServoHub_ConfigParameter param =
        static_cast<c_ServoHub_ConfigParameter>(
            static_cast<int>(c_ServoHub_kChannel0_DisableBehavior) + channel);

    uint32_t value;
    SIM_ServoHub_GetParameter(handle, param, &value);
    *disableBehavior = static_cast<bool>(value);

    return c_REVLibError_None;
}

c_SIM_ServoHub_FaultManager_handle c_SIM_ServoHub_GetOrCreateSimFaultManager(
    c_SIM_ServoHub_handle handle) {
    if (!c_SIM_ServoHub_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simFaultManager == NULL) {
        std::string deviceName =
            handle->m_internalSignals.deviceName + " FAULT MANAGER";
        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simFaultManager =
            new c_SIM_ServoHub_FaultManager_Obj(simHandle);
    }
    return handle->m_internalSignals.m_simFaultManager;
}

// Fault Manager

void c_SIM_ServoHub_SetRawFrameFromSimFaults(
    c_SIM_ServoHub_FaultManager_handle handle,
    c_ServoHub_PeriodicStatus1* rawFrame) {
    if (handle == nullptr) return;
    struct REVLibValue hval;

    // man, this is ugly
    // I would try to setup a loop but to ensure things stay matched up this is
    // probably best
    //
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_regulatorPowerGoodFault],
        &hval);
    rawFrame->regulatorPowerGoodFault = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_brownout], &hval);
    rawFrame->brownout = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_canWarning], &hval);
    rawFrame->canWarning = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_canBusOff], &hval);
    rawFrame->canBusOff = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_hardwareFault],
        &hval);
    rawFrame->hardwareFault = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_firmwareFault],
        &hval);
    rawFrame->firmwareFault = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_hasReset], &hval);
    rawFrame->hasReset = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_channel0Overcurrent],
        &hval);
    rawFrame->channel0Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_channel1Overcurrent],
        &hval);
    rawFrame->channel1Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_channel2Overcurrent],
        &hval);
    rawFrame->channel2Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_channel3Overcurrent],
        &hval);
    rawFrame->channel3Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_channel4Overcurrent],
        &hval);
    rawFrame->channel4Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_channel5Overcurrent],
        &hval);
    rawFrame->channel5Overcurrent = static_cast<bool>(hval.data.b);

    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyRegulatorPowerGoodFault],
        &hval);
    rawFrame->stickyRegulatorPowerGoodFault = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_stickyBrownout],
        &hval);
    rawFrame->stickyBrownout = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_stickyCanWarning],
        &hval);
    rawFrame->stickyCanWarning = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_stickyCanBusOff],
        &hval);
    rawFrame->stickyCanBusOff = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_stickyHardwareFault],
        &hval);
    rawFrame->stickyHardwareFault = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_stickyFirmwareFault],
        &hval);
    rawFrame->stickyFirmwareFault = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals[kSIM_ServoHub_FaultManagerSignal_stickyHasReset],
        &hval);
    rawFrame->stickyHasReset = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyChannel0Overcurrent],
        &hval);
    rawFrame->stickyChannel0Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyChannel1Overcurrent],
        &hval);
    rawFrame->stickyChannel1Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyChannel2Overcurrent],
        &hval);
    rawFrame->stickyChannel2Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyChannel3Overcurrent],
        &hval);
    rawFrame->stickyChannel3Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyChannel4Overcurrent],
        &hval);
    rawFrame->stickyChannel4Overcurrent = static_cast<bool>(hval.data.b);
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [kSIM_ServoHub_FaultManagerSignal_stickyChannel5Overcurrent],
        &hval);
    rawFrame->stickyChannel5Overcurrent = static_cast<bool>(hval.data.b);
}

bool c_SIM_ServoHub_GetSimFaultManagerExists(c_SIM_ServoHub_handle handle) {
    return handle->m_internalSignals.m_simFaultManager != nullptr;
}

void c_SIM_ServoHub_ClearSimFaults(c_SIM_ServoHub_FaultManager_handle handle) {
    for (int i = 0; i < kSIM_ServoHub_FaultManagerSignal_NumSignals; ++i) {
        REVLibValue tmp = SIM_ServoHub_CreateHALValue(
            s_SIM_ServoHub_FaultManagerSignals_Table[i].initialValue,
            s_SIM_ServoHub_FaultManagerSignals_Table[i].type);
        getREVLibDriver()->setSimValue(handle->m_signals[i], &tmp);
    }
}
