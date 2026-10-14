/*
 * Copyright (c) 2020-2026 REV Robotics
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

#include "rev/sim/CANSpark.h"

#include <rev/CANSparkDriver.h>
#include <rev/CANSparkFrames.h>
#include <rev/REVLibValue.h>
#include <rev/REVUtils.h>
#include <rev/driver/REVLibDriver.h>

#include <cmath>
#include <cstring>
#include <mutex>
#include <numbers>
#include <numeric>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include <fmt/format.h>

extern "C" {

// Build with -D to disable SIM device
// #define REV_CAN_SPARK_NO_SIM

#define _PARAM_TABLE_SIZE 256

enum c_SIM_Spark_SparkSignals {
    c_SIM_Spark_SparkSignal_setpoint,
    c_SIM_Spark_SparkSignal_position,
    c_SIM_Spark_SparkSignal_velocity,
    c_SIM_Spark_SparkSignal_appliedOutput,
    c_SIM_Spark_SparkSignal_arbFeedforward,
    c_SIM_Spark_SparkSignal_arbFFUnits,
    c_SIM_Spark_SparkSignal_controlMode,
    c_SIM_Spark_SparkSignal_ClosedLoopSlot,
    c_SIM_Spark_SparkSignal_motorTemp,
    c_SIM_Spark_SparkSignal_motorCurrent,
    c_SIM_Spark_SparkSignal_busVoltage,
    c_SIM_Spark_SparkSignal_followLeader,
    c_SIM_Spark_SparkSignal_NExternalSignals
};

typedef struct {
    const char* name;
    bool readOnly;
    REVLibType type;
    double initialValue;
} c_SIM_Spark_SparkSignals_TableDef;

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_SparkSignals_Table[c_SIM_Spark_SparkSignal_NExternalSignals] = {
        {"Setpoint", true, TYPE_DOUBLE, 0.0},
        {"Position", false, TYPE_DOUBLE, 0.0},
        {"Velocity", false, TYPE_DOUBLE, 0.0},
        {"Applied Output", true, TYPE_DOUBLE, 0.0},
        {"Arbitrary Feedforward", true, TYPE_DOUBLE, 0.0},
        {"ArbFF Units", true, TYPE_INT, 0},
        {"Control Mode", true, TYPE_INT, 0},
        {"Closed Loop Slot", true, TYPE_INT, 0},
        {"Motor Temperature", false, TYPE_INT, 25.0},
        {"Motor Current", true, TYPE_DOUBLE, 0.0},
        {"Bus Voltage", true, TYPE_DOUBLE, 12.0},
        {"Follow Leader", true, TYPE_BOOLEAN, 0.0}};

#if 0   // NOT Used - For documentation only
static const char* c_SIM_Spark_ControlModeNames[] = {
    "Duty Cycle", "Velocity",          "Voltage",           "Position",
    "Current",    "MAXMotionPosition", "MAXMotionVelocity", "None"};
#endif  // 0

static struct REVLibValue c_SIM_Spark_CreateHALValue(double v,
                                                     enum REVLibType type) {
    struct REVLibValue result {};
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

struct c_SIM_Spark_Base_Obj {
    int32_t m_simDevice;

    std::vector<int32_t> m_signals;

    using Signals_TableDef = std::span<const c_SIM_Spark_SparkSignals_TableDef>;

public:
    void close(void) { getREVLibDriver()->freeSimDevice(m_simDevice); }

protected:
    c_SIM_Spark_Base_Obj(int32_t simDevice, Signals_TableDef tableDef)
        : m_simDevice{simDevice} {
        for (const auto& def : tableDef) {
            const REVLibValue tmp =
                c_SIM_Spark_CreateHALValue(def.initialValue, def.type);
            m_signals.push_back(getREVLibDriver()->createSimValue(
                simDevice, def.name, def.readOnly, tmp));
        }
    }

    virtual ~c_SIM_Spark_Base_Obj() = default;
};

enum c_SIM_Spark_ExtOrAltEncoderSignals {
    c_SIM_Spark_ExtOrAltEncoderSignal_position,
    c_SIM_Spark_ExtOrAltEncoderSignal_velocity,
    c_SIM_Spark_ExtOrAltEncoderSignal_isInverted,
    c_SIM_Spark_ExtOrAltEncoderSignal_zeroOffset,
    c_SIM_Spark_ExtOrAltEncoderSignal_positionFactor,
    c_SIM_Spark_ExtOrAltEncoderSignal_velocityFactor,
    c_SIM_Spark_ExtOrAltEncoderSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_ExtOrAltEncoderSignals_Table
        [c_SIM_Spark_ExtOrAltEncoderSignal_NExternalSignals] = {
            {"Position", false, TYPE_DOUBLE, 0.0},
            {"Velocity", false, TYPE_DOUBLE, 0.0},
            {"Is Inverted", true, TYPE_BOOLEAN, 0.0},
            {"Zero Offset", true, TYPE_DOUBLE, 0.0},
            {"Position Conversion Factor", true, TYPE_DOUBLE, 1.0},
            {"Velocity Conversion Factor", true, TYPE_DOUBLE, 1.0},
};

struct c_SIM_Spark_ExtOrAltEncoder_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_ExtOrAltEncoder_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice,
                               c_SIM_Spark_ExtOrAltEncoderSignals_Table) {}
};

enum c_SIM_Spark_AbsoluteEncoderSignals {
    c_SIM_Spark_AbsoluteEncoderSignal_position,
    c_SIM_Spark_AbsoluteEncoderSignal_velocity,
    c_SIM_Spark_AbsoluteEncoderSignal_isInverted,
    c_SIM_Spark_AbsoluteEncoderSignal_zeroOffset,
    c_SIM_Spark_AbsoluteEncoderSignal_positionFactor,
    c_SIM_Spark_AbsoluteEncoderSignal_velocityFactor,
    c_SIM_Spark_AbsoluteEncoderSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_AbsoluteEncoderSignals_Table
        [c_SIM_Spark_AbsoluteEncoderSignal_NExternalSignals] = {
            {"Position", false, TYPE_DOUBLE, 0.0},
            {"Velocity", false, TYPE_DOUBLE, 0.0},
            {"Is Inverted", true, TYPE_BOOLEAN, 0.0},
            {"Zero Offset", true, TYPE_DOUBLE, 0.0},
            {"Position Conversion Factor", true, TYPE_DOUBLE, 1.0},
            {"Velocity Conversion Factor", true, TYPE_DOUBLE, 1.0},
};

struct c_SIM_Spark_AbsoluteEncoder_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_AbsoluteEncoder_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice,
                               c_SIM_Spark_AbsoluteEncoderSignals_Table) {}
};

enum c_SIM_Spark_AnalogSensorSignals {
    c_SIM_Spark_AnalogSensorSignal_voltage,
    c_SIM_Spark_AnalogSensorSignal_position,
    c_SIM_Spark_AnalogSensorSignal_velocity,
    c_SIM_Spark_AnalogSensorSignal_isInverted,
    c_SIM_Spark_AnalogSensorSignal_positionFactor,
    c_SIM_Spark_AnalogSensorSignal_velocityFactor,
    c_SIM_Spark_AnalogSensorSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_AnalogSensorSignals_Table
        [c_SIM_Spark_AnalogSensorSignal_NExternalSignals] = {
            {"Voltage", false, TYPE_DOUBLE, 0.0},
            {"Position", false, TYPE_DOUBLE, 0.0},
            {"Velocity", false, TYPE_DOUBLE, 0.0},
            {"Is Inverted", true, TYPE_BOOLEAN, 0.0},
            {"Position Conversion Factor", true, TYPE_DOUBLE, 1.0},
            {"Velocity Conversion Factor", true, TYPE_DOUBLE, 1.0},
};

struct c_SIM_Spark_AnalogSensor_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_AnalogSensor_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice,
                               c_SIM_Spark_AnalogSensorSignals_Table) {}
};

enum c_SIM_Spark_LimitSwitchSignals {
    c_SIM_Spark_LimitSwitchSignal_isPressed,
    c_SIM_Spark_LimitSwitchSignal_isEnabled,
    c_SIM_Spark_LimitSwitchSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_LimitSwitchSignals_Table
        [c_SIM_Spark_LimitSwitchSignal_NExternalSignals] = {
            {"Is Pressed", false, TYPE_BOOLEAN, 0.0},
            {"Is Enabled", true, TYPE_BOOLEAN, 0.0},
};

struct c_SIM_Spark_LimitSwitch_Obj : public c_SIM_Spark_Base_Obj {
    bool m_forward;

    c_SIM_Spark_LimitSwitch_Obj(int32_t simDevice, bool forward)
        : c_SIM_Spark_Base_Obj(simDevice, c_SIM_Spark_LimitSwitchSignals_Table),
          m_forward{forward} {}
};

enum c_SIM_Spark_FaultManagerSignals {
    c_SIM_Spark_FaultManagerSignal_otherFault,
    c_SIM_Spark_FaultManagerSignal_motorTypeFault,
    c_SIM_Spark_FaultManagerSignal_sensorFault,
    c_SIM_Spark_FaultManagerSignal_canFault,
    c_SIM_Spark_FaultManagerSignal_temperatureFault,
    c_SIM_Spark_FaultManagerSignal_drvFault,
    c_SIM_Spark_FaultManagerSignal_escEepromFault,
    c_SIM_Spark_FaultManagerSignal_firmwareFault,
    c_SIM_Spark_FaultManagerSignal_brownoutWarning,
    c_SIM_Spark_FaultManagerSignal_overcurrentWarning,
    c_SIM_Spark_FaultManagerSignal_escEepromWarning,
    c_SIM_Spark_FaultManagerSignal_extEepromWarning,
    c_SIM_Spark_FaultManagerSignal_sensorWarning,
    c_SIM_Spark_FaultManagerSignal_stallWarning,
    c_SIM_Spark_FaultManagerSignal_hasResetWarning,
    c_SIM_Spark_FaultManagerSignal_otherWarning,
    c_SIM_Spark_FaultManagerSignal_otherStickyFault,
    c_SIM_Spark_FaultManagerSignal_motorTypeStickyFault,
    c_SIM_Spark_FaultManagerSignal_sensorStickyFault,
    c_SIM_Spark_FaultManagerSignal_canStickyFault,
    c_SIM_Spark_FaultManagerSignal_temperatureStickyFault,
    c_SIM_Spark_FaultManagerSignal_drvStickyFault,
    c_SIM_Spark_FaultManagerSignal_escEepromStickyFault,
    c_SIM_Spark_FaultManagerSignal_firmwareStickyFault,
    c_SIM_Spark_FaultManagerSignal_brownoutStickyWarning,
    c_SIM_Spark_FaultManagerSignal_overcurrentStickyWarning,
    c_SIM_Spark_FaultManagerSignal_escEepromStickyWarning,
    c_SIM_Spark_FaultManagerSignal_extEepromStickyWarning,
    c_SIM_Spark_FaultManagerSignal_sensorStickyWarning,
    c_SIM_Spark_FaultManagerSignal_stallStickyWarning,
    c_SIM_Spark_FaultManagerSignal_hasResetStickyWarning,
    c_SIM_Spark_FaultManagerSignal_otherStickyWarning,
    c_SIM_Spark_FaultManagerSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_FaultManagerSignals_Table
        [c_SIM_Spark_FaultManagerSignal_NExternalSignals] = {
            {"Other Fault", false, TYPE_BOOLEAN, 0.0},
            {"Motor Type Fault", false, TYPE_BOOLEAN, 0.0},
            {"Sensor Fault", false, TYPE_BOOLEAN, 0.0},
            {"CAN Fault", false, TYPE_BOOLEAN, 0.0},
            {"Temperature Fault", false, TYPE_BOOLEAN, 0.0},
            {"DRV Fault", false, TYPE_BOOLEAN, 0.0},
            {"ESC Eeprom Fault", false, TYPE_BOOLEAN, 0.0},
            {"Firmware Fault", false, TYPE_BOOLEAN, 0.0},
            {"Brownout Warning", false, TYPE_BOOLEAN, 0.0},
            {"Over Current Warning", false, TYPE_BOOLEAN, 0.0},
            {"ESC Eeprom Warning", false, TYPE_BOOLEAN, 0.0},
            {"EXT Eeprom Warning", false, TYPE_BOOLEAN, 0.0},
            {"Sensor Warning", false, TYPE_BOOLEAN, 0.0},
            {"Stall Warning", false, TYPE_BOOLEAN, 0.0},
            {"Has Reset Warning", false, TYPE_BOOLEAN, 0.0},
            {"Other Warning", false, TYPE_BOOLEAN, 0.0},
            {"Other Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Motor Type Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Sensor Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"CAN Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Temperature Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"DRV Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"ESC Eeprom Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Firmware Sticky Fault", false, TYPE_BOOLEAN, 0.0},
            {"Brownout Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"Over Current Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"ESC Eeprom Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"EXT Eeprom Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"Sensor Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"Stall Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"Has Reset Sticky Warning", false, TYPE_BOOLEAN, 0.0},
            {"Other Sticky Warning", false, TYPE_BOOLEAN, 0.0},
};

struct c_SIM_Spark_FaultManager_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_FaultManager_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice,
                               c_SIM_Spark_FaultManagerSignals_Table) {}
};

enum c_SIM_Spark_MAXMotionSignals {
    c_SIM_Spark_MAXMotionSignal_positionTarget,
    c_SIM_Spark_MAXMotionSignal_velocityTarget,
    c_SIM_Spark_MAXMotionSignal_accelerationTarget,
    c_SIM_Spark_MAXMotionSignal_profileIsValid,
    c_SIM_Spark_MAXMotionSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_MAXMotionSignals_Table
        [c_SIM_Spark_MAXMotionSignal_NExternalSignals] = {
            {"Position Target", true, TYPE_DOUBLE, 0.0},
            {"Velocity Target", true, TYPE_DOUBLE, 0.0},
            {"Acceleration Target", true, TYPE_DOUBLE, 0.0},
            {"Profile Is Valid", true, TYPE_BOOLEAN, 0.0},
};

struct c_SIM_Spark_MAXMotion_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_MAXMotion_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice, c_SIM_Spark_MAXMotionSignals_Table) {}
};

enum c_SIM_Spark_FeedForwardSignals {
    c_SIM_Spark_FeedForwardSignal_s,
    c_SIM_Spark_FeedForwardSignal_v,
    c_SIM_Spark_FeedForwardSignal_a,
    c_SIM_Spark_FeedForwardSignal_g,
    c_SIM_Spark_FeedForwardSignal_cos,
    c_SIM_Spark_FeedForwardSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_FeedForwardSignals_Table
        [c_SIM_Spark_FeedForwardSignal_NExternalSignals] = {
            {"S term (Volts)", true, TYPE_DOUBLE, 0.0},
            {"V term (Volts)", true, TYPE_DOUBLE, 0.0},
            {"A term (Volts)", true, TYPE_DOUBLE, 0.0},
            {"G term (Volts)", true, TYPE_DOUBLE, 0.0},
            {"Cos term (Volts)", true, TYPE_DOUBLE, 0.0},
};

struct c_SIM_Spark_FeedForward_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_FeedForward_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice,
                               c_SIM_Spark_FeedForwardSignals_Table) {}
};

enum c_SIM_Spark_RelativeEncoderSignals {
    c_SIM_Spark_RelativeEncoderSignal_position,
    c_SIM_Spark_RelativeEncoderSignal_velocity,
    c_SIM_Spark_RelativeEncoderSignal_isInverted,
    c_SIM_Spark_RelativeEncoderSignal_positionFactor,
    c_SIM_Spark_RelativeEncoderSignal_velocityFactor,
    c_SIM_Spark_RelativeEncoderSignal_NExternalSignals
};

static constexpr c_SIM_Spark_SparkSignals_TableDef
    c_SIM_Spark_RelativeEncoderSignals_Table
        [c_SIM_Spark_RelativeEncoderSignal_NExternalSignals] = {
            {"Position", false, TYPE_DOUBLE, 0.0},
            {"Velocity", false, TYPE_DOUBLE, 0.0},
            {"Is Inverted", true, TYPE_BOOLEAN, 0.0},
            {"Position Conversion Factor", true, TYPE_DOUBLE, 1.0},
            {"Velocity Conversion Factor", true, TYPE_DOUBLE, 1.0},
};

struct c_SIM_Spark_RelativeEncoder_Obj : public c_SIM_Spark_Base_Obj {
    explicit c_SIM_Spark_RelativeEncoder_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice,
                               c_SIM_Spark_RelativeEncoderSignals_Table) {}
};

struct c_SIM_Spark_InternalSignals {
    int busId;
    int deviceId;
    std::string deviceName;
    float setpoint;
    float arbFF;
    float iGain{0.0f};
    float prevErr{0.0f};
    bool isAtSetpoint{false};
    maxmotion_profile_t maxMotionProfile;
    maxmotion_velocity_state_t MaxMotionLastSetpoint;
    maxmotion_state_t maxMotionIntermediateTarget;
    float currentLimitMovingAvg[1U << 6U]{};
    float currentLimitReboundFactor{0.002f};
    uint8_t arbFFUnits;
    uint8_t pidSlot;
    uint8_t sparkModel;
    uint8_t motorInterface;

    c_SIM_Spark_ExtOrAltEncoder_handle m_simExtOrAltEncoder{nullptr};
    c_SIM_Spark_AbsoluteEncoder_handle m_simAbsoluteEncoder{nullptr};
    c_SIM_Spark_AnalogSensor_handle m_simAnalogSensor{nullptr};
    c_SIM_Spark_LimitSwitch_handle m_simForwardLimitSwitch{nullptr};
    c_SIM_Spark_LimitSwitch_handle m_simReverseLimitSwitch{nullptr};
    c_SIM_Spark_FaultManager_handle m_simFaultManager{nullptr};
    c_SIM_Spark_RelativeEncoder_handle m_simRelativeEncoder{nullptr};
    c_SIM_Spark_MAXMotion_handle m_simMAXMotion{nullptr};
    c_SIM_Spark_FeedForward_handle m_simFeedForward{nullptr};

    ~c_SIM_Spark_InternalSignals() {
        delete m_simExtOrAltEncoder;
        delete m_simAbsoluteEncoder;
        delete m_simAnalogSensor;
        delete m_simForwardLimitSwitch;
        delete m_simReverseLimitSwitch;
        delete m_simFaultManager;
        delete m_simRelativeEncoder;
    }
};

struct c_SIM_Spark_Obj : public c_SIM_Spark_Base_Obj {
    uint32_t vParameterTable[_PARAM_TABLE_SIZE];  // NOLINT(runtime/arrays)
    int32_t m_controlMode{};

    c_SIM_Spark_InternalSignals m_internalSignals{};

    explicit c_SIM_Spark_Obj(int32_t simDevice)
        : c_SIM_Spark_Base_Obj(simDevice, c_SIM_Spark_SparkSignals_Table) {}
};

static void c_SIM_Spark_SetSparkSignalValue(c_SIM_Spark_handle handle,
                                            c_SIM_Spark_SparkSignals signal,
                                            double v) {
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

// List of parameters not modified by RestoreFactoryDefaults()
static const uint8_t _vParameterTableResetPersist[] = {
    c_Spark_kCANID,    c_Spark_kInputMode,     c_Spark_kMotorType,
    c_Spark_kIdleMode, c_Spark_kInputDeadband, c_Spark_kCompatibilityPortConfig,
};

static void c_SIM_Spark_InitializeParameterTable(uint32_t* vParameterTable) {
    std::memset(vParameterTable, 0, sizeof(uint32_t) * c_Spark_NumParameters);

    for (uint8_t i = 0u; i < c_Spark_NumParameters; i++) {
        vParameterTable[i] = c_Spark_GetParameterDefaultValue(
            static_cast<c_Spark_ConfigParameter>(i));
    }

    // The firmware default for closed loop control is 0, but 1 is required for
    // simulation to work
    vParameterTable[c_Spark_kClosedLoopControlSensor] = 1;
}

c_SIM_Spark_handle c_SIM_Spark_Create(int busId, int deviceId,
                                      c_Spark_SparkModel sparkModel) {
#ifdef REV_CAN_SPARK_NO_SIM
    return nullptr;
#endif

    auto deviceName = fmt::format(
        "{} [{},{}]",
        sparkModel == c_Spark_SparkFlex
            ? "SPARK Flex"
            : (sparkModel == c_Spark_SparkMax ? "SPARK MAX" : "UNKNOWN"),
        busId, deviceId);

    int32_t simHandle = getREVLibDriver()->createSimDevice(deviceName.c_str());
    if (simHandle == REVLIB_INVALID_HANDLE) {
        return nullptr;
    }

    c_SIM_Spark_handle handle =
        (c_SIM_Spark_handle) new c_SIM_Spark_Obj(simHandle);

    if (handle == nullptr) {
        getREVLibDriver()->freeSimDevice(simHandle);
        return nullptr;
    }

    handle->m_simDevice = simHandle;

    c_SIM_Spark_InitializeParameterTable(handle->vParameterTable);

    handle->m_internalSignals.sparkModel = sparkModel;
    handle->m_internalSignals.motorInterface =
        1;  // Use docked as default to support both brushed and brushless
    handle->m_internalSignals.busId = busId;
    handle->m_internalSignals.deviceId = deviceId;
    handle->m_internalSignals.deviceName = deviceName;

    return handle;
}

void c_SIM_Spark_Close(c_SIM_Spark_handle handle) { handle->close(); }

void c_SIM_Spark_Destroy(c_SIM_Spark_handle handle) { delete handle; }

bool c_SIM_Spark_IsSim(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return false;
    }

    return (handle->m_simDevice != REVLIB_INVALID_HANDLE);
}

c_REVLib_ErrorCode c_SIM_Spark_SetParameter(c_SIM_Spark_handle handle,
                                            uint8_t parameterID, uint8_t type,
                                            uint32_t value) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if (parameterID >= static_cast<uint8_t>(c_Spark_NumParameters)) {
        return c_REVLibError_ParamInvalidID;
    }
    if (static_cast<uint8_t>(c_Spark_GetParameterType(
            static_cast<c_Spark_ConfigParameter>(parameterID))) != type) {
        return c_REVLibError_ParamMismatchType;
    }

    c_Spark_ConfigParameter parameter =
        static_cast<c_Spark_ConfigParameter>(parameterID);

    float valueAsFloat;
    std::memcpy(&valueAsFloat, &value, sizeof(valueAsFloat));

    switch (parameter) {
        case c_Spark_kHardLimitFwdEn: {
            c_SIM_Spark_LimitSwitch_handle SIM_Spark_fwd_limitSwitchHandle =
                c_SIM_Spark_GetOrCreateSimForwardLimitSwitch(handle);
            c_SIM_Spark_SetSimLimitSwitchIsEnabled(
                SIM_Spark_fwd_limitSwitchHandle, static_cast<bool>(value));
            break;
        }
        case c_Spark_kHardLimitRevEn: {
            c_SIM_Spark_LimitSwitch_handle SIM_Spark_rev_limitSwitchHandle =
                c_SIM_Spark_GetOrCreateSimReverseLimitSwitch(handle);
            c_SIM_Spark_SetSimLimitSwitchIsEnabled(
                SIM_Spark_rev_limitSwitchHandle, static_cast<bool>(value));
            break;
        }
        case c_Spark_kEncoderInverted: {
            c_SIM_Spark_RelativeEncoder_handle SIM_Spark_relativeEncoderHandle =
                c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle);
            c_SIM_Spark_SetSimRelativeEncoderInverted(
                SIM_Spark_relativeEncoderHandle, static_cast<bool>(value));
            break;
        }
        case c_Spark_kPositionConversionFactor: {
            c_SIM_Spark_RelativeEncoder_handle SIM_Spark_relativeEncoderHandle =
                c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle);
            c_SIM_Spark_SetSimRelativeEncoderPositionFactor(
                SIM_Spark_relativeEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kVelocityConversionFactor: {
            c_SIM_Spark_RelativeEncoder_handle SIM_Spark_relativeEncoderHandle =
                c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle);
            c_SIM_Spark_SetSimRelativeEncoderVelocityFactor(
                SIM_Spark_relativeEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kAnalogPositionConversion: {
            c_SIM_Spark_AnalogSensor_handle SIM_Spark_analogSensorHandle =
                c_SIM_Spark_GetOrCreateSimAnalogSensor(handle);
            c_SIM_Spark_SetSimAnalogSensorPositionFactor(
                SIM_Spark_analogSensorHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kAnalogVelocityConversion: {
            c_SIM_Spark_AnalogSensor_handle SIM_Spark_analogSensorHandle =
                c_SIM_Spark_GetOrCreateSimAnalogSensor(handle);
            c_SIM_Spark_SetSimAnalogSensorVelocityFactor(
                SIM_Spark_analogSensorHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kAnalogInverted: {
            c_SIM_Spark_AnalogSensor_handle SIM_Spark_analogSensorHandle =
                c_SIM_Spark_GetOrCreateSimAnalogSensor(handle);
            c_SIM_Spark_SetSimAnalogSensorInverted(SIM_Spark_analogSensorHandle,
                                                   static_cast<bool>(value));
            break;
        }
        case c_Spark_kAltEncoderInverted: {
            c_SIM_Spark_ExtOrAltEncoder_handle SIM_Spark_extOrAltEncoderHandle =
                c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle);
            c_SIM_Spark_SetSimExtOrAltEncoderInverted(
                SIM_Spark_extOrAltEncoderHandle, static_cast<bool>(value));
            break;
        }
        case c_Spark_kAltEncoderPositionConversion: {
            c_SIM_Spark_ExtOrAltEncoder_handle SIM_Spark_extOrAltEncoderHandle =
                c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle);
            c_SIM_Spark_SetSimExtOrAltEncoderPositionFactor(
                SIM_Spark_extOrAltEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kAltEncoderVelocityConversion: {
            c_SIM_Spark_ExtOrAltEncoder_handle SIM_Spark_extOrAltEncoderHandle =
                c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle);
            c_SIM_Spark_SetSimExtOrAltEncoderVelocityFactor(
                SIM_Spark_extOrAltEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kDutyCyclePositionFactor: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            c_SIM_Spark_SetSimAbsoluteEncoderPositionFactor(
                SIM_Spark_absoluteEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kDutyCycleVelocityFactor: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            c_SIM_Spark_SetSimAbsoluteEncoderVelocityFactor(
                SIM_Spark_absoluteEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        case c_Spark_kDutyCycleInverted: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            c_SIM_Spark_SetSimAbsoluteEncoderInverted(
                SIM_Spark_absoluteEncoderHandle, static_cast<bool>(value));
            break;
        }
        case c_Spark_kDutyCycleOffset: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            c_SIM_Spark_SetSimAbsoluteEncoderZeroOffset(
                SIM_Spark_absoluteEncoderHandle,
                static_cast<double>(valueAsFloat));
            break;
        }
        default:
            // shut up compiler warnings
            break;
    }

    handle->vParameterTable[parameterID] = value;
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_GetParameter(c_SIM_Spark_handle handle,
                                            uint8_t parameterID,
                                            uint8_t expectedType,
                                            uint32_t* value) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    if (parameterID >= static_cast<uint8_t>(c_Spark_NumParameters)) {
        return c_REVLibError_ParamInvalidID;
    }
    if (static_cast<uint8_t>(c_Spark_GetParameterType(
            static_cast<c_Spark_ConfigParameter>(parameterID))) !=
        expectedType) {
        return c_REVLibError_ParamMismatchType;
    }

    float valueAsFloat;

    c_Spark_ConfigParameter parameter =
        static_cast<c_Spark_ConfigParameter>(parameterID);

    switch (parameter) {
        case c_Spark_kHardLimitFwdEn: {
            c_SIM_Spark_LimitSwitch_handle SIM_Spark_fwd_limitSwitchHandle =
                c_SIM_Spark_GetOrCreateSimForwardLimitSwitch(handle);
            *value =
                static_cast<uint32_t>(c_SIM_Spark_GetSimLimitSwitchIsEnabled(
                    SIM_Spark_fwd_limitSwitchHandle));
            break;
        }
        case c_Spark_kHardLimitRevEn: {
            c_SIM_Spark_LimitSwitch_handle SIM_Spark_rev_limitSwitchHandle =
                c_SIM_Spark_GetOrCreateSimReverseLimitSwitch(handle);
            *value =
                static_cast<uint32_t>(c_SIM_Spark_GetSimLimitSwitchIsEnabled(
                    SIM_Spark_rev_limitSwitchHandle));
            break;
        }
        case c_Spark_kEncoderInverted: {
            c_SIM_Spark_RelativeEncoder_handle SIM_Spark_relativeEncoderHandle =
                c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle);
            *value =
                static_cast<uint32_t>(c_SIM_Spark_GetSimRelativeEncoderInverted(
                    SIM_Spark_relativeEncoderHandle));
            break;
        }
        case c_Spark_kPositionConversionFactor: {
            c_SIM_Spark_RelativeEncoder_handle SIM_Spark_relativeEncoderHandle =
                c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle);
            valueAsFloat = static_cast<float>(
                c_SIM_Spark_GetSimRelativeEncoderPositionFactor(
                    SIM_Spark_relativeEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kVelocityConversionFactor: {
            c_SIM_Spark_RelativeEncoder_handle SIM_Spark_relativeEncoderHandle =
                c_SIM_Spark_GetOrCreateSimRelativeEncoder(handle);
            valueAsFloat = static_cast<float>(
                c_SIM_Spark_GetSimRelativeEncoderVelocityFactor(
                    SIM_Spark_relativeEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kAnalogPositionConversion: {
            c_SIM_Spark_AnalogSensor_handle SIM_Spark_analogSensorHandle =
                c_SIM_Spark_GetOrCreateSimAnalogSensor(handle);
            valueAsFloat =
                static_cast<float>(c_SIM_Spark_GetSimAnalogSensorPositionFactor(
                    SIM_Spark_analogSensorHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kAnalogVelocityConversion: {
            c_SIM_Spark_AnalogSensor_handle SIM_Spark_analogSensorHandle =
                c_SIM_Spark_GetOrCreateSimAnalogSensor(handle);
            valueAsFloat =
                static_cast<float>(c_SIM_Spark_GetSimAnalogSensorVelocityFactor(
                    SIM_Spark_analogSensorHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kAnalogInverted: {
            c_SIM_Spark_AnalogSensor_handle SIM_Spark_analogSensorHandle =
                c_SIM_Spark_GetOrCreateSimAnalogSensor(handle);
            *value =
                static_cast<uint32_t>(c_SIM_Spark_GetSimAnalogSensorInverted(
                    SIM_Spark_analogSensorHandle));
            break;
        }
        case c_Spark_kAltEncoderInverted: {
            c_SIM_Spark_ExtOrAltEncoder_handle SIM_Spark_extOrAltEncoderHandle =
                c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle);
            *value =
                static_cast<uint32_t>(c_SIM_Spark_GetSimExtOrAltEncoderInverted(
                    SIM_Spark_extOrAltEncoderHandle));
            break;
        }
        case c_Spark_kAltEncoderPositionConversion: {
            c_SIM_Spark_ExtOrAltEncoder_handle SIM_Spark_extOrAltEncoderHandle =
                c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle);
            valueAsFloat = static_cast<float>(
                c_SIM_Spark_GetSimExtOrAltEncoderPositionFactor(
                    SIM_Spark_extOrAltEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kAltEncoderVelocityConversion: {
            c_SIM_Spark_ExtOrAltEncoder_handle SIM_Spark_extOrAltEncoderHandle =
                c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(handle);
            valueAsFloat = static_cast<float>(
                c_SIM_Spark_GetSimExtOrAltEncoderVelocityFactor(
                    SIM_Spark_extOrAltEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kDutyCyclePositionFactor: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            valueAsFloat = static_cast<float>(
                c_SIM_Spark_GetSimAbsoluteEncoderPositionFactor(
                    SIM_Spark_absoluteEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kDutyCycleVelocityFactor: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            valueAsFloat = static_cast<float>(
                c_SIM_Spark_GetSimAbsoluteEncoderVelocityFactor(
                    SIM_Spark_absoluteEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }
        case c_Spark_kDutyCycleInverted: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            *value =
                static_cast<uint32_t>(c_SIM_Spark_GetSimAbsoluteEncoderInverted(
                    SIM_Spark_absoluteEncoderHandle));
            break;
        }
        case c_Spark_kDutyCycleOffset: {
            c_SIM_Spark_AbsoluteEncoder_handle SIM_Spark_absoluteEncoderHandle =
                c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(handle);
            valueAsFloat =
                static_cast<float>(c_SIM_Spark_GetSimAbsoluteEncoderZeroOffset(
                    SIM_Spark_absoluteEncoderHandle));
            std::memcpy(value, &valueAsFloat, sizeof(valueAsFloat));
            break;
        }

        default:
            *value = handle->vParameterTable[parameterID];
    }

    return c_REVLibError_None;
}

// New sim functions, telemetry 'get' functions only, SimDevice Values
// handle setting

float c_SIM_Spark_GetAppliedOutput(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_appliedOutput], &value);
    return static_cast<float>(value.data.d);
}
uint8_t c_SIM_Spark_GetInverted(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0;
    }

    return static_cast<uint8_t>(handle->vParameterTable[c_Spark_kInverted]);
}

uint8_t c_SIM_Spark_IsFollower(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0;
    }

    const uint32_t followerId =
        handle->vParameterTable[c_Spark_kFollowerModeLeaderId];
    return (followerId != 0) ? 1 : 0;
}

bool c_SIM_Spark_SoftForwardLimitReached(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0;
    }
    float softLimit;
    std::memcpy(&softLimit,
                &(handle->vParameterTable[c_Spark_kSoftLimitForward]),
                sizeof(float));
    return handle->vParameterTable[c_Spark_kSoftLimitFwdEn] &&
           (softLimit < c_SIM_Spark_GetPosition(handle));
}

bool c_SIM_Spark_SoftReverseLimitReached(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0;
    }

    float softLimit;
    std::memcpy(&softLimit,
                &(handle->vParameterTable[c_Spark_kSoftLimitReverse]),
                sizeof(float));
    return handle->vParameterTable[c_Spark_kSoftLimitRevEn] &&
           (softLimit < c_SIM_Spark_GetPosition(handle));
}

float c_SIM_Spark_GetOutputCurrent(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_motorCurrent], &value);
    return static_cast<float>(value.data.d);
}

float c_SIM_Spark_GetBusVoltage(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_busVoltage], &value);
    return static_cast<float>(value.data.d);
}

uint8_t c_SIM_Spark_GetMotorTemperature(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_motorTemp], &value);
    return static_cast<uint8_t>(value.data.i);
}

float c_SIM_Spark_GetVelocity(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_velocity], &value);
    return static_cast<float>(value.data.d);
}

float c_SIM_Spark_GetPosition(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_position], &value);
    return static_cast<float>(value.data.d);
}

ctrlType_t c_SIM_Spark_GetControlMode(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return CTRL_NONE;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_controlMode], &value);
    return static_cast<ctrlType_t>(value.data.i);
}

void c_SIM_Spark_SetAppliedOutput(c_SIM_Spark_handle handle, float value) {
    if (handle == nullptr) {
        return;
    }

    struct REVLibValue v;
    v.type = TYPE_DOUBLE;
    v.data.d = static_cast<double>(value);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_appliedOutput], &v);
}

c_REVLib_ErrorCode c_SIM_Spark_StartFollowerMode(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    struct REVLibValue v;
    v.type = TYPE_BOOLEAN;
    v.data.b = static_cast<bool>(true);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_followLeader], &v);

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_StopFollowerMode(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    REVLibValue v;
    v.type = TYPE_BOOLEAN;
    v.data.b = static_cast<bool>(false);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_followLeader], &v);

    return c_REVLibError_None;
}

/*
const int CMD_API_MECH_POS = 0x0A0;
const int CMD_API_I_ACCUM = 0x0A2;
const int CMD_API_ANALOG_POS = 0x0A3;
const int CMD_API_ALT_ENC_POS = 0x0A4;
*/
c_REVLib_ErrorCode c_SIM_Spark_SetTelemetry(c_SIM_Spark_handle handle,
                                            float value, int arbId) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    enum c_SIM_Spark_SparkSignals signalToChange;
    uint32_t baseArbId = arbId & 0x1FFFFFC0;
    switch (baseArbId) {
        case SPARK_SET_PRIMARY_ENCODER_POSITION_FRAME_ID:
            signalToChange = c_SIM_Spark_SparkSignal_position;
            break;
        case SPARK_SET_I_ACCUMULATION_FRAME_ID:
            return c_REVLibError_None;
            break;
        default:
            return c_REVLibError_Invalid;
            break;
    }
    REVLibValue v;
    v.type = TYPE_DOUBLE;
    v.data.d = static_cast<double>(value);

    getREVLibDriver()->getSimValue(handle->m_signals[signalToChange], &v);
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_RestoreFactoryDefaults(c_SIM_Spark_handle handle,
                                                      bool persist,
                                                      bool resetAll) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }
    int paramIdIdx = 0;
    for (uint32_t i = 0; i < c_Spark_NumParameters; i++) {
        if (!resetAll && i == _vParameterTableResetPersist[paramIdIdx]) {
            paramIdIdx++;
            continue;
        }
        handle->vParameterTable[i] = c_Spark_GetParameterDefaultValue(
            static_cast<c_Spark_ConfigParameter>(i));
    }

    // The firmware default for closed loop control is 0, but 1 is required for
    // simulation to work
    handle->vParameterTable[c_Spark_kClosedLoopControlSensor] = 1;

    (void)persist;

    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_SetSetpoint(c_SIM_Spark_handle handle,
                                           float value, uint8_t ctrl,
                                           int pidSlot, float arbFeedforward,
                                           int arbFFUnits) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    handle->m_internalSignals.arbFF = arbFeedforward;
    handle->m_internalSignals.arbFFUnits = arbFFUnits;
    handle->m_internalSignals.pidSlot = pidSlot;
    handle->m_internalSignals.setpoint = value;
    handle->vParameterTable[c_Spark_kControlType] = ctrl;

    c_SIM_Spark_SetSparkSignalValue(
        handle, c_SIM_Spark_SparkSignal_arbFeedforward, arbFeedforward);
    c_SIM_Spark_SetSparkSignalValue(handle, c_SIM_Spark_SparkSignal_arbFFUnits,
                                    arbFFUnits);
    c_SIM_Spark_SetSparkSignalValue(
        handle, c_SIM_Spark_SparkSignal_ClosedLoopSlot, pidSlot);
    // if duty cycle
    if (ctrl == 0) {
        // clamp the displayed output to [-1,1]
        // this does not affect the value returned by the getter
        // this is consistent with the actual function
        value = fminf(fmaxf(value, -1), 1);
    }
    c_SIM_Spark_SetSparkSignalValue(handle, c_SIM_Spark_SparkSignal_setpoint,
                                    value);
    c_SIM_Spark_SetSparkSignalValue(handle, c_SIM_Spark_SparkSignal_controlMode,
                                    ctrl);

    return c_REVLibError_None;
}

float c_SIM_Spark_GetSetpoint(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_setpoint], &value);
    return static_cast<float>(value.data.d);
}

bool c_SIM_Spark_IsAtSetpoint(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    return handle->m_internalSignals.isAtSetpoint;
}

int c_SIM_Spark_GetClosedLoopSlot(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return -1;
    }

    struct REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_ClosedLoopSlot], &value);
    return static_cast<int>(value.data.i);
}

float c_SIM_Spark_GetMAXMotionSetpointPosition(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    return handle->m_internalSignals.maxMotionIntermediateTarget.position;
}

float c_SIM_Spark_GetMAXMotionSetpointVelocity(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return 0.0f;
    }

    return handle->m_internalSignals.maxMotionIntermediateTarget.velocity;
}

c_REVLib_ErrorCode c_SIM_Spark_GetDRVStatus(c_SIM_Spark_handle handle,
                                            uint16_t* DRVStat0,
                                            uint16_t* DRVStat1,
                                            uint16_t* faults,
                                            uint16_t* stickyFaults) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    *DRVStat0 = 0;
    *DRVStat1 = 0;
    *faults = 0;
    *stickyFaults = 0;
    return c_REVLibError_None;
}

c_REVLib_ErrorCode c_SIM_Spark_ClearFaults(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    return c_REVLibError_None;
}

uint8_t c_SIM_Spark_GetSparkModel(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    return handle->m_internalSignals.sparkModel;
}

uint8_t c_SIM_Spark_GetMotorInterface(c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    return handle->m_internalSignals.motorInterface;
}

float GetFloatParameter(c_SIM_Spark_handle handle, int id) {
    uint32_t value = handle->vParameterTable[id];
    float tmp;
    std::memcpy(&tmp, &value, sizeof(tmp));
    return tmp;
}

feedforward_constants_t c_SIM_Spark_GetFeedforwardConstants(
    c_SIM_Spark_handle handle) {
    if (handle == nullptr) {
        return {0, 0, 0, 0, 0, 0};
    }
    int slot = handle->m_internalSignals.pidSlot;
    feedforward_constants_t constants;

    constexpr int constantOffset{c_Spark_kS_1 - c_Spark_kS_0};

    constants.kS =
        GetFloatParameter(handle, c_Spark_kS_0 + constantOffset * slot);
    constants.kV =
        GetFloatParameter(handle, c_Spark_kV_0 + constantOffset * slot);
    constants.kA =
        GetFloatParameter(handle, c_Spark_kA_0 + constantOffset * slot);
    constants.kG =
        GetFloatParameter(handle, c_Spark_kG_0 + constantOffset * slot);
    constants.kCos =
        GetFloatParameter(handle, c_Spark_kCos_0 + constantOffset * slot);
    constants.kCosRatio =
        GetFloatParameter(handle, c_Spark_kCosRatio_0 + 5 * slot);
    return constants;
}

float c_SIM_Spark_CalculateFeedforward(c_SIM_Spark_handle handle,
                                       const feedforward_constants_t* constants,
                                       feedforward_state_t state,
                                       feedforward_signals_t signals,
                                       const float Vbus) {
    float sgain = copysignf(constants->kS, state.velocity);
    float vgain = constants->kV * state.velocity;
    float again = constants->kA * state.acceleration;
    float ggain = constants->kG;
    float cosgain = constants->kCos;
    if (signals.cos && constants->kCosRatio != 0 && cosgain != 0) {
        cosgain *= cosf(state.position * constants->kCosRatio * PI / 180.0f);
    }

    float feedforward = sgain * signals.s + vgain * signals.v +
                        again * signals.a + ggain * signals.g +
                        cosgain * signals.cos;

    if (feedforward != 0 &&
        (signals.s || signals.v || signals.a || signals.g || signals.cos)) {
        c_SIM_Spark_SetSimFeedForwardTelemetry(
            c_SIM_Spark_GetOrCreateSimFeedForward(handle), signals.s * sgain,
            signals.v * vgain, signals.a * again, signals.g * ggain,
            signals.cos * cosgain);
    }

    // convert volts to duty cycle
    float referenceVoltage = 12;

    uint32_t voltageCompMode = 0;
    c_SIM_Spark_GetParameter(handle, c_Spark_kVoltageCompensationMode,
                             c_REVLib_kUint32, &voltageCompMode);
    bool voltageCompensation =
        voltageCompMode == c_Spark_kVoltageCompMode_NO_VOLTAGE_COMP;

    if (!voltageCompensation) {
        referenceVoltage =
            GetFloatParameter(handle, c_Spark_kCompensatedNominalVoltage);
    } else if (Vbus != 0) {
        referenceVoltage = Vbus;
    }

    if (referenceVoltage == 0) return 0;

    return feedforward / referenceVoltage;
}

float calculate_continuous_error(c_SIM_Spark_handle handle, float setpoint,
                                 float pv) {
    float minInput = GetFloatParameter(handle, c_Spark_kPositionPIDMinInput);
    float maxInput = GetFloatParameter(handle, c_Spark_kPositionPIDMaxInput);
    float modulus = (maxInput - minInput);
    float one_over_modulus = 1.0f / (maxInput - minInput);
    float errorBound = modulus / 2.0f;

    float error = setpoint - pv;
    int numMax = static_cast<int>((error + errorBound) * one_over_modulus);
    error -= numMax * modulus;

    int numMin = static_cast<int>((error - errorBound) * one_over_modulus);
    error -= numMin * modulus;

    return error;
}

float c_SIM_Spark_CalculatePID(c_SIM_Spark_handle handle, float setpoint,
                               float pv, float dt, float feedforward) {
    REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_ClosedLoopSlot], &value);
    int slot = static_cast<int>(value.data.i);
    int parameterIDOffset = c_Spark_kP_0 + 8 * slot;

    const float p = GetFloatParameter(handle, parameterIDOffset + 0);
    const float i = GetFloatParameter(handle, parameterIDOffset + 1);
    const float d = GetFloatParameter(handle, parameterIDOffset + 2);
    const float izone = GetFloatParameter(handle, parameterIDOffset + 4);
    [[maybe_unused]] const float dfilter =
        GetFloatParameter(handle, parameterIDOffset + 5);
    const float min = GetFloatParameter(handle, parameterIDOffset + 6);
    const float max = GetFloatParameter(handle, parameterIDOffset + 7);

    float error = setpoint - pv;

    // handle PID wrapping
    if (handle->vParameterTable[c_Spark_kPositionPIDWrapEnable] != 0) {
        error = calculate_continuous_error(handle, setpoint, pv);
    }

    handle->m_internalSignals.isAtSetpoint =
        fabsf(error) <=
        GetFloatParameter(handle, c_Spark_kAllowedClosedLoopError_0 + slot * 4);
    if (handle->m_internalSignals.isAtSetpoint) {
        return fminf(fmaxf(feedforward, min), max);
    }

    float pgain = error * p;

    if (fabsf(error) <= izone || izone == 0.0f) {
        handle->m_internalSignals.iGain =
            handle->m_internalSignals.iGain + (error * i * dt);
    } else {
        handle->m_internalSignals.iGain = 0;
    }

    float dgain = (error - handle->m_internalSignals.prevErr) / dt;
    handle->m_internalSignals.prevErr = error;
    dgain *= d;

    float output =
        pgain + handle->m_internalSignals.iGain + dgain + feedforward;
    output = fminf(fmaxf(output, min), max);

    return output;
}

void c_SIM_Spark_SetSimMAXMotionTelemetry(c_SIM_Spark_MAXMotion_handle handle,
                                          double position, double velocity,
                                          double acceleration, bool isValid) {
    if (handle == nullptr) return;
    struct REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_positionTarget], &hval);
    hval.data.d = position;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_positionTarget], &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_velocityTarget], &hval);
    hval.data.d = velocity;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_velocityTarget], &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_accelerationTarget],
        &hval);
    hval.data.d = acceleration;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_accelerationTarget],
        &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_profileIsValid], &hval);
    hval.data.b = isValid;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_MAXMotionSignal_profileIsValid], &hval);
}

void c_SIM_Spark_SetSimFeedForwardTelemetry(
    c_SIM_Spark_FeedForward_handle handle, double s, double v, double a,
    double g, double cos) {
    if (handle == nullptr) return;
    struct REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_s], &hval);
    hval.data.d = s;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_s], &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_v], &hval);
    hval.data.d = v;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_v], &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_a], &hval);
    hval.data.d = a;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_a], &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_g], &hval);
    hval.data.d = g;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_g], &hval);
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_cos], &hval);
    hval.data.d = cos;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_FeedForwardSignal_cos], &hval);
}

float c_SIM_Spark_SimulateMaxMotionPositionControl(c_SIM_Spark_handle handle,
                                                   float dt) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_ClosedLoopSlot], &value);
    int slot = static_cast<int>(value.data.i);
    int parameterIDOffset = c_Spark_kMAXMotionCruiseVelocity_0 + 5 * slot;

    maxmotion_constants_t motionParams = {
        GetFloatParameter(handle, parameterIDOffset + 0),
        GetFloatParameter(handle, parameterIDOffset + 1),
        GetFloatParameter(handle, parameterIDOffset + 2),
        GetFloatParameter(handle, parameterIDOffset + 3),
        static_cast<maxmotion_accel_mode_t>(
            GetFloatParameter(handle, parameterIDOffset + 4)),
        GetFloatParameter(handle, c_Spark_kPositionConversionFactor),
        GetFloatParameter(handle, c_Spark_kVelocityConversionFactor),
    };

    maxmotion_state_t current_state = {c_SIM_Spark_GetPosition(handle),
                                       c_SIM_Spark_GetVelocity(handle), 0, 0};

    maxmotion_state_t end_state = {handle->m_internalSignals.setpoint, 0, 0, 0};

    if (handle->vParameterTable[c_Spark_kPositionPIDWrapEnable] != 0) {
        if (handle->m_internalSignals.setpoint ==
            handle->m_internalSignals.maxMotionProfile.unwrapped_setpoint) {
            end_state.position =
                handle->m_internalSignals.maxMotionProfile.points[7]
                    .state.position;
        } else {
            handle->m_internalSignals.maxMotionProfile.is_valid = false;
        }
    }

    maxmotion_profile_check(&handle->m_internalSignals.maxMotionProfile,
                            motionParams, current_state, end_state);

    c_SIM_Spark_SetSimMAXMotionTelemetry(
        c_SIM_Spark_GetOrCreateSimMaxMotion(handle),
        handle->m_internalSignals.maxMotionIntermediateTarget.position,
        handle->m_internalSignals.maxMotionIntermediateTarget.velocity,
        handle->m_internalSignals.maxMotionIntermediateTarget.acceleration,
        handle->m_internalSignals.maxMotionProfile.is_valid);

    if (!handle->m_internalSignals.maxMotionProfile.is_valid) {
        // handle PID wrapping
        handle->m_internalSignals.maxMotionProfile.unwrapped_setpoint =
            handle->m_internalSignals.setpoint;
        if (handle->vParameterTable[c_Spark_kPositionPIDWrapEnable] != 0) {
            end_state.position =
                calculate_continuous_error(handle, end_state.position,
                                           current_state.position) +
                c_SIM_Spark_GetPosition(handle);
        }

        maxmotion_profile_generate(&handle->m_internalSignals.maxMotionProfile,
                                   motionParams, current_state, end_state);
    }

    handle->m_internalSignals.maxMotionIntermediateTarget =
        maxmotion_profile_get_point(
            &handle->m_internalSignals.maxMotionProfile,
            ++handle->m_internalSignals.maxMotionProfile.current_time);

    if (!handle->m_internalSignals.maxMotionProfile.is_valid) {
        handle->m_internalSignals.maxMotionIntermediateTarget = {
            c_SIM_Spark_GetPosition(handle), 0, 0, 0};
    }

    feedforward_constants_t ffConstants =
        c_SIM_Spark_GetFeedforwardConstants(handle);
    feedforward_state_t ffState;
    feedforward_signals_t ffSignals;
    float feedforward = 0;
    ffState.velocity =
        handle->m_internalSignals.maxMotionIntermediateTarget.velocity;
    ffState.acceleration =
        handle->m_internalSignals.maxMotionIntermediateTarget.acceleration;
    ffState.position =
        handle->m_internalSignals.maxMotionIntermediateTarget.position;
    ffSignals = {1, 1, 1, 1, 1};  // all ff signals are valid
    feedforward = c_SIM_Spark_CalculateFeedforward(
        handle, &ffConstants, ffState, ffSignals,
        c_SIM_Spark_GetBusVoltage(handle));

    return c_SIM_Spark_CalculatePID(
        handle, handle->m_internalSignals.maxMotionIntermediateTarget.position,
        c_SIM_Spark_GetPosition(handle), dt, feedforward);
}

float c_SIM_Spark_SimulateMaxMotionVelocityControl(c_SIM_Spark_handle handle,
                                                   float dt) {
    if (handle == nullptr) {
        return c_REVLibError_Invalid;
    }

    REVLibValue value;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_SparkSignal_ClosedLoopSlot], &value);
    int slot = static_cast<int>(value.data.i);
    int parameterIDOffset = c_Spark_kMAXMotionCruiseVelocity_0 + 5 * slot;

    uint32_t maxVelocity, maxAccel, maxJerk, allowedError, mode;

    maxVelocity = GetFloatParameter(handle, parameterIDOffset + 0);
    maxAccel = GetFloatParameter(handle, parameterIDOffset + 1);
    maxJerk = GetFloatParameter(handle, parameterIDOffset + 2);
    allowedError = GetFloatParameter(handle, parameterIDOffset + 3);
    mode = GetFloatParameter(handle, parameterIDOffset + 4);

    maxmotion_constants_t MAXMotionConstants = {
        static_cast<float>(maxVelocity),
        static_cast<float>(maxAccel),
        static_cast<float>(maxJerk),
        static_cast<float>(allowedError),
        static_cast<maxmotion_accel_mode_t>(mode),
        1.0,
        1.0};

    if (handle->m_internalSignals.MaxMotionLastSetpoint.velocity == 0) {
        handle->m_internalSignals.MaxMotionLastSetpoint.velocity =
            c_SIM_Spark_GetVelocity(handle);
    }

    handle->m_internalSignals.MaxMotionLastSetpoint =
        smartervelocity_calculate_output_velocity(
            &MAXMotionConstants, handle->m_internalSignals.setpoint,
            handle->m_internalSignals.MaxMotionLastSetpoint.velocity, dt);

    c_SIM_Spark_SetSimMAXMotionTelemetry(
        c_SIM_Spark_GetOrCreateSimMaxMotion(handle), 0,
        handle->m_internalSignals.MaxMotionLastSetpoint.velocity,
        handle->m_internalSignals.MaxMotionLastSetpoint.acceleration, true);

    float feedforward = 0;

    feedforward_constants_t ffConstants =
        c_SIM_Spark_GetFeedforwardConstants(handle);
    feedforward_state_t ffState = {
        0, handle->m_internalSignals.MaxMotionLastSetpoint.velocity,
        handle->m_internalSignals.MaxMotionLastSetpoint.acceleration, false};
    feedforward_signals_t ffSignals = {1, 1, 1, 0, 0};  // s, v, a; no g, cos

    feedforward = c_SIM_Spark_CalculateFeedforward(
        handle, &ffConstants, ffState, ffSignals,
        c_SIM_Spark_GetBusVoltage(handle));

    return c_SIM_Spark_CalculatePID(
        handle, handle->m_internalSignals.MaxMotionLastSetpoint.velocity,
        c_SIM_Spark_GetVelocity(handle), dt, feedforward);
}

float c_SIM_Spark_GetCurrentLimitMovingAvg(c_SIM_Spark_handle handle) {
    int size =
        sizeof(handle->m_internalSignals.currentLimitMovingAvg) / sizeof(float);
    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < size; i++) {
        float value = handle->m_internalSignals.currentLimitMovingAvg[i];
        if (value == 0.0f) {
            continue;
        }
        sum += value;
        count++;
    }
    if (count == 0) return 0;
    return sum / static_cast<float>(count);
}

void c_SIM_Spark_UpdateCurrentLimitMovingAvg(c_SIM_Spark_handle handle,
                                             float newValue) {
    int size =
        sizeof(handle->m_internalSignals.currentLimitMovingAvg) / sizeof(float);
    for (int i = size - 1; i > 1; i--) {
        handle->m_internalSignals.currentLimitMovingAvg[i] =
            handle->m_internalSignals.currentLimitMovingAvg[i - 1];
    }
    handle->m_internalSignals.currentLimitMovingAvg[0] = newValue;
}

float c_SIM_Spark_SimulateCurrentLimit(c_SIM_Spark_handle handle,
                                       float appliedOutput, float current) {
    float output = c_SIM_Spark_RunSmartCurrentLimit(
        appliedOutput, handle->vParameterTable[c_Spark_kSmartCurrentStallLimit],
        current, c_SIM_Spark_GetCurrentLimitMovingAvg(handle),
        &handle->m_internalSignals.currentLimitReboundFactor);

    c_SIM_Spark_UpdateCurrentLimitMovingAvg(handle, output);

    return output;
}

c_SIM_Spark_ExtOrAltEncoder_handle c_SIM_Spark_GetOrCreateSimExtOrAltEncoder(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simExtOrAltEncoder == NULL) {
        c_Spark_SparkModel device = static_cast<c_Spark_SparkModel>(
            handle->m_internalSignals.sparkModel);

        auto deviceName =
            fmt::format("{} {}", handle->m_internalSignals.deviceName,
                        device == c_Spark_SparkFlex ? "EXTERNAL ENCODER"
                                                    // default to MAX naming
                                                    : "ALTERNATE ENCODER");

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simExtOrAltEncoder =
            (c_SIM_Spark_ExtOrAltEncoder_handle) new c_SIM_Spark_ExtOrAltEncoder_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simExtOrAltEncoder;
}

c_SIM_Spark_RelativeEncoder_handle c_SIM_Spark_GetOrCreateSimRelativeEncoder(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simRelativeEncoder == NULL) {
        auto deviceName = fmt::format("{} RELATIVE ENCODER",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simRelativeEncoder =
            (c_SIM_Spark_RelativeEncoder_handle) new c_SIM_Spark_RelativeEncoder_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simRelativeEncoder;
}

c_SIM_Spark_AbsoluteEncoder_handle c_SIM_Spark_GetOrCreateSimAbsoluteEncoder(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simAbsoluteEncoder == NULL) {
        auto deviceName = fmt::format("{} ABSOLUTE ENCODER",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simAbsoluteEncoder =
            (c_SIM_Spark_AbsoluteEncoder_handle) new c_SIM_Spark_AbsoluteEncoder_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simAbsoluteEncoder;
}

c_SIM_Spark_AnalogSensor_handle c_SIM_Spark_GetOrCreateSimAnalogSensor(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simAnalogSensor == NULL) {
        auto deviceName = fmt::format("{} ANALOG SENSOR",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simAnalogSensor =
            (c_SIM_Spark_AnalogSensor_handle) new c_SIM_Spark_AnalogSensor_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simAnalogSensor;
}

c_SIM_Spark_LimitSwitch_handle c_SIM_Spark_GetOrCreateSimForwardLimitSwitch(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simForwardLimitSwitch == NULL) {
        auto deviceName = fmt::format("{} LIMIT SWITCH (FORWARD)",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simForwardLimitSwitch =
            (c_SIM_Spark_LimitSwitch_handle) new c_SIM_Spark_LimitSwitch_Obj(
                simHandle, true);
    }
    return handle->m_internalSignals.m_simForwardLimitSwitch;
}

c_SIM_Spark_LimitSwitch_handle c_SIM_Spark_GetOrCreateSimReverseLimitSwitch(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simReverseLimitSwitch == NULL) {
        auto deviceName = fmt::format("{} LIMIT SWITCH (REVERSE)",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simReverseLimitSwitch =
            (c_SIM_Spark_LimitSwitch_handle) new c_SIM_Spark_LimitSwitch_Obj(
                simHandle, false);
    }
    return handle->m_internalSignals.m_simReverseLimitSwitch;
}

c_SIM_Spark_FaultManager_handle c_SIM_Spark_GetOrCreateSimFaultManager(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simFaultManager == NULL) {
        auto deviceName = fmt::format("{} FAULT MANAGER",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simFaultManager =
            (c_SIM_Spark_FaultManager_handle) new c_SIM_Spark_FaultManager_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simFaultManager;
}

c_SIM_Spark_MAXMotion_handle c_SIM_Spark_GetOrCreateSimMaxMotion(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simMAXMotion == NULL) {
        auto deviceName =
            fmt::format("{} MAXMOTION", handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simMAXMotion =
            (c_SIM_Spark_MAXMotion_handle) new c_SIM_Spark_MAXMotion_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simMAXMotion;
}

c_SIM_Spark_FeedForward_handle c_SIM_Spark_GetOrCreateSimFeedForward(
    c_SIM_Spark_handle handle) {
    if (!c_SIM_Spark_IsSim(handle)) return nullptr;
    if (handle->m_internalSignals.m_simFeedForward == NULL) {
        auto deviceName = fmt::format("{} FEED FORWARD",
                                      handle->m_internalSignals.deviceName);

        int32_t simHandle =
            getREVLibDriver()->createSimDevice(deviceName.c_str());

        if (simHandle == REVLIB_INVALID_HANDLE) {
            return nullptr;
        }

        handle->m_internalSignals.m_simFeedForward =
            (c_SIM_Spark_FeedForward_handle) new c_SIM_Spark_FeedForward_Obj(
                simHandle);
    }
    return handle->m_internalSignals.m_simFeedForward;
}

// ExtOrAlt Encoder
void c_SIM_Spark_SetSimExtOrAltEncoderPosition(
    c_SIM_Spark_ExtOrAltEncoder_handle handle, double position) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_position], &hval);
    hval.data.d = position;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_position], &hval);
}
double c_SIM_Spark_GetSimExtOrAltEncoderPosition(
    c_SIM_Spark_ExtOrAltEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_position], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimExtOrAltEncoderVelocity(
    c_SIM_Spark_ExtOrAltEncoder_handle handle, double velocity) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_velocity], &hval);
    hval.data.d = velocity;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_velocity], &hval);
}
double c_SIM_Spark_GetSimExtOrAltEncoderVelocity(
    c_SIM_Spark_ExtOrAltEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_velocity], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimExtOrAltEncoderInverted(
    c_SIM_Spark_ExtOrAltEncoder_handle handle, bool inverted) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_isInverted], &hval);
    hval.data.b = static_cast<bool>(inverted);
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_isInverted], &hval);
}
bool c_SIM_Spark_GetSimExtOrAltEncoderInverted(
    c_SIM_Spark_ExtOrAltEncoder_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_isInverted], &hval);
    return static_cast<bool>(hval.data.b);
}
void c_SIM_Spark_SetSimExtOrAltEncoderZeroOffset(
    c_SIM_Spark_ExtOrAltEncoder_handle handle, double offset) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_zeroOffset], &hval);
    hval.data.d = offset;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_zeroOffset], &hval);
}
double c_SIM_Spark_GetSimExtOrAltEncoderZeroOffset(
    c_SIM_Spark_ExtOrAltEncoder_handle handle) {
    if (handle == nullptr) return 0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_zeroOffset], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimExtOrAltEncoderPositionFactor(
    c_SIM_Spark_ExtOrAltEncoder_handle handle, double posFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_positionFactor],
        &hval);
    hval.data.d = posFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_positionFactor],
        &hval);
}
double c_SIM_Spark_GetSimExtOrAltEncoderPositionFactor(
    c_SIM_Spark_ExtOrAltEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_positionFactor],
        &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimExtOrAltEncoderVelocityFactor(
    c_SIM_Spark_ExtOrAltEncoder_handle handle, double velFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_velocityFactor],
        &hval);
    hval.data.d = velFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_velocityFactor],
        &hval);
}
double c_SIM_Spark_GetSimExtOrAltEncoderVelocityFactor(
    c_SIM_Spark_ExtOrAltEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_ExtOrAltEncoderSignal_velocityFactor],
        &hval);
    return hval.data.d;
}

// Relative Encoder
void c_SIM_Spark_SetSimRelativeEncoderPosition(
    c_SIM_Spark_RelativeEncoder_handle handle, double position) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_position], &hval);
    hval.data.d = position;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_position], &hval);
}
double c_SIM_Spark_GetSimRelativeEncoderPosition(
    c_SIM_Spark_RelativeEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_position], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimRelativeEncoderVelocity(
    c_SIM_Spark_RelativeEncoder_handle handle, double velocity) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_velocity], &hval);
    hval.data.d = velocity;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_velocity], &hval);
}
double c_SIM_Spark_GetSimRelativeEncoderVelocity(
    c_SIM_Spark_RelativeEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_velocity], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimRelativeEncoderInverted(
    c_SIM_Spark_RelativeEncoder_handle handle, bool inverted) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_isInverted], &hval);
    hval.data.b = static_cast<bool>(inverted);
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_isInverted], &hval);
}
bool c_SIM_Spark_GetSimRelativeEncoderInverted(
    c_SIM_Spark_RelativeEncoder_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_isInverted], &hval);
    return static_cast<bool>(hval.data.b);
}
void c_SIM_Spark_SetSimRelativeEncoderPositionFactor(
    c_SIM_Spark_RelativeEncoder_handle handle, double posFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_positionFactor],
        &hval);
    hval.data.d = posFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_positionFactor],
        &hval);
}
double c_SIM_Spark_GetSimRelativeEncoderPositionFactor(
    c_SIM_Spark_RelativeEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_positionFactor],
        &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimRelativeEncoderVelocityFactor(
    c_SIM_Spark_RelativeEncoder_handle handle, double velFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_velocityFactor],
        &hval);
    hval.data.d = velFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_velocityFactor],
        &hval);
}
double c_SIM_Spark_GetSimRelativeEncoderVelocityFactor(
    c_SIM_Spark_RelativeEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_RelativeEncoderSignal_velocityFactor],
        &hval);
    return hval.data.d;
}

// Absolute Encoder
void c_SIM_Spark_SetSimAbsoluteEncoderPosition(
    c_SIM_Spark_AbsoluteEncoder_handle handle, double position) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_position], &hval);
    hval.data.d = position;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_position], &hval);
}
double c_SIM_Spark_GetSimAbsoluteEncoderPosition(
    c_SIM_Spark_AbsoluteEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_position], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAbsoluteEncoderVelocity(
    c_SIM_Spark_AbsoluteEncoder_handle handle, double velocity) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_velocity], &hval);
    hval.data.d = velocity;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_velocity], &hval);
}
double c_SIM_Spark_GetSimAbsoluteEncoderVelocity(
    c_SIM_Spark_AbsoluteEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_velocity], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAbsoluteEncoderInverted(
    c_SIM_Spark_AbsoluteEncoder_handle handle, bool inverted) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_isInverted], &hval);
    hval.data.b = static_cast<bool>(inverted);
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_isInverted], &hval);
}
bool c_SIM_Spark_GetSimAbsoluteEncoderInverted(
    c_SIM_Spark_AbsoluteEncoder_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_isInverted], &hval);
    return static_cast<bool>(hval.data.b);
}
void c_SIM_Spark_SetSimAbsoluteEncoderZeroOffset(
    c_SIM_Spark_AbsoluteEncoder_handle handle, double offset) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_zeroOffset], &hval);
    hval.data.d = offset;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_zeroOffset], &hval);
}
double c_SIM_Spark_GetSimAbsoluteEncoderZeroOffset(
    c_SIM_Spark_AbsoluteEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_zeroOffset], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAbsoluteEncoderPositionFactor(
    c_SIM_Spark_AbsoluteEncoder_handle handle, double posFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_positionFactor],
        &hval);
    hval.data.d = posFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_positionFactor],
        &hval);
}
double c_SIM_Spark_GetSimAbsoluteEncoderPositionFactor(
    c_SIM_Spark_AbsoluteEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_positionFactor],
        &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAbsoluteEncoderVelocityFactor(
    c_SIM_Spark_AbsoluteEncoder_handle handle, double velFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_velocityFactor],
        &hval);
    hval.data.d = velFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_velocityFactor],
        &hval);
}
double c_SIM_Spark_GetSimAbsoluteEncoderVelocityFactor(
    c_SIM_Spark_AbsoluteEncoder_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AbsoluteEncoderSignal_velocityFactor],
        &hval);
    return hval.data.d;
}

// Analog Sensor
void c_SIM_Spark_SetSimAnalogSensorVoltage(
    c_SIM_Spark_AnalogSensor_handle handle, double voltage) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_voltage], &hval);
    hval.data.d = voltage;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_voltage], &hval);
}
double c_SIM_Spark_GetSimAnalogSensorVoltage(
    c_SIM_Spark_AnalogSensor_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_voltage], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAnalogSensorPosition(
    c_SIM_Spark_AnalogSensor_handle handle, double position) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_position], &hval);
    hval.data.d = position;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_position], &hval);
}
double c_SIM_Spark_GetSimAnalogSensorPosition(
    c_SIM_Spark_AnalogSensor_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_position], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAnalogSensorVelocity(
    c_SIM_Spark_AnalogSensor_handle handle, double velocity) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_velocity], &hval);
    hval.data.d = velocity;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_velocity], &hval);
}
double c_SIM_Spark_GetSimAnalogSensorVelocity(
    c_SIM_Spark_AnalogSensor_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_velocity], &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAnalogSensorInverted(
    c_SIM_Spark_AnalogSensor_handle handle, bool inverted) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_isInverted], &hval);
    hval.data.b = static_cast<bool>(inverted);
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_isInverted], &hval);
}
bool c_SIM_Spark_GetSimAnalogSensorInverted(
    c_SIM_Spark_AnalogSensor_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_isInverted], &hval);
    return static_cast<bool>(hval.data.b);
}
void c_SIM_Spark_SetSimAnalogSensorPositionFactor(
    c_SIM_Spark_AnalogSensor_handle handle, double posFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_positionFactor],
        &hval);
    hval.data.d = posFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_positionFactor],
        &hval);
}
double c_SIM_Spark_GetSimAnalogSensorPositionFactor(
    c_SIM_Spark_AnalogSensor_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_positionFactor],
        &hval);
    return hval.data.d;
}
void c_SIM_Spark_SetSimAnalogSensorVelocityFactor(
    c_SIM_Spark_AnalogSensor_handle handle, double velFactor) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_velocityFactor],
        &hval);
    hval.data.d = velFactor;
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_velocityFactor],
        &hval);
}
double c_SIM_Spark_GetSimAnalogSensorVelocityFactor(
    c_SIM_Spark_AnalogSensor_handle handle) {
    if (handle == nullptr) return 0.0;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_AnalogSensorSignal_velocityFactor],
        &hval);
    return hval.data.d;
}

// Limit Switch
void c_SIM_Spark_SetSimLimitSwitchIsPressed(
    c_SIM_Spark_LimitSwitch_handle handle, bool pressed) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_LimitSwitchSignal_isPressed], &hval);
    hval.data.b = static_cast<bool>(pressed);
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_LimitSwitchSignal_isPressed], &hval);
}
bool c_SIM_Spark_GetSimLimitSwitchIsPressed(
    c_SIM_Spark_LimitSwitch_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_LimitSwitchSignal_isPressed], &hval);
    return static_cast<bool>(hval.data.b);
}
void c_SIM_Spark_SetSimLimitSwitchIsEnabled(
    c_SIM_Spark_LimitSwitch_handle handle, bool enabled) {
    if (handle == nullptr) return;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_LimitSwitchSignal_isEnabled], &hval);
    hval.data.b = static_cast<bool>(enabled);
    getREVLibDriver()->setSimValue(
        handle->m_signals[c_SIM_Spark_LimitSwitchSignal_isEnabled], &hval);
}
bool c_SIM_Spark_GetSimLimitSwitchIsEnabled(
    c_SIM_Spark_LimitSwitch_handle handle) {
    if (handle == nullptr) return false;
    REVLibValue hval;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_LimitSwitchSignal_isEnabled], &hval);
    return static_cast<bool>(hval.data.b);
}

bool c_SIM_Spark_GetSimLimitSwitchExists(c_SIM_Spark_handle handle,
                                         bool forward) {
    if (forward) {
        return handle->m_internalSignals.m_simForwardLimitSwitch != NULL;
    } else {
        return handle->m_internalSignals.m_simReverseLimitSwitch != NULL;
    }
}

// Fault Manager

void c_SIM_Spark_SetRawframeFromSimFaults(
    c_SIM_Spark_FaultManager_handle handle, c_Spark_PeriodicStatus1* rawframe) {
    if (handle == nullptr) return;
    struct REVLibValue hval;

    // man, this is ugly
    // I would try to setup a loop but to ensure things stay matched up this is
    // probably best

    // TODO(Landry): It's now already
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_otherFault], &hval);
    rawframe->otherFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_motorTypeFault],
        &hval);
    rawframe->motorTypeFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_sensorFault], &hval);
    rawframe->sensorFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_canFault], &hval);
    rawframe->canFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_temperatureFault],
        &hval);
    rawframe->temperatureFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_drvFault], &hval);
    rawframe->drvFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_escEepromFault],
        &hval);
    rawframe->escEepromFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_firmwareFault], &hval);
    rawframe->firmwareFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_brownoutWarning],
        &hval);
    rawframe->brownoutWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_overcurrentWarning],
        &hval);
    rawframe->overcurrentWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_escEepromWarning],
        &hval);
    rawframe->escEepromWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_extEepromWarning],
        &hval);
    rawframe->extEepromWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_sensorWarning], &hval);
    rawframe->sensorWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_stallWarning], &hval);
    rawframe->stallWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_hasResetWarning],
        &hval);
    rawframe->hasResetWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_otherWarning], &hval);
    rawframe->otherWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_otherStickyFault],
        &hval);
    rawframe->otherStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_motorTypeStickyFault],
        &hval);
    rawframe->motorTypeStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_sensorStickyFault],
        &hval);
    rawframe->sensorStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_canStickyFault],
        &hval);
    rawframe->canStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle
            ->m_signals[c_SIM_Spark_FaultManagerSignal_temperatureStickyFault],
        &hval);
    rawframe->temperatureStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_drvStickyFault],
        &hval);
    rawframe->drvStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_escEepromStickyFault],
        &hval);
    rawframe->escEepromStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_firmwareStickyFault],
        &hval);
    rawframe->firmwareStickyFault = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_brownoutStickyWarning],
        &hval);
    rawframe->brownoutStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals
            [c_SIM_Spark_FaultManagerSignal_overcurrentStickyWarning],
        &hval);
    rawframe->overcurrentStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle
            ->m_signals[c_SIM_Spark_FaultManagerSignal_escEepromStickyWarning],
        &hval);
    rawframe->escEepromStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle
            ->m_signals[c_SIM_Spark_FaultManagerSignal_extEepromStickyWarning],
        &hval);
    rawframe->extEepromStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_sensorStickyWarning],
        &hval);
    rawframe->sensorStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_stallStickyWarning],
        &hval);
    rawframe->stallStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_hasResetStickyWarning],
        &hval);
    rawframe->hasResetStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
    getREVLibDriver()->getSimValue(
        handle->m_signals[c_SIM_Spark_FaultManagerSignal_otherStickyWarning],
        &hval);
    rawframe->otherStickyWarning = static_cast<bool>(hval.data.b) ? 1 : 0;
}

void c_SIM_Spark_ClearSimFaults(c_SIM_Spark_FaultManager_handle handle) {
    for (int i = 0; i < c_SIM_Spark_FaultManagerSignal_NExternalSignals; i++) {
        struct REVLibValue tmp = c_SIM_Spark_CreateHALValue(
            c_SIM_Spark_FaultManagerSignals_Table[i].initialValue,
            c_SIM_Spark_FaultManagerSignals_Table[i].type);
        getREVLibDriver()->setSimValue(handle->m_signals[i], &tmp);
    }
}

bool c_SIM_Spark_GetSimFaultManagerExists(c_SIM_Spark_handle handle) {
    return handle->m_internalSignals.m_simFaultManager != NULL;
}

// Misc

float c_SIM_Spark_GetSimIAccum(c_SIM_Spark_handle handle) {
    return handle->m_internalSignals.iGain;
}

void c_SIM_Spark_SetSimIAccum(c_SIM_Spark_handle handle, float value) {
    handle->m_internalSignals.iGain = value;
}

}  // extern "C"
